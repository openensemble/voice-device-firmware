#include "ww-store.h"
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>

typedef struct {
    char model[128], manifest[128], next_model[128], next_manifest[128];
    char old_model[128], old_manifest[128], journal[128];
} paths_t;

static bool paths(paths_t *p, const char *base, unsigned slot)
{
    if (!base || slot >= 6 || strlen(base) > 95) return false;
    snprintf(p->model, sizeof(p->model), "%s/slot%u.tflite", base, slot);
    snprintf(p->manifest, sizeof(p->manifest), "%s/slot%u.json", base, slot);
    snprintf(p->next_model, sizeof(p->next_model), "%s/slot%u.next.tflite", base, slot);
    snprintf(p->next_manifest, sizeof(p->next_manifest), "%s/slot%u.next.json", base, slot);
    snprintf(p->old_model, sizeof(p->old_model), "%s/slot%u.old.tflite", base, slot);
    snprintf(p->old_manifest, sizeof(p->old_manifest), "%s/slot%u.old.json", base, slot);
    snprintf(p->journal, sizeof(p->journal), "%s/slot%u.txn", base, slot);
    return true;
}

static bool exists(const char *p) { struct stat s; return stat(p, &s) == 0; }
static bool remove_file(const char *p) { return unlink(p) == 0 || errno == ENOENT; }
static bool write_file(const char *p, const void *data, size_t len)
{
    FILE *f = fopen(p, "wb");
    if (!f) return false;
    bool ok = fwrite(data, 1, len, f) == len;
    if (fflush(f) != 0 || fsync(fileno(f)) != 0) ok = false;
    if (fclose(f) != 0) ok = false;
    return ok;
}

static bool cleanup(const paths_t *p)
{
    return remove_file(p->next_model) && remove_file(p->next_manifest) &&
        remove_file(p->old_model) && remove_file(p->old_manifest);
}

static bool restore_file(const char *active, const char *backup, bool had_old)
{
    if (!had_old) return remove_file(active);
    if (!exists(backup)) return exists(active); // not moved, or already restored
    return remove_file(active) && rename(backup, active) == 0;
}

bool ww_store_recover(const char *base, unsigned slot)
{
    paths_t p;
    if (!paths(&p, base, slot)) return false;
    if (!exists(p.journal)) return cleanup(&p); // committed pair; only debris remains
    unsigned char mark[4] = {0};
    FILE *f = fopen(p.journal, "rb");
    if (!f) return false;
    size_t n = fread(mark, 1, sizeof(mark), f);
    fclose(f);
    if (n != sizeof(mark) || mark[0] != 'W' || mark[1] != 'W' ||
        mark[2] > 3 || mark[3] != (unsigned char)(mark[2] ^ 0xff)) {
        // Journal creation failed before any rename. Backups cannot exist
        // here during a valid transaction; if they do, require repair.
        if (exists(p.old_model) || exists(p.old_manifest)) return false;
        return remove_file(p.journal) && cleanup(&p);
    }
    if (!restore_file(p.model, p.old_model, (mark[2] & 1) != 0) ||
        !restore_file(p.manifest, p.old_manifest, (mark[2] & 2) != 0)) return false;
    if (!remove_file(p.journal)) return false;
    return cleanup(&p);
}

bool ww_store_install(const char *base, unsigned slot,
                      const uint8_t *model, size_t model_len,
                      const char *manifest, size_t manifest_len,
                      ww_store_check_fn validate, ww_store_check_fn activate,
                      void *user)
{
    paths_t p;
    if (!paths(&p, base, slot) || !model || !model_len || !manifest ||
        !manifest_len || !validate || !activate || !ww_store_recover(base, slot)) return false;
    if (!write_file(p.next_model, model, model_len) ||
        !write_file(p.next_manifest, manifest, manifest_len) ||
        !validate(slot, p.next_model, p.next_manifest, user)) {
        cleanup(&p);
        return false;
    }
    unsigned char flags = (exists(p.model) ? 1 : 0) | (exists(p.manifest) ? 2 : 0);
    unsigned char mark[] = {'W', 'W', flags, (unsigned char)(flags ^ 0xff)};
    // Only a completely flushed marker permits destructive renames. The old
    // pair survives until activation of the complete replacement succeeds.
    if (!write_file(p.journal, mark, sizeof(mark))) {
        ww_store_recover(base, slot);
        return false;
    }
    if (((flags & 1) && rename(p.model, p.old_model) != 0) ||
        ((flags & 2) && rename(p.manifest, p.old_manifest) != 0) ||
        rename(p.next_model, p.model) != 0 ||
        rename(p.next_manifest, p.manifest) != 0 ||
        !activate(slot, p.model, p.manifest, user) ||
        !remove_file(p.journal)) {
        ww_store_recover(base, slot);
        return false;
    }
    // The new pair is committed. Cleanup may be retried at boot and does not
    // turn a successful activation into a rollback.
    cleanup(&p);
    return true;
}
