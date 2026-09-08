#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// The journal remains until both files are published and activation succeeds.
// Recover before reading a slot or accepting another mutation after a reboot.
bool ww_store_recover(const char *base, unsigned slot);
typedef bool (*ww_store_check_fn)(unsigned slot, const char *model,
                                  const char *manifest, void *user);
bool ww_store_install(const char *base, unsigned slot,
                      const uint8_t *model, size_t model_len,
                      const char *manifest, size_t manifest_len,
                      ww_store_check_fn validate, ww_store_check_fn activate,
                      void *user);
