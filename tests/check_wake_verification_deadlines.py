#!/usr/bin/env python3
"""Exercise production provisional-wake capture and watchdog with a fake clock."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / 'main/main.c').read_text()
HEADER = (ROOT / 'components/oe_client/include/oe_client.h').read_text()


def between(source, start, end):
    offset = source.index(start)
    return source[offset:source.index(end, offset)]


defines = '\n'.join(re.findall(
    r'^#define (?:VG_\w+|PREROLL_SAMPLES|CAPTURE_MAX_UTTERANCE_MS|CAPTURE_BUFFER_SAMPLES) .+$',
    MAIN, re.M))
defines += '\n' + re.search(r'^#define OE_VERIFY_GATE_TIMEOUT_MS .+$', HEADER, re.M).group()
result_type = between(HEADER, 'typedef enum {\n    OE_VERIFY_ACCEPT', '// server_url')
declarations = between(MAIN, 'typedef enum { PROV_NONE', '// Playback hold snapshot.')
step = between(MAIN, 'static vad_end_reason_t vg_provisional_step(', 'static void capture_and_drive_task(')
watchdog = between(MAIN, '        if (s_verify_inflight && s_verify_inflight_since_us != 0 &&',
                   '        // Clear a gate-fault indication')

code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "vad.h"
#define WW_SAMPLE_RATE 16000
#define WW_FRAME_SAMPLES 160
#define OE_URL_MAX 256
#define OE_DEVICE_ID_MAX 64
#define WW_WAKE_SLUG_MAX 40
#define pdTRUE 1
#define ESP_OK 0
#define UI_STATE_IDLE 0
#define UI_STATE_LISTENING 1
typedef void *SemaphoreHandle_t;
typedef void *TaskHandle_t;
''' + defines + '\n' + result_type + declarations + r'''
static int64_t clock_us;
static bool muted, alarm, connected, s_verify_cancelled, s_in_utterance;
static bool s_stt_streaming, s_stt_send_failed, s_caps_stt_stream;
static int commits, rollbacks, accepted, dispatches, aborts, replays, ui;
static size_t s_capture_used, replay_samples;
static uint32_t s_stt_seq;
static int s_active_slot, s_active_wake_prob;
static char s_prov_turn_id[24], s_turn_id[24], s_active_wake_slug[40];
static int16_t s_capture_buf[CAPTURE_BUFFER_SAMPLES], wake_window[VG_WIN_SAMPLES];
static vad_state_t *s_vad;
static vad_end_reason_t next_end;
static struct {char server_url[OE_URL_MAX], default_agent_id[64];} g_dev_config;
static int64_t esp_timer_get_time(void){return clock_us;}
static bool alarm_is_firing(void){return alarm;}
static bool leds_buttons_is_muted(void){return muted;}
static bool oe_ws_connected(void){return connected;}
bool vad_feed(vad_state_t *v, const int16_t *frame, size_t n, vad_end_reason_t *end){*end=next_end;next_end=VAD_END_NONE;return true;}
static void oe_udplog_send(const char *s){}
static void udplog_turn(const char *s,const char *id){}
static void set_ui_state(int state){ui=state;}
static void vg_restore_rejected_identity(void){}
static void vg_playback_hold_rollback(void){rollbacks++;}
static bool vg_playback_hold_commit(void){commits++;return true;}
static bool vg_lifecycle_take(void){return true;}
static void vg_lifecycle_give(void){}
static void airplay_resume(void){}
static void ambient_resume_locked(bool v){}
static void vg_accept_identity(void){accepted++;strcpy(s_turn_id,s_prov_turn_id);}
static void verify_gate_config_snapshot(char *path,char *id){strcpy(path,"/api/voice-gate/v1/verify");strcpy(id,"vdev_test");}
static int xSemaphoreGive(SemaphoreHandle_t s){dispatches++;return pdTRUE;}
static int oe_ws_send_stt_backlog(const char *turn,int slot,int prob,const char *agent,const int16_t *pcm,size_t n,int budget,uint32_t *seq){
    assert(strcmp(turn,"wake-test")==0);assert(n && pcm[0]==300 && pcm[n-1]==300);
    replay_samples=n;replays++;return ESP_OK;
}
static void oe_verify_gate_abort(void){aborts++;}
''' + step + '\nstatic void watchdog(void){\n' + watchdog + r'''
}
static int64_t begin(void){
    clock_us=1000000;muted=alarm=s_verify_cancelled=false;connected=true;
    commits=rollbacks=accepted=dispatches=aborts=replays=0;replay_samples=0;
    s_capture_used=0;s_in_utterance=true;s_caps_stt_stream=true;
    s_verify_done=s_verify_inflight=false;s_prov_state=PROV_FILL_TAIL;
    s_prov_vad_end=next_end=VAD_END_NONE;s_verify_wav=wake_window;
    s_verify_pre_len=s_verify_win_len=VG_PRE_SAMPLES;
    strcpy(s_prov_turn_id,"wake-test");strcpy(s_active_wake_slug,"sophia");
    strcpy(g_dev_config.server_url,"http://127.0.0.1:3737");
    s_turn_id[0]=s_verify_res_turn[0]=0;
    int16_t frame[WW_FRAME_SAMPLES];
    for(size_t i=0;i<WW_FRAME_SAMPLES;i++)frame[i]=300;
    while(s_prov_state==PROV_FILL_TAIL){
        clock_us+=10000;assert(vg_provisional_step(frame,WW_FRAME_SAMPLES)==VAD_END_NONE);
    }
    assert(dispatches==1 && s_verify_inflight && s_prov_state==PROV_AWAIT_VERDICT);
    return clock_us;
}
static vad_end_reason_t tick(int64_t at){
    clock_us=at;int16_t frame[WW_FRAME_SAMPLES];
    for(size_t i=0;i<WW_FRAME_SAMPLES;i++)frame[i]=300;
    return vg_provisional_step(frame,WW_FRAME_SAMPLES);
}
static void publish(oe_verify_result_t verdict,const char *turn){
    s_verify_effective=verdict;strcpy(s_verify_res_turn,turn);s_verify_done=true;s_verify_inflight=false;
}
int main(void){
    int64_t start=begin();
    next_end=VAD_END_SILENCE;assert(tick(start+100000)==VAD_END_NONE);
    const size_t complete_command=s_capture_used;
    assert(tick(start+2400000)==VAD_END_NONE);
    assert(s_prov_state==PROV_AWAIT_VERDICT && s_capture_used==complete_command);
    assert(commits==0 && rollbacks==0 && replays==0);
    publish(OE_VERIFY_ACCEPT,"wake-test");
    assert(tick(start+2500000)==VAD_END_SILENCE);
    assert(commits==1 && accepted==1 && rollbacks==0 && replays==1);
    assert(replay_samples==complete_command && s_capture_used==complete_command && s_prov_state==PROV_NONE);

    start=begin();assert(tick(start+3600000)==VAD_END_NONE);
    assert(s_prov_state==PROV_AWAIT_VERDICT && rollbacks==0);
    assert(tick(start+3900000)==VAD_END_NONE);
    assert(s_prov_state==PROV_NONE && rollbacks==1 && commits==0 && s_capture_used==0);
    clock_us=start+5900000;watchdog();assert(aborts==0);
    clock_us=start+6100000;watchdog();assert(aborts==1);

    start=begin();publish(OE_VERIFY_REJECT,"wake-test");tick(start+2000000);
    assert(s_prov_state==PROV_NONE && rollbacks==1 && commits==0 && s_capture_used==0);

    start=begin();publish(OE_VERIFY_ACCEPT,"superseded-turn");tick(start+2500000);
    assert(s_prov_state==PROV_AWAIT_VERDICT && commits==0);
    tick(start+3900000);assert(s_prov_state==PROV_NONE && rollbacks==1);

    start=begin();muted=true;publish(OE_VERIFY_ACCEPT,"wake-test");tick(start+2000000);
    assert(s_prov_state==PROV_NONE && rollbacks==1 && commits==0);
    puts("Wake verification: delayed accept preserves completed command; timeout, reject, stale verdict, mute and watchdog pass");
}
'''

with tempfile.TemporaryDirectory(prefix='oe-wake-deadlines-') as directory:
    source = Path(directory) / 'wake.c'
    executable = Path(directory) / 'wake'
    source.write_text(code)
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Wno-unused-parameter',
                    '-Wno-unused-variable', '-I', str(ROOT / 'components/vad/include'),
                    str(source), '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
