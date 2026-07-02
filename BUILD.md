# Building the OE voice-device firmware

Target: **Seeed XIAO ESP32-S3** socketed onto a **ReSpeaker XVF3800 4-Mic Array** carrier.

## Prerequisites

1. ESP-IDF v5.4 or later, installed and sourced (`. $HOME/esp/esp-idf/export.sh`).
2. The XVF3800 board must be running the **I²S-master** firmware variant, not the factory USB variant. See "XVF3800 prep" below — this is a one-time step done with `dfu-util` over the on-board USB-C port before the I²S path works.
3. A 4–8 Ω speaker plugged into the JST speaker connector on the XVF3800 board.

## XVF3800 prep (do this once)

> **If the board doesn't enumerate, or `dfu-util` reports "no device found":** unplug, then **press and hold the Mute button on the XVF3800 while plugging USB-C back in.** Keep holding until `dfu-util --list` shows VID `0x20b1`. The board only exposes the DFU interface in this mode — a plain plug-in lands in the runtime USB-audio variant, where flashing silently fails.

```sh
# Hold the Mute button on the XVF3800 while plugging in USB-C to enter DFU.
# Download the binary from Seeed's GitHub releases:
#   github.com/respeaker/reSpeaker_XVF3800_USB_4MIC_ARRAY → releases
# Find: respeaker_xvf3800_i2s_master_dfu_firmware_v1.0.x_48k.bin

dfu-util --list                       # should show "Found DFU: [20b1:..." — if not, redo the mute-plug step
dfu-util -d 0x20b1: -a 0 -D respeaker_xvf3800_i2s_master_dfu_firmware_v1.0.x_48k.bin -R
```

After this the XVF3800 outputs I²S to GPIO43 of the socketed XIAO and accepts I²S input on GPIO44.

## Build + flash

```sh
cd firmware/voice-device
idf.py set-target esp32s3
idf.py menuconfig    # optional — defaults in sdkconfig.defaults are tuned for XIAO
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

First boot will come up as a Wi-Fi AP named `oe-voice-XXXX`. Join the AP, fill in the captive portal form (Wi-Fi creds + OE server URL + an 8-char pairing code generated from OE Settings → Devices), the device redeems the code, then reboots into operational mode.

## Wake-word models

The `wakewords` partition is an SPIFFS image at `/ww/`. Slot files are named `slot0.tflite`, `slot1.tflite`, etc.

To build the wake-word image:

```sh
python -m esptool --chip esp32s3 \
    write_flash 0x620000 build/wakewords.bin
```

Generate `build/wakewords.bin` with `spiffsgen.py` from the IDF tools:

```sh
python $IDF_PATH/components/spiffs/spiffsgen.py 0x80000 wakewords/ build/wakewords.bin
```

`wakewords/` is a directory of `slotN.tflite` + `slotN.json` pairs. The stock image ships `slot0` = "Hey Ensemble" and `slot1` = "Hey Computer"; the firmware supports 6 slots (`WW_NUM_SLOTS`), and OE replaces slot contents per-user over the WS `ww_upload` path after pairing — a USB reflash restores the stock models until the next server sync.

## Component map

| Component | Purpose |
|---|---|
| `main` | State machine (`main.c`) + shared types (`state.h`) |
| `audio_io` | Full-duplex I²S RX/TX wrappers, 48k↔16k sample-rate conversion |
| `airplay` | AirPlay 1 receiver (raop bridge by philippe44, MIT; Apple ALAC decoder, Apache-2.0) |
| `alarm` | Device-side alarm arm/fire/ack with server watchdog handshake |
| `xvf3800_ctrl` | I²C control surface: LEDs, mute, DoA, VAD probability, beam config |
| `wakeword` | microWakeWord runtime (loads `slotN.tflite` from `/ww` SPIFFS) |
| `vad` | Energy-threshold + silence-timeout end-of-utterance detector |
| `mp3_decode` | libhelix MP3 → 16-bit PCM, mono mixdown |
| `oe_client` | `/api/devices/redeem`, `/ws` auth+chat, `/api/stt`, `/api/tts` |
| `captive_portal` | First-boot AP + tiny HTML form + DNS hijack |
| `nvs_creds` | Persisted Wi-Fi/server/token/device config |
| `leds_buttons` | UI state → XVF LED pattern, mute-switch polling |

## Pin map (XIAO → XVF3800 via socket)

| Signal | XIAO silk | XIAO GPIO | XVF3800 role |
|---|---|---|---|
| I²C SDA | D4 | GPIO5 | I²C slave SDA, addr 0x2C |
| I²C SCL | D5 | GPIO6 | I²C slave SCL |
| I²S WS | D8 | GPIO7 | I²S WS (XVF master) |
| I²S BCK | D9 | GPIO8 | I²S BCK (XVF master) |
| I²S RX | D6 | GPIO43 | I²S DOUT from XVF (mic audio in) |
| I²S TX | D7 | GPIO44 | I²S DIN to XVF (TTS audio out, also AEC reference) |
| XVF reset | — | GPIO9 | Hard-reset line |

## Known gaps / open items

1. **TLS cert handling.** Right now `skip_cert_common_name_check = true` for all
   HTTPS calls so a self-signed OE server works for development. Production: pin
   the server cert via `nvs_creds_set_server_cert` during pairing and feed it
   into `esp_http_client_config_t.cert_pem`.

2. **AEC tuning levels.** The XVF3800 ships with conservative defaults. If
   barge-in is sluggish, tweak `AUDIO_MGR_REF_GAIN`, `AUDIO_MGR_MIC_GAIN`,
   `AUDIO_MGR_SYS_DELAY` via `xvf3800_raw_set()` — but read the warnings in
   `xvf3800_ctrl.c` first, and never SAVE_CONFIGURATION while experimenting.

3. **Legacy sentence chunker is byte-level.** When the server-side TTS
   streaming path isn't active (older servers / non-Pocket TTS providers),
   `main.c::accumulate_token` splits on `.`/`!`/`?` followed by whitespace and
   will mis-split abbreviations like "Dr. Smith". The streaming path
   (`tts_audio_*` frames) doesn't use it.

Resolved since the original bring-up: microWakeWord frontend + TFLM glue are
vendored and live (`components/wakeword/`), OTA is wired end-to-end
(`esp_https_ota`, dual app slots, server-driven via the `ota_check` WS
message), and TTS is streamed from the server as paced PCM frames over the
WS instead of per-sentence MP3 pulls.

## Day-one bring-up order

1. DFU-flash the XVF3800 I²S-master image.
2. `arecord` test on a Linux PC (still USB-bus-powered) to confirm the board is alive: should show 4 channels of mic input.
3. Socket the XIAO. `idf.py flash monitor` from this directory.
4. Verify `XVF3800 I²C bus up` and `I²S slave mode` lines in the boot log.
5. Join the `oe-voice-XXXX` AP from a phone, complete captive portal.
6. After auto-reboot, verify `ws connected` and `wake word slot 0 loaded`.
7. Say the wake word — `wake!` line in log; LED ring goes blue/listening.
8. Speak — `stt: "..."` line in log; LED → thinking; TTS reply through speaker.
