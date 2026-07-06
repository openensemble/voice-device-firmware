# OpenEnsemble Voice Device Firmware

ESP32-S3 firmware that turns a **Seeed XIAO ESP32-S3** socketed onto a
**ReSpeaker XVF3800 4-Mic Array** into an [OpenEnsemble](https://github.com/openensemble/openensemble)
voice satellite: on-device wake words, far-field capture with hardware AEC,
and a WebSocket link to your OE server, which does all the heavy lifting
(STT, LLM, TTS).

This is the device half of OpenEnsemble's voice system — a fully open,
self-hosted alternative to an Alexa- or Google-Home-style smart speaker:
open hardware, open firmware, and a server you run yourself. Prebuilt
binaries ship with every OE install (with a browser-based flash wizard), so
you only need this repo to hack on the firmware itself. Overview:
[Voice devices](https://openensemble.github.io/openensemble/voice-devices).

## What it does

- **Wake words on-device** — [microWakeWord](https://github.com/kahrendt/microWakeWord)
  streaming models, 6 slots, per-user models pushed by the server over the
  WS `ww_upload` path. Stock image ships "Hey Ensemble" and "Hey Computer".
- **Voice turns** — wake → capture (XVF3800 beamformed + echo-cancelled) →
  utterance to OE `/api/stt` → reply streamed back as paced PCM frames over
  the WS and played out the speaker. Barge-in works during playback.
- **AirPlay 1 receiver** — stream music to the device from iOS/macOS;
  wake words stay live during playback.
- **Alarms & timers** — armed device-side so they fire even if Wi-Fi is down,
  with a server watchdog fallback.
- **Ambient audio, volume/pause voice intents, headphone/line-out mode.**
- **OTA updates** — server-driven `esp_https_ota` with dual app slots.
- **First-boot captive portal** — the device comes up as a Wi-Fi AP; a small
  form takes Wi-Fi credentials, the OE server URL, and a pairing code.

## Hardware

| Part | Role |
|---|---|
| Seeed XIAO ESP32-S3 | Runs this firmware |
| ReSpeaker XVF3800 4-Mic Array (I²S-master variant) | Mic array, AEC, beamforming, LEDs, speaker amp |
| 4–8 Ω speaker on the XVF3800 JST connector | Output |

The XVF3800 must be flashed once to its I²S-master firmware variant —
see [BUILD.md](BUILD.md) for the one-time `dfu-util` procedure, full build
steps, the pin map, and the component map.

## Building

ESP-IDF **v5.4**, target `esp32s3`:

```sh
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

CI builds every push via `.github/workflows/build.yml` and uploads the
flashable artifacts.

Most users don't need to build at all: the OpenEnsemble server ships
prebuilt binaries and a browser-based flash wizard
(Settings → Voice devices), and devices update themselves over OTA
afterwards.

## Release flow (maintainers)

1. Bump `PROJECT_VER` in the top-level `CMakeLists.txt` — this is the
   canonical version, baked into the app descriptor and reported to the
   server on WS auth.
2. `idf.py fullclean && idf.py build` — **fullclean matters**: an
   incremental build can leave the old version string baked into
   `esp_app_desc`.
3. Copy `build/oe_voice_device.bin` (and, if changed, `wakewords.bin`,
   `bootloader.bin`, `partition-table.bin`, `ota_data_initial.bin`) into the
   OE repo at `public/firmware/voice-device/` and set the same version in
   `manifest.json` there. Paired devices see the new version and offer OTA.

## Third-party code

| Component | Origin | License |
|---|---|---|
| `components/airplay/raop/` | AirPlay 1 (RAOP) bridge by [philippe44](https://github.com/philippe44) | MIT |
| `components/airplay/alac/` | Apple ALAC decoder | Apache-2.0 |
| `components/wakeword/upstream/` | Vendored from [ESPHome micro_wake_word](https://github.com/esphome/esphome/tree/dev/esphome/components/micro_wake_word) (see in-file headers for what was stripped) | Apache-2.0 |
| `main/xvf_firmware/*.bin` | XVF3800 I²S-master firmware from [Seeed's reSpeaker releases](https://github.com/respeaker/reSpeaker_XVF3800_USB_4MIC_ARRAY) | Seeed/XMOS |
| Managed components (TFLM, esp-nn, micro-speech features, libhelix-mp3, esp_websocket_client, mdns) | Pinned in `main/idf_component.yml`, fetched at configure time | Apache-2.0 / MIT |

Everything else is original OpenEnsemble code.

## Getting help

Questions, build problems, or hardware ideas? Ask in
[OpenEnsemble Discussions](https://github.com/openensemble/openensemble/discussions)
— the server and firmware share one community. Bugs specific to this
firmware are welcome in this repo's issues.

## License

AGPL-3.0, matching the OpenEnsemble server. Third-party components listed
above retain their own licenses.
