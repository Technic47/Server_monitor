# CLAUDE.md: Server Monitor (TFT35 V2.0 + ESP-01)

Guidance for Claude Code working in this repository. Read README.md first for the overview; this file holds what
you need to change the code safely.

## Working with the owner

- Answer in Russian, casual and concise. Practical steps beat theory.
- Deliver sketches as `.txt` files (he copies them more easily than `.ino`). Keep the `.ino` and `.txt` identical.
- Never put real secrets (Wi-Fi password, web password) in README or docs. Defaults live in the sketches
  (`DEF_SSID`, `DEF_PASS`, `WEB_PASS`, `AP_PASS`); do not print them in answers unless asked.
- Before changing a sketch, copy it to a `*_before_<topic>.bak` file next to it.
- No hardware here: you cannot run the code. Do a syntax check with host stubs (g++ `-fsyntax-only` with stub
  `Arduino.h`, `Adafruit_GFX.h`, ESP8266 headers). Ambiguous `String(long)` / missing `toCharArray` /
  `indexOf(String)` errors in stubs are stub artifacts, not real errors. Say clearly that nothing was run on hardware.

## Components

| Part | File | Target |
|---|---|---|
| Display firmware | `tft35_fsmc/tft35_fsmc.ino` (+ `build_opt.h`) | STM32F103VCT6, Arduino STM32 core 3.0.0, Adafruit GFX |
| ESP firmware | `esp01_monitor/esp01_monitor.ino` (FW_VERSION in file) | ESP8266 ESP-01, 1 MB flash, OTA |
| Server service | `docker-status/` | Python (stdlib) behind `tecnativa/docker-socket-proxy` |

Data path: Prometheus + docker-status -> (HTTP/Wi-Fi) ESP-01 -> (UART 115200) display. The display never talks
to the network. The ESP polls Prometheus every 3 s and docker-status every 10 s.

## UART protocol (keep both sides in sync)

Full table is in README.md. Rules when changing it:

- Lines end with `\n`; display line buffer is 200 chars, so keep lines short (CFG up to about 190).
- The display validates lines strictly and drops damaged ones; keep that behavior when adding new line types.
- `CFG:` carries `ssid=` LAST (an SSID may contain `;`). New fields go before it.
- ESP sends `ALERT:<name>[:<value>]` repeatedly while a condition holds (every cycle). The display beeps once per
  new alert name (4-slot memory) and holds an alert for 10 s after the last repeat. Keep alert names stable per
  condition, otherwise it re-beeps.
- Docker frame: `DK:` (counters, starts a frame), up to 24 `DKI:` lines, `DKEND:n` (commit). The display accepts a
  frame only if the number of `DKI:` lines equals `n`.
- ESP paces `DKI:` lines (3 ms apart) so the display RX buffer does not overflow.

## Display firmware notes (STM32)

- Core 3.0.0 UART: `Uart Dbg(PB11, PB10)` (debug, USART3), `Uart EspSerial(PA3, PA2)` (ESP, USART2). RX buffer
  64 B by default; `build_opt.h` with `-DSERIAL_RX_BUFFER_SIZE=256` is required.
- LCD: ILI9488 480x320 on FSMC 16-bit (own minimal `FsmcLcd : Adafruit_GFX`). Touch: XPT2046 bit-banged on
  PC0/PB3/PB4/PB5, pen on PC1. JTAG is disabled in `setup()` to free PB3/PB4 (SWD stays).
- Flash storage (emulated EEPROM in the last page): touch calibration at offset 0 (magic `0xCA1B0002`), battery
  calibration at offset 32 (magic `0xBA7C0001`). If the layout is extended, use a new offset and a new magic.
- Arduino auto-prototype pitfalls: no struct/class parameters in function signatures, no default arguments.
  Newlib Nano has no float `printf`: use `String(v, 1)` or `dtostrf`-style helpers.
- Main screen rows (top to bottom): CPU, RAM, GPU0, GPU1, SSD, mapped by `ROW_TEMP[]` / `ROW_LOAD[]`.
  Icons are drawn from primitives (`icoCpu`, `icoGpu`, `icoSsd`, `icoRam`, `icoDocker`).
- Screens: main, Menu (Touch Cal, ESP-01, Battery Cal, Docker), ESP-01 (status, Wi-Fi, Server, Reset),
  Wi-Fi (scan, keyboard), Docker (list, paging, Server), Battery Cal. Text drawing is slow: call `pollEsp()`
  between draws on screens that stay open.
- Switches at the top of the file: `BAT_ENABLED` (0 by default, USB power), `SLEEP_AFTER_MS` (0 = never blank),
  `BL_ENABLED` / `BL_PIN`, `SLEEP_LCD_CMD`.
- The backlight cannot be switched in software on this board (no LED pin in BTT sources; the panel flex has no
  accessible control). A black "sleep" does not save power. Do not promise power savings from sleep.
- Free pins on the board (found by the owner): PA0/PA1 (ENCODER, ADC-capable), PB10/PB11 (USART3), PA9/PA10
  (RS232), PB8/PB9 (TIM4), PD11/PD12 digital. Only PA0/PA1 are ADC.
- The display is flashed through ST-Link over SWD from Arduino IDE (Generic STM32F1 / Generic F103VCTx /
  STM32CubeProgrammer SWD). The BTT bootloader is gone; SD-card updates do not work.

## ESP firmware notes (ESP8266)

- EEPROM(2048) layout: `SrvCfg@0`, `WifiCfg@64`, `SensBlk@192` (4 temperature queries + alert thresholds),
  `LoadBlk@860` (5 load queries), `DkCfg@1680`. Each block has its own magic number and validity check, so older
  data is preserved when new blocks are added. Extend by appending at a new address, never by moving blocks.
- A new Wi-Fi network is stored only after a successful connection (20 s timeout, then it reverts). After
  60 s offline the ESP opens the fallback AP `TFT-Monitor-Setup`.
- OTA: ArduinoOTA (password `WEB_PASS`) and HTTP `/update` (`admin` / `WEB_PASS`). Flash size must be
  "1MB (FS:none OTA:~502KB)".
- Web page is built in one `String` and sent with a single `server.send()` (chunked output was very slow).
- Prometheus polling: `http.setTimeout(2000)`, one retry on link errors, 15 s backoff after a link failure.
  Sensor keys: temperatures `CPU GPU SSD P102`, loads `LCPU LGPU LSSD LP102 LRAM`. Query max length 159.
- Default PromQL strings are EXAMPLES (Linux node_exporter / nvidia_gpu_exporter style) and were not verified
  against the owner's exporters; real queries are set on the web page.
- docker-status client uses `http.getString()` (response about 1-2 KB). The service should send
  `Content-Length` and `Connection: close`; chunked encoding is tolerated but avoid it.

## Server side (docker-status)

- Format spec: first line `SUM total=N ok=N warn=N bad=N`, then `<code>|<seconds>|<restarts>|<exit>|<name>`
  sorted bad first, then warn, then ok. Codes `O S P U R X`. Names sanitized to `[A-Za-z0-9._-]`, max 30.
- Docker access only through `docker-socket-proxy` (read-only, `CONTAINERS=1`). Never mount the raw socket
  into the app.
- The host currently runs Windows 11 with Docker Desktop and may move to Linux. Keep everything
  cross-platform and containerized. `node_exporter` inside Docker on Windows sees the WSL2 VM, not the host.
- Docker Desktop on Windows: ports must be allowed in Windows Firewall for the LAN.

## Known quirks

- The owner's main Chrome (Kaspersky injecting a blocking script) loads the ESP web page very slowly; other
  browsers are instant. Not fixable in firmware.
- ESP-01 flashing by wire needs EN tied to 3V3 (forgetting it gave total silence), GPIO0 to GND only during
  flashing, TX/RX crossed. Serial Monitor must be closed during upload. Boot messages are at 74880 baud.
- HTTP code meanings in the ESP table: 200 with no data = query valid but no metric; 400 = bad PromQL;
  -1 = connection refused (port not published or firewall).

## Status and ideas

- Implemented and running in a 3D-printed stand: temperatures, load bars, Docker screen, alerts, Wi-Fi and
  server selection on the display, OTA, web settings page, USB power.
- Open / optional: nicer spacing of the GPU0/GPU1 labels next to the temperature value; sleeping the ESP
  (`MODE:SLEEP`) when the display is idle; more than 24 containers with paging from the ESP; NTP clock; flashing
  the display over Wi-Fi would need a custom bootloader (not started).
