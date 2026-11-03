# Repository Guidance

## Project

- PlatformIO project using the Arduino framework on an STM32 Blue Pill (`bluepill_f103c8`).
- The active environment is `bluepill_f103c8`, defined in `platformio.ini`.
- Firmware entry point is `src/main.cpp`. Project headers belong in `include/`; project-local libraries belong in `lib/`; PlatformIO tests belong in `test/`.
- Dependencies currently include Adafruit GFX, Adafruit SSD1306, and DFRobot RTU.

## Build and Upload

- Build: `pio run -e bluepill_f103c8`
- Upload to a connected board: `pio run -e bluepill_f103c8 -t upload`
- Run PlatformIO tests when tests exist: `pio test -e bluepill_f103c8`
- Do not assume a board, serial port, or upload method beyond the configured environment; check `platformio.ini` and the user's setup before changing upload configuration.

## Firmware Conventions

- Follow the existing Arduino/C++ style and keep changes focused on the relevant firmware behavior.
- Preserve the existing pin assignments and the RS485, stepper, encoder, and OLED wiring unless the task explicitly changes the hardware interface.
- The firmware uses interrupt-driven Step/Dir input. Keep interrupt handlers short and safe for interrupt context; avoid blocking operations and non-atomic shared-state changes there.
- The firmware and comments currently use Bulgarian in places. Preserve the surrounding language when editing existing text.
- Add or update PlatformIO tests for behavior that can be isolated from hardware; clearly call out hardware-only behavior that cannot be validated by host tests.