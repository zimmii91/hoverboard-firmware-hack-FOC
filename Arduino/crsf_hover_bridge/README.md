# CRSF Hover Bridge Example

This example shows the shape of a two-way ESP32 bridge:

1. Read CRSF control input from the receiver.
2. Send hoverboard control frames over UART.
3. Parse hoverboard feedback frames.
4. Map hover feedback into CRSF telemetry.

The included sketch focuses on steps 3 and 4 so you can see the packet flow clearly without depending on a CRSF library.

## What the example demonstrates

- Hover feedback is parsed as a fixed 18-byte binary frame.
- Battery voltage, temperature, and motor speed are kept in a local telemetry state.
- The `publishCrsfTelemetry()` function shows where CRSF telemetry frames would be sent.

## Suggested telemetry mapping

- Battery voltage: `batVoltage / 100.0`
- Temperature: `boardTemp / 10.0`
- RPM: average of left and right measured RPM
- Status: `cmdLed` or a custom flag field

## Next step when you wire this into your real project

Replace the debug print section inside `publishCrsfTelemetry()` with a real CRSF telemetry encoder or library call. Keep the hover feedback parser as-is if your firmware packet format matches the current repository.