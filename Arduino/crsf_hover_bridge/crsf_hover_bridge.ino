// *******************************************************************
//  ESP32 learning example: hoverboard feedback -> CRSF telemetry bridge
//
//  This sketch is intentionally split into two parts:
//  1) Parse the hoverboard binary feedback frame from UART.
//  2) Show where CRSF telemetry frames would be published.
//
//  The CRSF transmit side below is a scaffold, not a full protocol stack.
//  Plug in your preferred CRSF library or frame encoder where marked.
// *******************************************************************

#include <Arduino.h>

#if !defined(ESP32)
#error This example is written for ESP32 because it expects multiple hardware UARTs.
#endif

#define HOVER_SERIAL_BAUD 115200
#define CRSF_SERIAL_BAUD  420000
#define START_FRAME       0xABCD

// Current hoverboard firmware feedback packet.
// Matches Src/main.c SerialFeedback.
typedef struct __attribute__((packed)) {
  uint16_t start;
  int16_t cmd1;
  int16_t cmd2;
  int16_t speedR_meas;
  int16_t speedL_meas;
  int16_t batVoltage;
  int16_t boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
} HoverFeedbackFrame;

static_assert(sizeof(HoverFeedbackFrame) == 18, "Hover feedback frame must be 18 bytes");

struct TelemetryState {
  int16_t cmd1 = 0;
  int16_t cmd2 = 0;
  int16_t speedRMeas = 0;
  int16_t speedLMeas = 0;
  int16_t batVoltage = 0;
  int16_t boardTemp = 0;
  uint16_t cmdLed = 0;
  bool valid = false;
};

HardwareSerial HoverSerial(2);
HardwareSerial CrsfSerial(1);

static TelemetryState telemetry;
static HoverFeedbackFrame frameBuffer;
static uint8_t *frameWritePtr = reinterpret_cast<uint8_t *>(&frameBuffer);
static size_t frameIndex = 0;
static uint8_t previousByte = 0;
static uint16_t rollingStart = 0;
static unsigned long lastTelemetryPrintMs = 0;

static uint16_t checksumFor(const HoverFeedbackFrame &frame) {
  return static_cast<uint16_t>(frame.start ^ frame.cmd1 ^ frame.cmd2 ^ frame.speedR_meas ^ frame.speedL_meas ^
                               frame.batVoltage ^ frame.boardTemp ^ frame.cmdLed);
}

static void resetFrameCapture() {
  frameIndex = 0;
}

static void acceptFeedbackFrame(const HoverFeedbackFrame &frame) {
  telemetry.cmd1 = frame.cmd1;
  telemetry.cmd2 = frame.cmd2;
  telemetry.speedRMeas = frame.speedR_meas;
  telemetry.speedLMeas = frame.speedL_meas;
  telemetry.batVoltage = frame.batVoltage;
  telemetry.boardTemp = frame.boardTemp;
  telemetry.cmdLed = frame.cmdLed;
  telemetry.valid = true;
}

static void feedHoverByte(uint8_t incomingByte) {
  rollingStart = static_cast<uint16_t>((incomingByte << 8) | previousByte);

  if (rollingStart == START_FRAME) {
    frameIndex = 0;
    frameWritePtr = reinterpret_cast<uint8_t *>(&frameBuffer);
    *frameWritePtr++ = previousByte;
    *frameWritePtr++ = incomingByte;
    frameIndex = 2;
  } else if (frameIndex >= 2 && frameIndex < sizeof(HoverFeedbackFrame)) {
    *frameWritePtr++ = incomingByte;
    frameIndex++;
  }

  if (frameIndex == sizeof(HoverFeedbackFrame)) {
    if (frameBuffer.start == START_FRAME && checksumFor(frameBuffer) == frameBuffer.checksum) {
      acceptFeedbackFrame(frameBuffer);
    }
    resetFrameCapture();
  }

  previousByte = incomingByte;
}

static void pollHoverboard() {
  while (HoverSerial.available() > 0) {
    feedHoverByte(static_cast<uint8_t>(HoverSerial.read()));
  }
}

static void publishCrsfTelemetry(const TelemetryState &state) {
  if (!state.valid) {
    return;
  }

  // Replace these debug prints with real CRSF telemetry frames.
  // Suggested mapping:
  // - batVoltage / 100 -> VFAS or battery voltage sensor
  // - boardTemp / 10   -> temperature sensor
  // - speedR/L_meas    -> RPM sensor or custom gauge
  // - cmdLed           -> optional status flags
  CrsfSerial.print("CRSF telemetry: V=");
  CrsfSerial.print(state.batVoltage / 100.0f, 2);
  CrsfSerial.print(" Temp=");
  CrsfSerial.print(state.boardTemp / 10.0f, 1);
  CrsfSerial.print(" RPM=");
  CrsfSerial.print((state.speedRMeas + state.speedLMeas) / 2);
  CrsfSerial.print(" CmdLED=");
  CrsfSerial.println(state.cmdLed);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Hoverboard to CRSF bridge example");

  HoverSerial.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, 16, 17);
  CrsfSerial.begin(CRSF_SERIAL_BAUD, SERIAL_8N1, 4, 5);

  Serial.println("Hover UART: RX=16 TX=17");
  Serial.println("CRSF UART:  RX=4 TX=5");
  Serial.println("Feedback frame: 18 bytes");
}

void loop() {
  pollHoverboard();

  const unsigned long now = millis();
  if (telemetry.valid && now - lastTelemetryPrintMs >= 100) {
    lastTelemetryPrintMs = now;
    publishCrsfTelemetry(telemetry);
  }

  // Control path would live here as well:
  // 1) read CRSF channel packets
  // 2) build hoverboard command frame
  // 3) send control frame to HoverSerial
}