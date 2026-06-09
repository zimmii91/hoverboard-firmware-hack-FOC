// Hoverboard_Control tab

// Include necessary libraries and define constants
#define HOVER_SERIAL_BAUD   115200      // Baud rate for HoverSerial (used to communicate with the hoverboard)
#define START_FRAME         0xABCD      // Start frame definition for reliable serial communication
#define TIME_SEND           100         // Sending time interval

#include <SoftwareSerial.h>
#include "Arduino.h"

int16_t mappedSteer = 0; // Define and initialize mappedSteer
int16_t mappedSpeed = 0; // Define and initialize mappedSpeed
extern bool horn;       // defined in RC_Remote_Control.h

// Define HoverSerial
SoftwareSerial HoverSerial(3,1);        // RX, TX

// Global variables
uint8_t idx = 0;  
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;



typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   int8_t   buzzer;
   uint8_t  reserved; // padding to match STM32 struct alignment
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

// Setup function
void setupHoverboard() {
  Serial.begin(115200);
  Serial.println("Hoverboard Serial v1.0");

  HoverSerial.begin(HOVER_SERIAL_BAUD);
  pinMode(2, OUTPUT);
}

// Send function
void Send(int16_t uSteer, int16_t uSpeed) {
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.buzzer   = (int8_t)horn; // maps horn switch to buzzer
  Command.reserved = 0;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed ^ Command.buzzer);

  // Write to Serial
  HoverSerial.write((uint8_t *) &Command, sizeof(Command)); 
}

// Receive function - parses incoming 18-byte hover feedback frames
void Receive() {
  if (!HoverSerial.available()) return;

  incomingByte  = HoverSerial.read();
  bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;

  if (bufStartFrame == START_FRAME) {
    p    = (byte *)&NewFeedback;
    *p++ = incomingBytePrev;
    *p++ = incomingByte;
    idx  = 2;
  } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {
    *p++ = incomingByte;
    idx++;
  }

  if (idx == sizeof(SerialFeedback)) {
    uint16_t checksum = (uint16_t)(
      NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^
      NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^
      NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
    }
    idx = 0;
  }
  incomingBytePrev = incomingByte;
}

// ---- CRSF telemetry helpers ----
extern HardwareSerial crsfSerial; // defined in main .ino

static uint8_t crsfCrc8(const uint8_t *buf, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (uint8_t j = 0; j < 8; j++)
      crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
  }
  return crc;
}

// Publish hover feedback as a CRSF battery sensor frame.
// batVoltage from firmware = calibrated voltage * 100 (e.g. 4200 = 42.00 V).
// CRSF battery voltage field = 0.1 V units, so divide by 10.
void publishTelemetry() {
  if (!Feedback.start) return; // no valid frame received yet

  uint16_t voltage_100mv = (uint16_t)(Feedback.batVoltage / 10);
  uint16_t current_100ma = 0;   // not available from hover firmware
  uint32_t capacity_mah  = 0;   // not tracked
  uint8_t  remaining     = 100; // not calculated

  // CRSF battery sensor frame (12 bytes total)
  uint8_t frame[12];
  frame[0]  = 0xC8; // CRSF_ADDRESS_FLIGHT_CONTROLLER
  frame[1]  = 10;   // length: type(1) + payload(8) + crc(1)
  frame[2]  = 0x08; // CRSF_FRAMETYPE_BATTERY_SENSOR
  frame[3]  = (voltage_100mv >> 8) & 0xFF;
  frame[4]  = voltage_100mv & 0xFF;
  frame[5]  = (current_100ma >> 8) & 0xFF;
  frame[6]  = current_100ma & 0xFF;
  frame[7]  = (capacity_mah >> 16) & 0xFF;
  frame[8]  = (capacity_mah >> 8)  & 0xFF;
  frame[9]  = capacity_mah & 0xFF;
  frame[10] = remaining;
  frame[11] = crsfCrc8(&frame[2], 9); // CRC covers type + payload
  crsfSerial.write(frame, sizeof(frame));
}

// Loop function
unsigned long iTimeSend = 0;

void ControllLoop() {
  // Parse any incoming hover feedback bytes
  Receive();

  // Send control command to hoverboard
  Send(mappedSteer, mappedSpeed);

  // Publish CRSF telemetry at ~10 Hz (every 100 ms)
  unsigned long now = millis();
  if (now - iTimeSend >= 100) {
    iTimeSend = now;
    publishTelemetry();
  }

  // Blink onboard LED
  digitalWrite(2, (millis() % 2000) < 1000);
}