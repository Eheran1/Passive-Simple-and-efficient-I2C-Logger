/*
RP2040 (Raspberry Pi Pico)
Install info: https://arduino-pico.readthedocs.io/en/latest/install.html#installing-via-arduino-boards-manager
Board manager: Raspberry Pi Pico Arduino core (Earle F. Philhower)

Set RTC: send over serial "YYYY-MM-DD HH:MM:SS" eg. "2025-02-01 01:22:40"
*/

//main.ino
#include "globals.h"

// Forward declarations of functions in other .ino files
void initSniffer();
void initRTC();
void initLogging();
float getLatestMeasurementValue();       // returns the most recent measurement
void processSerialForTimeNonBlocking();  // checks serial for time input, non-blocking
void applyPendingTime();                 // sets the RTC from the parsed time
void handleSniffer();                    // handle completed messages
void logData(const DecodeResult& res);
uint16_t FindLowestNr();
void onI2CReceive(int numBytes);
void onI2CRequest();
DecodeResult decodePCFData(const uint8_t* data, size_t len);
void openLogFile(uint16_t num);
void onReceive(int len);
void onRequest(void);

time_t readRTCTime();
void writeRTCTime(time_t t);
void blinkIfSDError();
void blinkIfReceive();



void setup() {
  watchdog_enable(6000, 1);  // 2 seconds watchdog
  pinMode(SPI_CS, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  Serial.begin(115200);

  while (!Serial && millis() < 4000) {}  // wait up to 4 s for a host
  watchdog_update();
  delay(500);
  Serial.println("USB connected");

  initRTC();

  initSniffer();
  initLogging();


  if (serialEnabled) {
    Serial.println("Set time via input: YYYY-MM-DD HH:MM:SS");
  }
  delay(500);
  //Serial.println("Setup finished.");
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  watchdog_update();
  // 1) For the first 10 seconds after power-up, we watch for date/time on Serial.
  //    We'll do this by recording the time on the first call.
  static unsigned long startupMillis = millis();

  if (serialEnabled) {
    // Non-blocking check for date/time on Serial, do not block
    processSerialForTimeNonBlocking();
  }
  if (havePendingTime) {
    applyPendingTime();  // got time via serial
  }


  // read all available PIO words
  while (!pio_sm_is_rx_fifo_empty(pio, SM_MAIN)) {
    uint32_t w = pio_sm_get(pio, SM_MAIN);
    decodeAndBuffer(w);
  }
  // sniff and handle immediately:
  handleSniffer();


  blinkIfSDError();
  blinkIfReceive();
}
