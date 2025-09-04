// RTC_handler.ino
// DS3231 code plus the non-blocking serial time logic

#include "globals.h"



void initRTC() {
  // --- HW-I²C for RTC ----------------------------------------------------
  Wire.setSDA(SDA_RTC_PIN);
  Wire.setSCL(SCL_RTC_PIN);
  Wire.begin();
  Wire.setClock(I2C_RTC_FREQ);  // 400 kHz
  time_t t = readRTCTime();     // uses Wire1

  if (t < 86400) {  // epoch or error (< 1 day)
    Serial.println("RTC not found or returns epoch");
    rtcInitialized = false;
    return;
  }
  rtcInitialized = true;



  UTC_offset = 1;  // Determine UTC_offset from our timestamp array
  for (size_t i = 0; i < sizeof(timestampDataArray) / sizeof(timestampDataArray[0]); i++) {
    if (t > timestampDataArray[i].timestamp) {
      UTC_offset = timestampDataArray[i].offset;
    }
  }
  // Adjust the pending time by the offset (in seconds):
  t += UTC_offset * 3600;

  // --- sync the C runtime’s clock to the RTC seconds ---
  {
    struct timeval tv;
    tv.tv_sec = t;   // calendar seconds since epoch
    tv.tv_usec = 0;  // start sub-second at zero
    settimeofday(&tv, nullptr);
  }


  if (serialEnabled) {
    char TimeStr[40];
    sprintf(TimeStr, "%04d-%02d-%02d %02d:%02d:%02d",
            year(t), month(t), day(t),
            hour(t), minute(t), second(t));

    //Serial.println("RTC time:");
    Serial.println(TimeStr);
  }
}


// Non-blocking approach: we read from serial each loop
// If we find a line "YYYY-MM-DD HH:MM:SS", store in pendingDT
void processSerialForTimeNonBlocking() {
  static String inputLine;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputLine.length() >= 19) {
        // Expecting a string like "2023-10-03 15:23:45"
        int y = inputLine.substring(0, 4).toInt();
        int mo = inputLine.substring(5, 7).toInt();
        int d = inputLine.substring(8, 10).toInt();
        int hh = inputLine.substring(11, 13).toInt();
        int mm = inputLine.substring(14, 16).toInt();
        int ss = inputLine.substring(17, 19).toInt();
        if (y > 2000 && mo >= 1 && mo <= 12 && d >= 1 && d <= 31) {
          // Use TimeLib to convert to time_t.
          tmElements_t tm;
          tm.Year = y - 1970;  // TimeLib expects year offset from 1970
          tm.Month = mo;
          tm.Day = d;
          tm.Hour = hh;
          tm.Minute = mm;
          tm.Second = ss;
          pendingTime = makeTime(tm);
          // Determine UTC_offset from our timestamp array:
          UTC_offset = 1;
          for (size_t i = 0; i < sizeof(timestampDataArray) / sizeof(timestampDataArray[0]); i++) {
            if (pendingTime > timestampDataArray[i].timestamp) {
              UTC_offset = timestampDataArray[i].offset;
            }
          }
          // Adjust the pending time by the offset (in seconds):
          pendingTime -= UTC_offset * 3600;


          havePendingTime = true;
          //applyPendingTime();  //2025-02-01 15:44:00
          Serial.print("Parsed new time: ");
          Serial.println(inputLine);

        } else {
          Serial.print("Invalid parse in: ");
          Serial.println(inputLine);
        }
      }
      inputLine = "";
    } else {
      inputLine += c;
    }
  }
}


void applyPendingTime() {
  if (!rtcInitialized || !havePendingTime) return;
  havePendingTime = false;

  /* ---------- 1. grab the old RTC value before we overwrite it ---------- */
  time_t oldTime = readRTCTime();



  // Get the pending time (from Serial) as t:
  time_t t = pendingTime;
  // Update the RTC:
  writeRTCTime(t);
  // Determine UTC_offset from our timestamp array:
  UTC_offset = 1;
  for (size_t i = 0; i < sizeof(timestampDataArray) / sizeof(timestampDataArray[0]); i++) {
    if (t > timestampDataArray[i].timestamp) {
      UTC_offset = timestampDataArray[i].offset;
    }
  }
  // Adjust the pending time by the offset (in seconds):
  oldTime += UTC_offset * 3600;
  t += UTC_offset * 3600;

  /* ---------- 5. pretty-print both stamps ---------- */
  char oldTimeStr[32];
  char newTimeStr[32];
  sprintf(oldTimeStr, "%04d-%02d-%02d %02d:%02d:%02d",
          year(oldTime), month(oldTime), day(oldTime),
          hour(oldTime), minute(oldTime), second(oldTime));
  sprintf(newTimeStr, "%04d-%02d-%02d %02d:%02d:%02d",
          year(t), month(t), day(t),
          hour(t), minute(t), second(t));

  // Now re-sync the C runtime’s time
  {
    struct timeval tv;
    tv.tv_sec = t;
    tv.tv_usec = 0;
    settimeofday(&tv, nullptr);
  }

  if (serialEnabled) {
    Serial.println("RTC time changed from -> to:");
    Serial.println(oldTimeStr);
    Serial.println(newTimeStr);
  }
}

// BCD conversion
static inline uint8_t bin2bcd(uint8_t v) {
  return ((v / 10) << 4) | (v % 10);
}
static inline uint8_t bcd2bin(uint8_t v) {
  return (v >> 4) * 10 + (v & 0x0F);
}



time_t readRTCTime() {
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(0x00);             // start at REG_TIME
  Wire.endTransmission(false);  // repeated-start
  Wire.requestFrom(DS3231_ADDRESS, 7);

  uint8_t ss = Wire.read();
  uint8_t mm = Wire.read();
  uint8_t hh = Wire.read();
  Wire.read();  // skip DOW
  uint8_t dd = Wire.read();
  uint8_t mo = Wire.read();
  uint8_t yy = Wire.read();

  tmElements_t tm;
  tm.Second = bcd2bin(ss & 0x7F);
  tm.Minute = bcd2bin(mm);
  tm.Hour = bcd2bin(hh & 0x3F);
  tm.Day = bcd2bin(dd);
  tm.Month = bcd2bin(mo & 0x1F);
  tm.Year = bcd2bin(yy) + (2000 - 1970);
  return makeTime(tm);
}

void writeRTCTime(time_t t) {
  tmElements_t tm;
  breakTime(t, tm);
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(0x00);
  Wire.write(bin2bcd(tm.Second));
  Wire.write(bin2bcd(tm.Minute));
  Wire.write(bin2bcd(tm.Hour));
  Wire.write(bin2bcd(tm.Wday));
  Wire.write(bin2bcd(tm.Day));
  Wire.write(bin2bcd(tm.Month));
  Wire.write(bin2bcd((tm.Year + 1970) - 2000));
  Wire.endTransmission();

  // clear OSF
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(REG_STATUS);
  Wire.endTransmission(false);
  Wire.requestFrom(DS3231_ADDRESS, 1);
  uint8_t stat = Wire.read() & ~0x80;
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(REG_STATUS);
  Wire.write(stat);
  Wire.endTransmission();
}