// Log_to_SD.ino
// Manages the SD card, auto-incrementing filenames, and log writing.

#include "globals.h"


void initLogging() {
  // 2) Initialize VSPI on your pins
  //SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);
  SPI.setTX(SPI_MOSI);
  SPI.setRX(SPI_MISO);
  SPI.setSCK(SPI_SCK);
  SPI.begin();  // no arguments in Mbed core
  // 3) Mount the card at x MHz
  if (!sd.begin(SPI_CS, SD_SCK_MHZ(18))) {
    if (serialEnabled) {
      Serial.println("SdFat init failed");
      sd.printSdError(&Serial);  // <--- NEW
    }
    sdOK = false;

    return;
  }
  sdOK = true;
  delay(50);

  // 4) Find next file number & open it
  nextFileNumber = FindLowestNr();
  openLogFile(nextFileNumber);
}

void openLogFile(uint16_t num) {
  char fname[12];
  sprintf(fname, "%05u.txt", num);

  // O_CREAT|O_WRITE behaves like FILE_WRITE: create if needed, append
  currentLogFile = sd.open(fname, O_CREAT | O_WRITE);
  if (!currentLogFile) {
    if (serialEnabled) {
      Serial.print("Failed: ");
      Serial.println(fname);
    }
    sdOK = false;
    return;
  }
  if (serialEnabled) {
    Serial.println(fname);
  }
  // Write CSV header
  currentLogFile.println("millis,t,Text,m,Stable");
  currentLogFile.flush();
}


void handleSdWriteError() {
  ++sdRetryCount;
  if (sdRetryCount >= SD_MAX_RETRIES) {
    sdFatal = true;
    if (serialEnabled) {
      Serial.println(F("!!! SD write failure – logging disabled"));
    }
  } else {
    // close so that next call will re-open
    if (currentLogFile.isOpen()) currentLogFile.close();
  }
}



// Logs a transition (both to Serial and SD, for example)
void logTransition(unsigned long t, const DecodeResult &res) {
  //if (!sdFatal) { digitalWrite(LED_PIN, HIGH); }
  if (!sdOK || !currentLogFile.isOpen()) {
    if (sdRetryCount < SD_MAX_RETRIES) {
      ++sdRetryCount;

      // remount if needed
      if (!sdOK) {
        initLogging();  // sets sdOK internally
        if (sdOK) sdRetryCount = 0;
      }

      // reopen file if needed
      if (sdOK && !currentLogFile.isOpen()) {
        openLogFile(nextFileNumber);  // opens currentLogFile
        if (currentLogFile.isOpen()) sdRetryCount = 0;
      }
    }

    if (sdRetryCount >= SD_MAX_RETRIES && !sdFatal) {
      sdFatal = true;
      if (serialEnabled) {
        Serial.println(F("!!! SD failure – logging disabled"));
      }
    }
  }

  char timestamp[64] = "NoRTC";
  {
    struct timeval tv;
    gettimeofday(&tv, NULL);  // tv.tv_sec / tv.tv_usec now reflect your RTC + sub-seconds

    struct tm tm;
    gmtime_r(&tv.tv_sec, &tm);  // or localtime_r(&tv.tv_sec,&tm) if you prefer local TZ

    uint16_t ms = tv.tv_usec / 1000;  // 0…999

    snprintf(timestamp, sizeof(timestamp),
             "%04d-%02d-%02d %02d:%02d:%02d.%03u",
             tm.tm_year + 1900,
             tm.tm_mon + 1,
             tm.tm_mday,
             tm.tm_hour,
             tm.tm_min,
             tm.tm_sec,
             ms);
  }


  if (serialEnabled) {
    char tbuf[12];
    snprintf(tbuf, sizeof(tbuf), "%8lu", t);
    Serial.print(tbuf);
    Serial.print(",");
    Serial.print(timestamp);

    Serial.print(",");
    char vbuf[16];
    snprintf(vbuf, sizeof(vbuf), "%8.2f", res.value);
    Serial.print(vbuf);

    if (res.stable) {
      Serial.print(",  stable");
    } else {
      Serial.print(",unstable");
    }
    Serial.print(",");
    Serial.print(res.text);
    Serial.println();
  }

  //–– Build your full CSV line into one buffer ––
  if (!sdFatal) {
    char line[128];
    int len = snprintf(line, sizeof(line),
                       "%lu,%s,%s,%.2f,%u\n",
                       t,
                       timestamp,
                       res.text,
                       res.value,
                       res.stable ? 1 : 0);
    if (len < 0 || len >= (int)sizeof(line)) {
      // should never happen unless your buffer is too small
      return;
    }

    //–– Write it out in one shot and check the return ––
    int32_t written = currentLogFile.write((const uint8_t *)line, len);

    if (written != len || !currentLogFile.sync()) {
      handleSdWriteError();
      return;
    }

    //–– All good: reset retry counter if we’d been failing before ––
    sdRetryCount = 0;
  }
}

// Binary‐search the highest existing “NNNNN.txt” on card
uint16_t FindLowestNr() {
  uint32_t low = 0, high = 99999;
  while (low < high) {
    uint16_t mid = (low + high + 1) / 2;
    char testName[12];
    sprintf(testName, "%05u.txt", mid);
    if (sd.exists(testName)) {
      low = mid;
    } else {
      high = mid - 1;
    }
  }
  return low + 1;
}




// Buffering function: logs only the first and last values of a block.
// called each time a new decoded measurement is available.
void processDecodedResult(const DecodeResult &newRes, unsigned long newTime) {
  // ─── 0.  Detect “value changed” or “stable toggled” ──────────
  bool valueChanged = fabs(newRes.value - blockRes.value) >= TOLERANCE;
  bool stableChanged = (newRes.stable != blockRes.stable);
  bool anyChange = valueChanged || stableChanged;

  // If no block is active, start one:
  if (!blockActive) {
    blockActive = true;
    blockRes = newRes;  // start the block with the new measurement
    blockStartTime = newTime;
    blockEndTime = newTime;
    lastLoggedValue = newRes.value;
    holdUntil = newTime + HOLD_MS;   // open x s window to log every sample
    logTransition(newTime, newRes);  // log first reading
    return;
  }

  // ─── 2.  If we're inside the “hold” window, log every reading ─
  if (newTime < holdUntil) {
    // Only log if something actually changed (prevents duplicates)
    if (anyChange) {
      holdUntil = newTime + HOLD_MS;  // <-- extend window here
      lastLoggedValue = newRes.value;
    }
    logTransition(newTime, newRes);  // log unconditionally
    blockRes = newRes;               // keep block updated
    blockStartTime = newTime;
    blockEndTime = newTime;
    return;  // do NOT try to merge
  }


  // ─── 3.  No recent change; handle steady block aggregation ───
  if (!anyChange) {
    blockEndTime = newTime;
    if ((newTime - blockStartTime) >= BLOCK_TIMEOUT) {
      // flush once after timeout
      logTransition(blockEndTime, blockRes);
      blockStartTime = newTime;
      //blockActive = false;
    }
    return;
  }

  // log the new reading
  logTransition(newTime, newRes);
  lastLoggedValue = newRes.value;

  // start a fresh block & new hold window
  blockRes = newRes;
  blockStartTime = blockEndTime = newTime;
  holdUntil = newTime + HOLD_MS;
}

inline void blinkIfSDError() {
  if (!sdFatal) return;

  if (millis() - lastBlink >= BLINK_MS) {
    lastBlink = millis();
    static bool on = false;
    on = !on;
    digitalWrite(LED_PIN, on);
  }
}


void blinkIfReceive() {
  // these retain state across calls:
  static bool blinking = false;
  static uint32_t startMillis = 0;

  if (blink_receive_Requested && !blinking) {
    // start a new blink
    digitalWrite(LED_PIN, HIGH);
    startMillis = millis();
    blinking = true;
    blink_receive_Requested = false;
  }

  // turn it off after 100 ms
  if (blinking && (millis() - startMillis >= BLINK_RECEIVE_DURATION)) {
    digitalWrite(LED_PIN, LOW);
    blinking = false;
  }
}
