/*
LogBoy - ESP32 firmware
- 128x64 SSD1306 I2C OLED (0x3C)
- ADS1115 for battery / AIN1 / AIN0 readings
- SD_MMC 1-bit for SD logging (Optimized: File stays OPEN)
- WiFi + web UI
- 5 interactive modes via UP/DOWN/ENTER buttons

Notes:
- IMPORTANT: Calibrate AIN1_DIVIDER and CURRENT_SENSITIVITY constants below for accurate readings.
- AIN2 (Battery Voltage) now assumes NO divider.
- OLED display simplified to status only (No logs/history) for performance.

Libraries required:
- Adafruit SSD1306
- Adafruit GFX
- Adafruit ADS1X15
- SD_MMC (built-in for ESP32)

Target: ESP32 (Core >= 2.0)
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_ADS1X15.h>
#include <WiFi.h>
#include <WebServer.h>
#include "SD_MMC.h"

// ---------------- PINS & HARDWARE ----------------
#define BUTTON_UP 15
#define BUTTON_ENTER 21
#define BUTTON_DOWN 14

#define LED_115200 9
#define LED_38400 10
#define LED_9600 11
#define LED_19200 12
#define LED_MODE 13

#define UART2_TX_PIN 17
#define UART2_RX_PIN 18

// SD 1-bit pins
#define SD_CLK 36
#define SD_CMD 35
#define SD_D0 37

// I2C pins for ADS1115 and OLED
#define I2C_SDA 4
#define I2C_SCL 5

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ADS1115
Adafruit_ADS1115 ads;
const float ADS1115_LSB_mV = 0.1875F;  // mV at GAIN_TWOTHIRDS

// ---------------- WIFI (priority order) ----------------
const char* ssid_list[] = { "SNet", "SCUPDATE" };
const char* pass_list[] = { "SNet1234", "sun12345!" };
const int SSID_COUNT = sizeof(ssid_list) / sizeof(ssid_list[0]);

// NTP
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3 * 3600;  // Kenya +3
const int daylightOffset_sec = 0;

// ---------------- GLOBAL TIMERS & CONSTANTS ----------------
unsigned long timeoutMs = 0;
unsigned long EVENT_TIMEOUT = 30000;  // 30s timeout for boot menu
const unsigned long OLED_UPDATE_MS = 500;
const unsigned long POWER_INTEGRATION_MS = 200;

// ---------------- GLOBALS ----------------
unsigned long baudList[4] = { 115200, 38400, 9600, 19200 };
const char* baudNames[4] = { "115200", "38400", "9600", "19200" };
int baudIndex = 0;
int Mode = -1;  // -1: Pre-menu/uninitialized, 0..4: Operating Mode

// battery state
float batteryVoltage = 0.0;
float batteryPercent = 0.0;
const float BATTERY_PRESENT_V = 3.0;
const float BATTERY_SD_MIN_V = 3.3;
const float BATTERY_MAX_V = 4.2;
const float BATTERY_MIN_V = 3.4;

// Calibration factors (ADJUST THESE TO YOUR HARDWARE!)
// AIN1 for Terminal Voltage - ASSUMED DIVIDER IS USED
const float AIN1_DIVIDER = (294000.0 + 20000.0) / 20000.0;  // ~15.7
// AIN2 for 18650 Battery Voltage - NO DIVIDER (BATTERY_DIVIDER is now OBSOLETE)
float CURRENT_SENSITIVITY_V_PER_A = 0.2;  // MUST CALIBRATE
const float CURRENT_VREF = 2.5;

// history
#define HISTORY_LINES 100
String historyLines[HISTORY_LINES];
int historyHead = 0;
int historyCount = 0;

// SD file handling (for stability, keep file open)
String currentLogFilename = "";
File currentLogFile;  // <-- GLOBAL FILE HANDLE
bool sdMounted = false;
bool wifiConnected = false;

// timestamp caching
String cachedTimestamp = "[NO-TIME] ";
unsigned long lastTimestampUpdate = 0;

// webserver
WebServer server(80);

// control
unsigned long btnPrevMs = 0;
unsigned long ledPrevMs = 0;
unsigned long oledPrevMs = 0;
unsigned long btnDebounceMs = 40;
int btnPrevState[3] = { 1, 1, 1 };
int btnNowState[3] = { 1, 1, 1 };
int LEDs[5] = { LED_115200, LED_38400, LED_9600, LED_19200, LED_MODE };

// power integration
double accumulatedWattsSeconds = 0.0;
unsigned long lastPowerMs = 0;
// Coulomb counting
double totalCoulombs = 0.0; // Amps * seconds
float total_mAh = 0.0;

// ---------------- FORWARD DECLARATIONS ----------------
void StartUpAnimation();
void chooseBaudRate();
void chooseModeWithOLED();
void startModeInitialization();
int readButton();
void blinkLED(int idx);
void updateCachedTimestamp();
String get_timestamp();
void initADS1115();
float readADSVoltage_mV(int channel);
float readBatteryVoltage();  // Corrected: NO divider
float voltageToPercent(float v);
bool connectToFirstAvailableWiFi();
void initNTP();
bool Innit_SDCard();
String makeSequentialLogFilename();
int findNextLogNumber();
void appendLineToSD(const String& line);
void pushHistory(const String& line);

void web_handle_root();
void web_handle_history_data();
void web_handle_files();
void web_handle_download();
void web_handle_logs();
void web_handle_status();

void oledShowLogo();
void oledShowMenu(int sel);
void oledShowBaudSelect(int sel);
void oledShowStatus();


// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  delay(300);

  // pins
  pinMode(BUTTON_UP, INPUT);
  pinMode(BUTTON_ENTER, INPUT);
  pinMode(BUTTON_DOWN, INPUT);
  for (int i = 0; i < 5; i++) digitalWrite(LEDs[i], LOW);

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  // ADS1115
  initADS1115();
  batteryVoltage = readBatteryVoltage();

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 not found (Check 0x3C address and wiring).");

  } else {
    Serial.println("SSD1306 initialized.");
    display.clearDisplay();
    display.display();
    oledShowLogo();
    delay(2000);
  }

  StartUpAnimation();
  chooseBaudRate();
  chooseModeWithOLED();

  // Perform initializations specific to the chosen mode
  startModeInitialization();

  // start Serial2 at chosen baud
  Serial2.begin(baudList[baudIndex], SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);
  Serial2.setTimeout(50);

  Serial.println("LogBoy ready.");
  updateCachedTimestamp();
}

// ---------------- MODE INITIALIZATION ----------------
void startModeInitialization() {
  // --- NEW: POWEROFF HANDLER ---
  if (Mode == 5) {
    Serial.println("Powering off...");
    display.clearDisplay();
    display.display();
    // Turn off all LEDs
    for (int i = 0; i < 5; i++) digitalWrite(LEDs[i], LOW);
    // Enter Deep Sleep (requires hardware button reset or power cycle to wake)
    esp_deep_sleep_start();
  }

  // 1. SD Card Initialization (Modes 1, 2, 3)
  if (Mode == 1 || Mode == 2 || Mode == 3) {
    if (batteryVoltage >= BATTERY_SD_MIN_V) {
      sdMounted = Innit_SDCard();
      if (sdMounted) {
        currentLogFilename = makeSequentialLogFilename();
        currentLogFile = SD_MMC.open((String("/") + currentLogFilename).c_str(), FILE_APPEND);

        if (currentLogFile) {
          currentLogFile.printf("Log start (Mode %d): %s\n", Mode, get_timestamp().c_str());
          Serial.printf("Logging to SD file: %s\n", currentLogFilename.c_str());
        } else {
          Serial.println("Failed to open log file for writing.");
          sdMounted = false;
        }

      } else {
        Serial.println("SD mount failed; falling back to memory-only.");
      }

    } else {
      Serial.printf("Battery (%.3fV) < SD mount threshold - skip SD.\n", batteryVoltage);
    }
  }

  // 2. WiFi/Web Initialization (Modes 1, 3)
  if (Mode == 1 || Mode == 3) {
    if (batteryVoltage >= BATTERY_PRESENT_V) {
      wifiConnected = connectToFirstAvailableWiFi();
      if (wifiConnected) {
        initNTP();
        // start webserver endpoints
        server.on("/", web_handle_root);
        server.on("/data", web_handle_history_data);
        server.on("/logs", web_handle_logs);
        server.on("/files", web_handle_files);
        server.on("/download", web_handle_download);
        server.on("/status", web_handle_status);
        server.begin();
        Serial.println("Web server started.");
        digitalWrite(LED_MODE, HIGH);

      } else {
        Serial.println("No known WiFi available - web UI unavailable.");
      }

    } else {
      Serial.printf("Battery not present (%.3fV) - skipping WiFi/web.\n", batteryVoltage);
    }
  }
}


// ---------------- MAIN LOOP ----------------
void loop() {
  updateCachedTimestamp();

  // update battery every 1s
  static unsigned long lastBattMs = 0;
  if (millis() - lastBattMs >= 1000) {
    lastBattMs = millis();
    batteryVoltage = readBatteryVoltage();
    batteryPercent = voltageToPercent(batteryVoltage);
  }

  // Handle web server (Modes 1, 3)
  if (Mode == 1 || Mode == 3) server.handleClient();

  // Read input from Serial2
  if (Serial2.available() > 0) {
    String line = Serial2.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      String fullLine = get_timestamp() + line;
      String printLine = (Mode == 0) ? line : fullLine;

      pushHistory(fullLine);
      if (Mode != 3) Serial.println(printLine);

      if ((Mode == 1 || Mode == 2 || Mode == 3) && sdMounted) {
        appendLineToSD(fullLine);
      }
    }
  }

  // --- SINGLE INTEGRATION BLOCK FOR POWER & COULOMBS ---
  unsigned long now = millis();
  if (Mode == 3 || Mode == 4) {
    if (lastPowerMs == 0) lastPowerMs = now;
    unsigned long dt = now - lastPowerMs;
    
    if (dt >= POWER_INTEGRATION_MS) {  // integrate every 200ms
      float v_probe = (readADSVoltage_mV(1) / 1000.0) * AIN1_DIVIDER;
      float i_meas = (readADSVoltage_mV(0) / 1000.0 - CURRENT_VREF) / CURRENT_SENSITIVITY_V_PER_A;
      
      double deltaSeconds = dt / 1000.0;
      
      // Watt-hour integration
      double powerW = (double)v_probe * i_meas;
      accumulatedWattsSeconds += powerW * deltaSeconds;

      // Coulomb counting (Amp-seconds)
      totalCoulombs += (double)i_meas * deltaSeconds;
      
      // Convert Coulombs to mAh: (As / 3600) * 1000 -> As / 3.6
      total_mAh = (totalCoulombs / 3.6); 

      lastPowerMs = now;
    }
  }

  // OLED updates
  if (millis() - oledPrevMs >= OLED_UPDATE_MS) {
    oledPrevMs = millis();
    if (display.width() > 0) {
      oledShowStatus();
    }
  }
  delay(1);
}
// ---------------- ADC FUNCTIONS (FIXED) ----------------

// read AIN2 (channel 2) for battery voltage
float readBatteryVoltage() {
  float mv = readADSVoltage_mV(2);  // mV at ADS input (AIN2)
  float v_at_ads = mv / 1000.0;     // Convert mV to V
  // AIN2 is connected DIRECTLY to the 18650, so V_batt = V_at_ads
  float v_batt = v_at_ads;
  return v_batt;
}

// ---------------- OLED UI – CLEAN PHONE-STYLE LAYOUT ----------------

// ---------------- OLED UI ----------------

void oledShowStatus() {
  if (display.width() == 0) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // TOP BAR
  display.setCursor(0, 0);
  display.printf("M:%d", Mode);

  char battStr[10];
  snprintf(battStr, sizeof(battStr), "%.1fV %d%%", batteryVoltage, (int)batteryPercent);
  int battWidth = strlen(battStr) * 6;
  display.setCursor(128 - battWidth - 2, 0);
  display.print(battStr);

  int y = 14;

  // Measurement area (Mode 3 & 4)
  if (Mode == 3 || Mode == 4) {
    float v_probe = (readADSVoltage_mV(1) / 1000.0) * AIN1_DIVIDER;
    float i_meas = (readADSVoltage_mV(0) / 1000.0 - CURRENT_VREF) / CURRENT_SENSITIVITY_V_PER_A;
    float powerW = v_probe * i_meas;

    display.setCursor(0, y);
    display.printf("Vp:%.2fV I:%.2fA", v_probe, i_meas);
    y += 10;

    display.setCursor(0, y);
    display.printf("P: %.2fW", powerW);
    y += 10;
    
    display.setCursor(0, y);
    display.printf("Cap: %d mAh", (int)total_mAh); // Coulomb Counter Result
    y += 12;
  }

  // IP or Log Status
  display.setCursor(0, 56);
  if ((Mode == 1 || Mode == 3) && WiFi.status() == WL_CONNECTED) {
    display.print(WiFi.localIP().toString());
  } else if (sdMounted && currentLogFilename.length() > 0) {
    display.print("Log OK: ");
    display.print(currentLogFilename.substring(0, 8)); // Shortened
  } else {
    display.print("System Ready");
  }

  display.display();
}
// ---------------- HELPERS ----------------

void appendLineToSD(const String& line) {
  if (sdMounted && currentLogFile) {
    currentLogFile.println(line);
    currentLogFile.flush();
  } else {
    Serial.println("Error: SD file not available for write.");
  }
}

void StartUpAnimation() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(LEDs[i], HIGH);
    delay(120);
    digitalWrite(LEDs[i], LOW);
  }
}

void blinkLED(int idx) {
  unsigned long now = millis();
  if (now - ledPrevMs >= 500) {
    ledPrevMs = now;
    digitalWrite(LEDs[idx], !digitalRead(LEDs[idx]));
  }
}

int readButton() {
  unsigned long now = millis();
  if (now - btnPrevMs < btnDebounceMs) return -1;
  btnPrevMs = now;
  for (int i = 0; i < 3; i++) {
    int pin = (i == 0 ? BUTTON_UP : (i == 1 ? BUTTON_ENTER : BUTTON_DOWN));
    btnNowState[i] = digitalRead(pin);
    if (btnNowState[i] == LOW && btnPrevState[i] == HIGH) {
      btnPrevState[i] = btnNowState[i];
      return i;
    }
    btnPrevState[i] = btnNowState[i];
  }
  return -1;
}

void chooseBaudRate() {
  timeoutMs = millis();
  ledPrevMs = millis();
  btnPrevMs = millis();
  for (int i = 0; i < 4; i++) digitalWrite(LEDs[i], LOW);
  Serial.println("Select Baud Rate: UP/DOWN, ENTER to confirm (timeout default). (OLED if present)");
  int sel = 0;
  while (millis() - timeoutMs < EVENT_TIMEOUT) {
    blinkLED(sel);
    if (display.width() > 0) oledShowBaudSelect(sel);
    int b = readButton();
    if (b == 0) {
      sel = (sel - 1 + 4) % 4;
      Serial.printf("Baud preview: %lu\n", baudList[sel]);
    }
    if (b == 2) {
      sel = (sel + 1) % 4;
      Serial.printf("Baud preview: %lu\n", baudList[sel]);
    }
    if (b == 1) {
      break;
    }
    delay(50);
  }
  baudIndex = sel;
  for (int i = 0; i < 4; i++) digitalWrite(LEDs[i], LOW);
  digitalWrite(LEDs[baudIndex], HIGH);
  delay(200);
  Serial.printf("Baud selected: %lu\n", baudList[baudIndex]);
}

void chooseModeWithOLED() {
  timeoutMs = millis();
  ledPrevMs = millis();
  btnPrevMs = millis();
  Serial.println("Select Mode: Use UP/DOWN to change, ENTER to select. Timeout selects Mode 0.");
  int sel = 0;  // 0..5 (Increased to include Poweroff)
  while (millis() - timeoutMs < EVENT_TIMEOUT) {
    blinkLED(4);
    if (display.width() > 0) oledShowMenu(sel);
    int b = readButton();
    if (b == 0) {
      sel = (sel - 1 + 6) % 6;  // Wrap around 6 options
      Serial.printf("Mode preview: %d\n", sel);
    }
    if (b == 2) {
      sel = (sel + 1) % 6;  // Wrap around 6 options
      Serial.printf("Mode preview: %d\n", sel);
    }
    if (b == 1) {
      break;
    }
    delay(60);
  }
  Mode = sel;
  digitalWrite(LED_MODE, Mode == 1 ? HIGH : LOW);
  Serial.printf("Mode selected: %d\n", Mode);
}

// NTP / timestamp
void initNTP() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}

void updateCachedTimestamp() {
  if (millis() - lastTimestampUpdate < 1000) return;
  lastTimestampUpdate = millis();
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 500)) {
    char buf[32];
    strftime(buf, sizeof(buf), "[%Y-%m-%d %H:%M:%S] ", &timeinfo);
    cachedTimestamp = String(buf);
  } else {
    cachedTimestamp = "[NO-TIME] ";
  }
}

String get_timestamp() {
  return cachedTimestamp;
}

// ADS1115
void initADS1115() {
  if (!ads.begin()) {
    Serial.println("ADS1115 not found; ADC readings will be zero.");
  } else {
    ads.setGain(GAIN_TWOTHIRDS);
    Serial.println("ADS1115 initialised.");
  }
}

float readADSVoltage_mV(int channel) {
  int16_t raw = ads.readADC_SingleEnded(channel);
  float mv = raw * ADS1115_LSB_mV;
  return mv;
}

float voltageToPercent(float v) {
  float p = (v - BATTERY_MIN_V) / (BATTERY_MAX_V - BATTERY_MIN_V) * 100.0;
  if (p < 0) p = 0;
  if (p > 100) p = 100;
  return p;
}

// WiFi auto-select first available known SSID (priority order)
bool connectToFirstAvailableWiFi() {
  Serial.println("Scanning WiFi...");
  int n = WiFi.scanNetworks(false, true);
  if (n <= 0) {
    Serial.println("No networks found.");
    return false;
  }
  for (int i = 0; i < SSID_COUNT; i++) {
    const char* want = ssid_list[i];
    for (int j = 0; j < n; j++) {
      String found = WiFi.SSID(j);
      if (found.equals(want)) {
        Serial.printf("Found known SSID: %s -> connecting\n", want);
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid_list[i], pass_list[i]);
        unsigned long start = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - start < 5000) {
          delay(100);
        }
        if (WiFi.status() == WL_CONNECTED) {
          Serial.print("WiFi connected: ");
          Serial.println(WiFi.localIP());
          wifiConnected = true;
          return true;
        } else {
          Serial.println("Connect failed - try next");
          WiFi.disconnect(true);
          delay(200);
        }
      }
    }
  }
  Serial.println("No known SSID available.");
  return false;
}

// SD mount
bool Innit_SDCard() {
  Serial.println("Attempting SD mount (1-bit)...");
  SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0);
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("SD mount failed.");
    return false;
  }
  Serial.println("SD mounted.");
  uint64_t cs = SD_MMC.cardSize();
  if (cs) Serial.printf("Card size: %llu MB\n", cs / (1024ULL * 1024ULL));
  return true;
}


int findNextLogNumber() {
  int maxLog = 0;
  File root = SD_MMC.open("/");
  if (!root) return 1;

  while (true) {
    File file = root.openNextFile();
    if (!file) break; // Correct way to exit the loop

    if (!file.isDirectory()) {
      String filename = file.name();
      if (filename.startsWith("Log") && filename.endsWith(".txt")) {
        int start = 3;
        int end = filename.indexOf('_', start);
        if (end == -1) end = filename.indexOf('.', start);

        if (end > start) {
          int logNum = filename.substring(start, end).toInt();
          if (logNum > maxLog) maxLog = logNum;
        }
      }
    }
    file.close(); // Crucial for S3 memory stability
  }
  root.close();
  return maxLog + 1;
}


String makeSequentialLogFilename() {
  int logNum = findNextLogNumber();
  String ts = cachedTimestamp;
  ts.replace("[", "");
  ts.replace("]", "");
  ts.replace(":", "-");
  ts.replace(" ", "_");
  ts.trim(); 

  return "Log" + String(logNum) + "_" + ts + ".txt";
}
void pushHistory(const String& line) {
  historyLines[historyHead] = line;
  historyHead = (historyHead + 1) % HISTORY_LINES;
  if (historyCount < HISTORY_LINES) historyCount++;
}

// ---------------- Web handlers ----------------

void web_handle_history_data() {
  String data = "";
  int start = (historyHead - historyCount + HISTORY_LINES) % HISTORY_LINES;
  for (int i = 0; i < historyCount; i++) {
    int idx = (start + i) % HISTORY_LINES;
    data += historyLines[idx] + "\n";
  }
  server.send(200, "text/plain", data);
}

void web_handle_root() {
  String html = "<!doctype html><html><head><meta charset='utf-8'><title>LogBoy</title>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'/>";
  html += "<style>body{font-family:monospace;background:#111;color:#eee;padding:12px} .battery{position:fixed;left:12px;top:8px;padding:6px 10px;background:#222;border-radius:6px} .log-box{max-height:60vh;overflow-y:scroll;margin-top:20px;border:1px solid #333;padding:10px}</style>";
  html += "</head><body>";
  html += "<div class='battery' id='batt-status'>Battery: </div>";
  html += "<h2>LogBoy - Live Feed (<span id='line-count'>0</span> lines)</h2>";
  html += "<pre id='log-content' class='log-box' style='white-space:pre-wrap;'>Loading...</pre>";
  html += "<p>";
  html += "Current Log File: <b>";
  html += currentLogFilename.length() > 0 ? currentLogFilename : "N/A (SD not mounted)";
  html += "</b><br>";
  html += "<a href='/files'>Browse All Log Files</a> | ";
  html += "<a href='/logs'>Download Current Log File</a>";
  html += "</p>";

  html += "<script>function updateStatus() {fetch('/status').then(res => res.json()).then(data => {document.getElementById('batt-status').innerHTML = `Battery: ${data.battery_v} V (${data.battery_pct}%)`;});fetch('/data').then(res => res.text()).then(data => {const content = document.getElementById('log-content');content.textContent = data;document.getElementById('line-count').textContent = data.split('\\n').length - 1;if (content.scrollHeight - content.scrollTop < content.clientHeight + 100) {content.scrollTop = content.scrollHeight;}});}setInterval(updateStatus, 2000);window.onload = updateStatus;</script>";

  html += "</body></html>";
  server.send(200, "text/html", html);
}

void web_handle_files() {
  if (!SD_MMC.cardType()) {
    server.send(503, "text/plain", "SD Card not mounted.");
    return;
  }

  String html = "<!doctype html><html><head><meta charset='utf-8'><title>LogBoy Files</title><meta name='viewport' content='width=device-width,initial-scale=1'/><style>body{font-family:monospace;background:#111;color:#eee;padding:12px} a{color:#4CAF50;text-decoration:none}</style></head><body>";
  html += "<h2>💾 Saved Log Files</h2><ul>";

  File root = SD_MMC.open("/");
  File file = root.openNextFile();
  bool foundFiles = false;

  while (file) {
    String filename = file.name();
    if (filename.startsWith("Log") && filename.endsWith(".txt")) {
      foundFiles = true;
      html += "<li><a href='/download?name=" + filename + "'>" + filename + "</a> (" + String(file.size()) + " bytes)</li>";
    }
    file = root.openNextFile();
  }
  root.close();

  if (!foundFiles) html += "<li>No log files found on SD card.</li>";

  html += "</ul><p><a href='/'>\u2190 Back to Live Feed</a></p></body></html>";
  server.send(200, "text/html", html);
}

void web_handle_download() {
  if (!SD_MMC.cardType()) {
    server.send(503, "text/plain", "SD Card not mounted.");
    return;
  }
  String filename = server.arg("name");
  if (filename.length() == 0) {
    server.send(400, "text/plain", "Error: Missing file name parameter.");
    return;
  }
  String path = "/" + filename;
  File f = SD_MMC.open(path.c_str(), FILE_READ);
  if (!f) {
    server.send(404, "text/plain", "Error: File not found: " + filename);
    return;
  }
  server.setContentLength(f.size());
  server.sendHeader("Content-Disposition", "attachment; filename=" + filename);
  server.sendHeader("Content-Type", "application/octet-stream");
  server.send(200);
  uint8_t buffer[1024];
  size_t bytesRead;
  while ((bytesRead = f.read(buffer, 1024)) > 0) {
    server.sendContent((const char*)buffer, bytesRead);
  }
  f.close();
}

void web_handle_logs() {
  if (currentLogFilename.length()) {
    server.sendHeader("Location", "/download?name=" + currentLogFilename, true);
    server.send(302);
    return;
  }
  String page = "<html><body style='font-family:monospace'><h2>Logs (memory)</h2><pre>";
  int start = (historyHead - historyCount + HISTORY_LINES) % HISTORY_LINES;
  for (int i = 0; i < historyCount; i++) {
    int idx = (start + i) % HISTORY_LINES;
    page += historyLines[idx] + "\n";
  }
  page += "</pre></body></html>";
  server.send(200, "text/html", page);
}

void web_handle_status() {
  String s = "{";
  s += "\"battery_v\":" + String(batteryVoltage, 3) + ",";
  s += "\"battery_pct\":" + String((int)batteryPercent) + ",";
  s += "\"ssid\":\"" + String(WiFi.SSID()) + "\",";
  s += "\"ip\":\"" + String(WiFi.localIP().toString()) + "\"";
  s += "}";
  server.send(200, "application/json", s);
}

// ---------------- OLED Menu UI ----------------

void oledShowLogo() {
  if (display.width() == 0) return;
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(30, 10);
  display.println("Log-Boy");
  display.setTextSize(1);
  display.setCursor(30, 50);
  display.println("Starting...");
  display.display();
}

void oledShowMenu(int sel) {
  if (display.width() == 0) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Select Mode:");
  // Added "Poweroff" as the 6th item
  const char* items[6] = {
    "Mode 0 - DISP",
    "Mode 1 - WiFi&SD",
    "Mode 2 - NO WIFI",
    "Mode 3 - High Power",
    "Mode 4 - Meter",
    "Poweroff"
  };
  for (int i = 0; i < 6; i++) {
    if (i == sel) {
      display.print("> ");
    } else {
      display.print("  ");
    }
    display.println(items[i]);
  }
  display.display();
}

void oledShowBaudSelect(int sel) {
  if (display.width() == 0) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Select Baudrate:");
  for (int i = 0; i < 4; i++) {
    if (i == sel) display.print("> ");
    else display.print("");
    display.println(baudList[i]);
  }
  display.display();
}