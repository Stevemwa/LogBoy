/*
 * LogBoy - Logging-only Phase 2/3 (Final)
 * - ESP32-S3 N16R8
 * - Serial2 -> logs (RX=18 TX=17)
 * - Mode 0 = USB CDC prints (NO TIMESTAMP)
 * - Mode 1 = SD logging + WebServer when battery present (WITH TIMESTAMP)
 * - SDMMC 1-bit on pins CLK=36, CMD=35, D0=37
 * - ADS1115 on I2C SDA=4, SCL=5 -> AIN2 measures battery voltage (Confirmed)
 * - AIN0 and AIN1 are reserved for future Current/Watt Meter function.
 * - WiFi auto-selects the first available SSID from a priority list
 * - NTP cached timestamp updated once per second
 * - Web UI shows battery & latest 100 lines
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "SD_MMC.h"
#include "time.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// ---------------- PINS & HARDWARE ----------------
#define BUTTON_UP 14
#define BUTTON_ENTER 21
#define BUTTON_DOWN 15

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

// I2C pins for ADS1115
#define I2C_SDA 4
#define I2C_SCL 5

// ---------------- WIFI (priority order) ----------------
const char* ssid_list[] = {"SNet", "SCUPDATE"}; 	
const char* pass_list[] = {"SNet1234", "sun12345!"}; 	
const int SSID_COUNT = sizeof(ssid_list)/sizeof(ssid_list[0]);

// NTP
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3 * 3600; // Kenya +3
const int daylightOffset_sec = 0;

// ---------------- GLOBAL TIMERS & CONSTANTS ----------------
unsigned long timeoutMs = 0; 
unsigned long EVENT_TIMEOUT = 30000; // 30s timeout for boot menu

// ---------------- GLOBALS ----------------
unsigned long baudList[4] = {115200, 38400, 9600, 19200}; 
int baudIndex = 0;

int Mode = 0; // 0 USB, 1 SD+Web

Adafruit_ADS1115 ads; // default 0x48

// battery state
float batteryVoltage = 0.0;
float batteryPercent = 0.0;
const float BATTERY_PRESENT_V = 3.0;
const float BATTERY_SD_MIN_V = 3.2;
const float BATTERY_MAX_V = 5.0;
const float BATTERY_MIN_V = 3.0;
const float ADS1115_LSB_mV = 0.1875; // gain TWOTHIRDS

// history
#define HISTORY_LINES 100
String historyLines[HISTORY_LINES];
int historyHead = 0;
int historyCount = 0;

// SD filename
String currentLogFilename = "";

// timestamp caching
String cachedTimestamp = "[NO-TIME] ";
unsigned long lastTimestampUpdate = 0;

// webserver
WebServer server(80);

// control
unsigned long btnPrevMs = 0;
unsigned long ledPrevMs = 0;
unsigned long btnDebounceMs = 40;
int btnPrevState[3] = {1,1,1};
int btnNowState[3] = {1,1,1};
int LEDs[5] = {LED_115200, LED_38400, LED_9600, LED_19200, LED_MODE};

// ---------------- FORWARD ----------------
void StartUpAnimation();
void chooseBaudRate();
void chooseMode();
int readButton();
void blinkLED(int idx);
void updateCachedTimestamp();
String get_timestamp();
void initADS1115();
float readBatteryVoltage();
float voltageToPercent(float v);
bool connectToFirstAvailableWiFi();
void initNTP();
bool Innit_SDCard();
String makeUniqueLogFilename();
void appendLineToSD(const String &line);
void pushHistory(const String &line);
void web_handle_root();
void web_handle_logs();
void web_handle_status();

// ---------------- SETUP ----------------
void setup() {
	Serial.begin(115200);
	delay(300);

	// pins
	pinMode(BUTTON_UP, INPUT_PULLUP);
	pinMode(BUTTON_ENTER, INPUT_PULLUP);
	pinMode(BUTTON_DOWN, INPUT_PULLUP);
	for (int i=0;i<5;i++) pinMode(LEDs[i], OUTPUT);

	StartUpAnimation();
	chooseBaudRate();
	chooseMode(); // sets Mode variable

	// I2C & ADS1115
	Wire.begin(I2C_SDA, I2C_SCL);
	initADS1115();

	// start Serial2 at chosen baud
	Serial2.begin(baudList[baudIndex], SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);

	// Set a small timeout for non-blocking line reading (e.g., 50ms)
    Serial2.setTimeout(50); 
    
	Serial.println("LogBoy ready.");
}

// ---------------- MAIN LOOP ----------------
void loop() {
	// update timestamp every second
	updateCachedTimestamp();

	// update battery every 1s
	static unsigned long lastBattMs = 0;
	if (millis() - lastBattMs >= 1000) {
		lastBattMs = millis();
		batteryVoltage = readBatteryVoltage();
		batteryPercent = voltageToPercent(batteryVoltage);
	}

	// initialize Mode==1 once
	static bool mode1Init = false;
	if (Mode == 1 && !mode1Init) {
		mode1Init = true;

		bool battPresent = (batteryVoltage >= BATTERY_PRESENT_V);
		if (battPresent) {
			Serial.printf("Battery present: %.3fV (%.0f%%)\n", batteryVoltage, batteryPercent);

			// Try connect to first available WiFi
			bool wifiOK = connectToFirstAvailableWiFi();
			if (wifiOK) {
				initNTP();
				// start webserver endpoints
				server.on("/", web_handle_root);
				server.on("/logs", web_handle_logs);
				server.on("/status", web_handle_status);
				server.begin();
				Serial.println("Web server started.");
			} else {
				Serial.println("No known WiFi available - web UI unavailable.");
			}
		} else {
			Serial.printf("Battery not present (%.3fV) - skipping WiFi/web.\n", batteryVoltage);
		}

		// Try SD mount only if battery good (guard brownout)
		if (batteryVoltage >= BATTERY_SD_MIN_V) {
			if (Innit_SDCard()) {
				currentLogFilename = makeUniqueLogFilename();
				Serial.printf("Logging to SD file: %s\n", currentLogFilename.c_str());
				// write header
				File f = SD_MMC.open(("/" + currentLogFilename).c_str(), FILE_APPEND);
				if (f) {
					f.printf("Log start: %s\n", get_timestamp().c_str());
					f.close();
				}
			} else {
				Serial.println("SD mount failed; falling back to memory-only.");
			}
		} else {
			Serial.printf("Battery (%.3fV) < SD mount threshold (%.3fV) - skip SD.\n", batteryVoltage, BATTERY_SD_MIN_V);
		}
	}

	// serve web if active
	if (Mode == 1) server.handleClient();

	// --- SIMPLIFIED Non-Blocking Serial2 Ingestion ---
	if (Serial2.available() > 0) {
		String line = Serial2.readStringUntil('\n');

        // Cleanup: remove any trailing carriage return, newline, or whitespace
        line.trim();

		if (line.length() > 0) {
			String fullLine; // The line with a timestamp (for history/SD)
			String printLine; // The line to print to USB 

			if (Mode == 1) {
				// Mode 1: Log to SD/Web -> use timestamp
				fullLine = get_timestamp() + line;
				printLine = fullLine; 
			} else {
				// Mode 0: USB print only -> NO timestamp
				fullLine = get_timestamp() + line; // Still calculate it for history buffer
				printLine = line; // Print ONLY the raw log line to USB
			}

			// Store history (always uses timestamped version for web access)
			pushHistory(fullLine);

			// USB print
			Serial.println(printLine); 

			// SD append if mode1 and SD mounted
			if (Mode == 1 && currentLogFilename.length()) {
				appendLineToSD(fullLine);
			}
		}
	}

	// tiny yield
	delay(1);
}

// ---------------- HELPERS ----------------

void StartUpAnimation() {
	for (int i=0;i<5;i++){
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
	for (int i=0;i<3;i++) {
		btnNowState[i] = digitalRead( (i==0?BUTTON_UP:(i==1?BUTTON_ENTER:BUTTON_DOWN)) );
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
	for (int i=0;i<4;i++) digitalWrite(LEDs[i], LOW);
	Serial.println("Select Baud Rate: UP/DOWN, ENTER to confirm (timeout default).");
	int sel = 0;
	while (millis() - timeoutMs < EVENT_TIMEOUT) {
		blinkLED(sel);
		int b = readButton();
		if (b == 0) { sel = (sel - 1 + 4) % 4; Serial.printf("Baud preview: %lu\n", baudList[sel]); }
		if (b == 2) { sel = (sel + 1) % 4; Serial.printf("Baud preview: %lu\n", baudList[sel]); }
		if (b == 1) { break; }
		delay(50);
	}
	baudIndex = sel;
	for (int i=0;i<4;i++) digitalWrite(LEDs[i], LOW);
	digitalWrite(LEDs[baudIndex], HIGH);
	delay(200);
	Serial.printf("Baud selected: %lu\n", baudList[baudIndex]);
}

void chooseMode() {
	timeoutMs = millis();
	ledPrevMs = millis();
	btnPrevMs = millis();
	Serial.println("Select Mode: ENTER = SD mode, timeout = USB");
	while (millis() - timeoutMs < EVENT_TIMEOUT) {
		blinkLED(4);
		int b = readButton();
		if (b == 1) {
			Mode = 1;
			digitalWrite(LED_MODE, HIGH);
			Serial.println("SD Mode Selected");
			return;
		}
		delay(50);
	}
	Mode = 0;
	digitalWrite(LED_MODE, LOW);
	Serial.println("USB Mode Selected (No Timestamp)");
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
		Serial.println("ADS1115 not found; battery readings will be zero.");
	} else {
		ads.setGain(GAIN_TWOTHIRDS); // +/-6.144V range
		Serial.println("ADS1115 initialised.");
	}
}

// read AIN2 (channel 2) for battery voltage
float readBatteryVoltage() {
	// Reads from AIN2 (channel 2), as per user confirmation.
	int16_t raw = ads.readADC_SingleEnded(2); 
	float mv = raw * ADS1115_LSB_mV; // mV
	float v = mv / 1000.0;
	// IMPORTANT: If you used a resistor divider for voltage measurement, 
    // you must multiply 'v' by the divider factor here.
	// v *= divider_factor; 
	return v;
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
	for (int i=0;i<SSID_COUNT;i++) {
		const char* want = ssid_list[i];
		for (int j=0;j<n;j++) {
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
	if (cs) Serial.printf("Card size: %llu MB\n", cs / (1024ULL*1024ULL));
	return true;
}

String makeUniqueLogFilename() {
	String ts = cachedTimestamp;
	ts.replace("[", "");
	ts.replace("] ", "");
	ts.replace(":", "-");
	ts.replace(" ", "-");
	String fname = "Log" + ts + ".txt";
	return fname;
}

void appendLineToSD(const String &line) {
	if (currentLogFilename.length() == 0) return;
	String path = "/" + currentLogFilename;
	File f = SD_MMC.open(path.c_str(), FILE_APPEND);
	if (!f) {
		Serial.println("Failed open log file.");
		return;
	}
	f.println(line);
	f.close();
}

void pushHistory(const String &line) {
	historyLines[historyHead] = line;
	historyHead = (historyHead + 1) % HISTORY_LINES;
	if (historyCount < HISTORY_LINES) historyCount++;
}

// Web handlers
void web_handle_root() {
	String html = "<!doctype html><html><head><meta charset='utf-8'><title>LogBoy</title>";
	html += "<meta name='viewport' content='width=device-width,initial-scale=1'/>";
	html += "<style>body{font-family:monospace;background:#111;color:#eee;padding:12px} .battery{position:fixed;left:12px;top:8px;padding:6px 10px;background:#222;border-radius:6px}</style>";
	html += "</head><body>";
	html += "<div class='battery'>Battery: " + String(batteryVoltage,3) + " V (" + String((int)batteryPercent) + "%)</div>";
	html += "<h2>LogBoy - Latest " + String(historyCount) + " lines</h2><pre style='white-space:pre-wrap;'>";
	int start = (historyHead - historyCount + HISTORY_LINES) % HISTORY_LINES;
	for (int i=0;i<historyCount;i++) {
		int idx = (start + i) % HISTORY_LINES;
		html += historyLines[idx] + "\n";
	}
	html += "</pre><p><a href='/logs'>Raw logs (SD if available)</a></p></body></html>";
	server.send(200, "text/html", html);
}

void web_handle_logs() {
	if (currentLogFilename.length()) {
		String path = "/" + currentLogFilename;
		File f = SD_MMC.open(path.c_str(), FILE_READ);
		if (f) {
			String page = "<html><body style='font-family:monospace'><h2>" + currentLogFilename + "</h2><pre>";
			while (f.available()) page += (char)f.read();
			page += "</pre></body></html>";
			server.send(200, "text/html", page);
			f.close();
			return;
		}
	}
	// fallback
	String page = "<html><body style='font-family:monospace'><h2>Logs (memory)</h2><pre>";
	int start = (historyHead - historyCount + HISTORY_LINES) % HISTORY_LINES;
	for (int i=0;i<historyCount;i++) {
		int idx = (start + i) % HISTORY_LINES;
		page += historyLines[idx] + "\n";
	}
	page += "</pre></body></html>";
	server.send(200, "text/html", page);
}

void web_handle_status() {
	String s = "{";
	s += "\"battery_v\":" + String(batteryVoltage,3) + ",";
	s += "\"battery_pct\":" + String((int)batteryPercent) + ",";
	s += "\"ssid\":\"" + String(WiFi.SSID()) + "\",";
	s += "\"ip\":\"" + String(WiFi.localIP().toString()) + "\"";
	s += "}";
	server.send(200, "application/json", s);
}