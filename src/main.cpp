#include <Arduino.h>
#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <Preferences.h>
#include <GxEPD2_BW.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSans9pt7b.h>

// ============================================================
// Waveshare ESP32-S3-ePaper-1.54 Pin-Konfiguration
// ============================================================

// E-Paper SPI Pins
#define EPD_CS    11
#define EPD_DC    10
#define EPD_SCK   12
#define EPD_MOSI  13
#define EPD_RST   9
#define EPD_BUSY  8

// Power-Management Pins
#define EPD_PWR_PIN   6   // LOW = Display einschalten
#define VBAT_PWR_PIN  17  // HIGH = Batteriemessung aktivieren

// Buttons
#define BOOT_BTN  0   // BOOT-Taste (wakeup-fähig)
#define PWR_BTN   18  // PWR-Taste (wakeup-fähig)

// Built-in LED (GPIO2 für ESP32-S3 DevKitC-1 v1.0 – anpassen falls nötig)
#define LED_PIN   2

// Batterie ADC
#define BATT_ADC  4   // GPIO4 = ADC1_CH3, Spannungsteiler 2:1

// Button Mask für ext1 wakeup (beide Buttons)
#define BTN_PIN_MASK  ((1ULL << BOOT_BTN) | (1ULL << PWR_BTN))

// Display 200x200 (GDEH0154D67 kompatibel)
// Falls Display nicht reagiert: GxEPD2_154, GxEPD2_154_M09, GxEPD2_154_M10 versuchen
GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> display(GxEPD2_154_D67(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY));

// Scan-Intervall: 5 Minuten
#define SCAN_INTERVAL_US 300000000ULL
#define SCANS_PER_HOUR 12
#define SCANS_PER_DAY 288
#define SCANS_PER_WEEK 2016
#define HISTORY_SIZE SCANS_PER_WEEK

Preferences prefs;

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int historyIndex = 0;
RTC_DATA_ATTR int scansUntilFlashSave = 12;
RTC_DATA_ATTR int scansUntilFullRefresh = 6;

struct ScanData {
  uint8_t wifiCount;
  uint8_t bleCount;
  bool isOffline;
};

RTC_DATA_ATTR ScanData lastScan = {0, 0, false};

struct Statistics {
  int totalScans;
  int offlineScans;
  int offlineMinutes;
  float offlinePercent;
};

RTC_DATA_ATTR uint8_t historyBitmap[HISTORY_SIZE / 8 + 1];
RTC_DATA_ATTR uint16_t wifiHistory[SCANS_PER_DAY];
RTC_DATA_ATTR uint16_t bleHistory[SCANS_PER_DAY];

String statusText = "Ready";
bool isScanning = false;

// ============================================================
// LED
// ============================================================
void blinkLED() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(50);
  digitalWrite(LED_PIN, LOW);
}

// ============================================================
// Display Power
// ============================================================
void displayPowerOn() {
  pinMode(EPD_PWR_PIN, OUTPUT);
  digitalWrite(EPD_PWR_PIN, LOW);  // Active LOW
  delay(100);
}

void displayPowerOff() {
  digitalWrite(EPD_PWR_PIN, HIGH);
}

// ============================================================
// WiFi-Scan
// ============================================================
int scanWiFi() {
  Serial.println("Scanne WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  int n = WiFi.scanNetworks(false, false, false, 300);
  WiFi.mode(WIFI_OFF);
  int result = (n > 0) ? n : 0;
  Serial.printf("  WiFi: %d Netzwerke\n", result);
  return result;
}

// ============================================================
// Bluetooth-Scan
// ============================================================
int scanBluetooth() {
  Serial.println("Scanne Bluetooth...");
  BLEDevice::init("");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  pBLEScan->setActiveScan(true);
  BLEScanResults activeDevices = pBLEScan->start(8, false);
  int activeCount = activeDevices.getCount();
  pBLEScan->setActiveScan(false);
  BLEScanResults passiveDevices = pBLEScan->start(8, false);
  int passiveCount = passiveDevices.getCount();
  int totalCount = max(activeCount, passiveCount);
  pBLEScan->clearResults();
  BLEDevice::deinit(false);
  Serial.printf("  BLE total: %d Geräte\n", totalCount);
  return totalCount;
}

// ============================================================
// Scan durchführen
// ============================================================
ScanData performScan() {
  isScanning = true;
  ScanData scan;
  scan.wifiCount = min(scanWiFi(), 255);
  delay(500);
  scan.bleCount = min(scanBluetooth(), 255);
  scan.isOffline = (scan.wifiCount == 0 && scan.bleCount == 0);
  Serial.printf("Scan: WiFi=%d, BLE=%d, Status=%s\n",
                scan.wifiCount, scan.bleCount,
                scan.isOffline ? "OFFLINE" : "online");
  isScanning = false;
  return scan;
}

// ============================================================
// Historie (Ringpuffer)
// ============================================================
void setBit(int index, bool value) {
  int byteIndex = index / 8;
  int bitIndex = index % 8;
  if (value) historyBitmap[byteIndex] |= (1 << bitIndex);
  else       historyBitmap[byteIndex] &= ~(1 << bitIndex);
}

bool getBit(int index) {
  int byteIndex = index / 8;
  int bitIndex = index % 8;
  return (historyBitmap[byteIndex] & (1 << bitIndex)) != 0;
}

void addToHistory(ScanData scan) {
  setBit(historyIndex, scan.isOffline);
  int dayIndex = historyIndex % SCANS_PER_DAY;
  wifiHistory[dayIndex] = scan.wifiCount;
  bleHistory[dayIndex] = scan.bleCount;
  historyIndex = (historyIndex + 1) % HISTORY_SIZE;
}

Statistics calculateStats(int lookbackScans) {
  Statistics stats = {0, 0, 0, 0.0};
  int actualScans = min(lookbackScans, HISTORY_SIZE);
  int startIndex = (historyIndex - actualScans + HISTORY_SIZE) % HISTORY_SIZE;
  for (int i = 0; i < actualScans; i++) {
    int idx = (startIndex + i) % HISTORY_SIZE;
    if (getBit(idx)) stats.offlineScans++;
    stats.totalScans++;
  }
  stats.offlineMinutes = stats.offlineScans * 5;
  stats.offlinePercent = stats.totalScans > 0
    ? (float)stats.offlineScans / stats.totalScans * 100.0 : 0.0;
  return stats;
}

// ============================================================
// Flash-Speicherung
// ============================================================
void saveToFlash() {
  Serial.println("Speichere in Flash...");
  prefs.begin("nature-track", false);
  prefs.putBytes("bitmap", historyBitmap, sizeof(historyBitmap));
  prefs.putBytes("wifi24h", wifiHistory, sizeof(wifiHistory));
  prefs.putBytes("ble24h", bleHistory, sizeof(bleHistory));
  prefs.putInt("histIdx", historyIndex);
  prefs.end();
}

void loadFromFlash() {
  prefs.begin("nature-track", true);
  size_t bitmapSize = prefs.getBytesLength("bitmap");
  if (bitmapSize > 0) {
    prefs.getBytes("bitmap", historyBitmap, sizeof(historyBitmap));
  }
  prefs.getBytes("wifi24h", wifiHistory, sizeof(wifiHistory));
  prefs.getBytes("ble24h", bleHistory, sizeof(bleHistory));
  historyIndex = prefs.getInt("histIdx", 0);
  prefs.end();
}

// ============================================================
// Batterie (Waveshare: GPIO4 = ADC1_CH3, Teiler 2:1)
// ============================================================
float getBatteryVoltage() {
  pinMode(VBAT_PWR_PIN, OUTPUT);
  digitalWrite(VBAT_PWR_PIN, HIGH);  // Messung aktivieren
  delay(20);
  analogReadResolution(12);
  uint32_t adcSum = 0;
  for (int i = 0; i < 10; i++) {
    adcSum += analogRead(BATT_ADC);
    delay(5);
  }
  digitalWrite(VBAT_PWR_PIN, LOW);  // Messung deaktivieren (Strom sparen)
  float adcValue = adcSum / 10.0;
  return (adcValue / 4095.0) * 3.3 * 2.0;  // Teiler 2:1
}

int getBatteryPercent(float voltage) {
  if (voltage >= 4.2) return 100;
  if (voltage >= 4.0) return 75 + (int)((voltage - 4.0) * 125);
  if (voltage >= 3.7) return 50 + (int)((voltage - 3.7) * 83);
  if (voltage >= 3.5) return 25 + (int)((voltage - 3.5) * 125);
  if (voltage >= 3.0) return (int)((voltage - 3.0) * 50);
  return 0;
}

void drawBattery(int x, int y, int percent, float voltage) {
  int w = 20, h = 10;
  display.drawRect(x, y, w, h, GxEPD_BLACK);
  display.fillRect(x + w, y + 2, 2, h - 4, GxEPD_BLACK);
  int fillWidth = (percent * 18) / 100;
  if (fillWidth > 0) display.fillRect(x + 1, y + 1, fillWidth, h - 2, GxEPD_BLACK);
  display.setFont(nullptr);
  display.setTextSize(1);
  display.setCursor(x + w + 5, y + 2);
  char voltageStr[6];
  sprintf(voltageStr, "%.2fV", voltage);
  display.print(voltageStr);
}

// ============================================================
// Histogramm (24h, stündlich)
// ============================================================
void calculateHourlyHistogram(uint8_t histogram[24]) {
  for (int h = 0; h < 24; h++) histogram[h] = 0;
  for (int h = 0; h < 24; h++) {
    int totalDevices = 0, scanCount = 0;
    int hourStartIndex = historyIndex - (24 - h) * SCANS_PER_HOUR;
    for (int s = 0; s < SCANS_PER_HOUR; s++) {
      int idx = (hourStartIndex + s + HISTORY_SIZE) % HISTORY_SIZE;
      int dayIdx = idx % SCANS_PER_DAY;
      totalDevices += wifiHistory[dayIdx] + bleHistory[dayIdx];
      scanCount++;
    }
    if (scanCount > 0) histogram[h] = min(totalDevices / scanCount, 255);
  }
}

void drawGrayBar(int x, int y, int w, int h) {
  for (int dy = 0; dy < h; dy++)
    for (int dx = 0; dx < w; dx++)
      if ((dx + dy) % 2 == 0) display.drawPixel(x + dx, y + dy, GxEPD_BLACK);
}

String formatTime(int minutes) {
  char buf[10];
  sprintf(buf, "%dh%02dm", minutes / 60, minutes % 60);
  return String(buf);
}

// ============================================================
// Display aktualisieren
// ============================================================
void updateDisplay(ScanData current, Statistics hour, Statistics day, Statistics week, bool fullRefresh = false) {
  display.setRotation(0);

  if (fullRefresh) {
    display.setFullWindow();
    display.clearScreen();
  } else {
    display.setPartialWindow(0, 0, 200, 200);
  }

  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);

    // Header
    display.setFont(&FreeSansBold9pt7b);
    display.setTextColor(GxEPD_BLACK);
    display.setCursor(5, 14);
    display.print("RADIOMETER");

    // Batterie (oben rechts)
    float battVoltage = getBatteryVoltage();
    int battPercent = getBatteryPercent(battVoltage);
    drawBattery(125, 5, battPercent, battVoltage);

    display.drawLine(0, 22, 200, 22, GxEPD_BLACK);

    // Aktuelle Geräte
    display.setFont(&FreeSans9pt7b);
    display.setCursor(5, 38);
    display.printf("WiFi:%d BLE:%d", current.wifiCount, current.bleCount);

    // Histogramm (24h)
    int histX = 20, histY = 55, histWidth = 168, histHeight = 50;
    uint8_t histogram[24];
    calculateHourlyHistogram(histogram);

    uint8_t maxValue = 1;
    for (int i = 0; i < 24; i++) if (histogram[i] > maxValue) maxValue = histogram[i];
    int maxDevices = ((maxValue / 5) + 1) * 5;
    if (maxDevices < 5) maxDevices = 5;

    display.drawLine(histX - 2, histY, histX - 2, histY + histHeight, GxEPD_BLACK);
    display.drawLine(histX - 2, histY + histHeight, histX + histWidth, histY + histHeight, GxEPD_BLACK);

    display.setFont(nullptr);
    display.setTextSize(1);
    display.drawLine(histX - 2, histY + histHeight, histX - 5, histY + histHeight, GxEPD_BLACK);
    display.setCursor(1, histY + histHeight - 2);
    display.print("0");

    int midY = histY + histHeight / 2;
    display.drawLine(histX - 2, midY, histX - 5, midY, GxEPD_BLACK);
    char midLabel[4]; sprintf(midLabel, "%d", maxDevices / 2);
    display.setCursor(1, midY - 2); display.print(midLabel);

    display.drawLine(histX - 2, histY, histX - 5, histY, GxEPD_BLACK);
    char maxLabel[4]; sprintf(maxLabel, "%d", maxDevices);
    display.setCursor(1, histY - 2); display.print(maxLabel);

    int barWidth = 7;
    for (int i = 0; i < 24; i++) {
      int barHeight = (histogram[i] * histHeight) / maxDevices;
      if (barHeight > histHeight) barHeight = histHeight;
      int x = histX + ((23 - i) * barWidth);
      int y = histY + histHeight - barHeight;
      if (barHeight > 0) drawGrayBar(x, y, barWidth - 1, barHeight);
    }

    // X-Achse Labels (rechts=0h=jetzt, links=24h)
    int x0h = histX + histWidth;
    display.drawLine(x0h, histY + histHeight, x0h, histY + histHeight + 3, GxEPD_BLACK);
    display.setCursor(x0h - 2, histY + histHeight + 5); display.print("0");
    int x6h = histX + (3 * histWidth / 4);
    display.drawLine(x6h, histY + histHeight, x6h, histY + histHeight + 3, GxEPD_BLACK);
    display.setCursor(x6h - 3, histY + histHeight + 5); display.print("6");
    int x12h = histX + (histWidth / 2);
    display.drawLine(x12h, histY + histHeight, x12h, histY + histHeight + 3, GxEPD_BLACK);
    display.setCursor(x12h - 5, histY + histHeight + 5); display.print("12");
    int x18h = histX + (histWidth / 4);
    display.drawLine(x18h, histY + histHeight, x18h, histY + histHeight + 3, GxEPD_BLACK);
    display.setCursor(x18h - 5, histY + histHeight + 5); display.print("18");
    int x24h = histX;
    display.drawLine(x24h, histY + histHeight, x24h, histY + histHeight + 3, GxEPD_BLACK);
    display.setCursor(x24h - 2, histY + histHeight + 5); display.print("24");

    // Statistiken
    int y = 138;
    display.setFont(&FreeSans9pt7b);
    display.setCursor(5, y); display.print("1h:");
    display.setCursor(50, y); display.print(formatTime(hour.offlineMinutes));
    display.setCursor(130, y); display.printf("%.0f%%", hour.offlinePercent);
    y += 18;
    display.setCursor(5, y); display.print("24h:");
    display.setCursor(50, y); display.print(formatTime(day.offlineMinutes));
    display.setCursor(130, y); display.printf("%.0f%%", day.offlinePercent);
    y += 18;
    display.setCursor(5, y); display.print("7d:");
    display.setCursor(50, y); display.print(formatTime(week.offlineMinutes));
    display.setCursor(130, y); display.printf("%.0f%%", week.offlinePercent);

    // Status-Text
    y += 22;
    display.setFont(&FreeSansBold9pt7b);
    display.setCursor(5, y);
    if (isScanning)                                          display.print("Scanning...");
    else if (current.wifiCount > 0 || current.bleCount > 0) display.print("RADIOACTIVE");
    else                                                     display.print("RADIO INACTIVE");

  } while (display.nextPage());

  Serial.println("Display aktualisiert");
}

// ============================================================
// Main Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  bootCount++;
  Serial.printf("\n=== BOOT #%d (Waveshare ESP32-S3-ePaper-1.54) ===\n", bootCount);

  pinMode(BOOT_BTN, INPUT);
  pinMode(PWR_BTN, INPUT);

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  bool isManualScan = false;

  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
    uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
    Serial.printf("Button-Wakeup (Maske: 0x%llX)\n", wakeup_pin_mask);
    isManualScan = true;
  } else if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
    Serial.println("Timer-Wakeup");
  } else {
    Serial.println("Erster Boot");
  }

  // Display power on + SPI mit custom Pins initialisieren
  displayPowerOn();
  SPI.begin(EPD_SCK, -1, EPD_MOSI, EPD_CS);
  // Nur beim ersten Boot hard-reset (bootCount==1), danach soft-init für partial refresh ohne Flackern
  display.init(115200, bootCount == 1, 10, false, SPI, SPISettings(4000000, MSBFIRST, SPI_MODE0));
  delay(100);
  Serial.println("Display initialisiert");

  if (bootCount == 1) {
    loadFromFlash();
  }

  Statistics hour = calculateStats(SCANS_PER_HOUR);
  Statistics day  = calculateStats(SCANS_PER_DAY);
  Statistics week = calculateStats(SCANS_PER_WEEK);

  bool needsFullRefresh = !isManualScan && ((bootCount == 1) || (scansUntilFullRefresh <= 1));
  isScanning = true;
  updateDisplay(lastScan, hour, day, week, needsFullRefresh);

  ScanData currentScan = performScan();
  lastScan = currentScan;

  if (!isManualScan) {
    addToHistory(currentScan);
  }

  hour = calculateStats(SCANS_PER_HOUR);
  day  = calculateStats(SCANS_PER_DAY);
  week = calculateStats(SCANS_PER_WEEK);

  Serial.printf("1h:  %.1f%% (%s)\n", hour.offlinePercent, formatTime(hour.offlineMinutes).c_str());
  Serial.printf("24h: %.1f%% (%s)\n", day.offlinePercent,  formatTime(day.offlineMinutes).c_str());
  Serial.printf("7d:  %.1f%% (%s)\n", week.offlinePercent, formatTime(week.offlineMinutes).c_str());

  updateDisplay(currentScan, hour, day, week, false);
  blinkLED();  // Kurzes Aufblinken zeigt: Gerät läuft, Display wurde aktualisiert

  if (!isManualScan) {
    if (--scansUntilFullRefresh <= 0) scansUntilFullRefresh = 6;
    if (--scansUntilFlashSave <= 0) {
      saveToFlash();
      scansUntilFlashSave = 12;
    }
  }

  // Deep Sleep (5 Minuten + Button-Wakeup)
  Serial.println(">>> Deep Sleep <<<");
  display.hibernate();
  displayPowerOff();

  esp_sleep_enable_ext1_wakeup(BTN_PIN_MASK, ESP_EXT1_WAKEUP_ANY_HIGH);
  esp_sleep_enable_timer_wakeup(SCAN_INTERVAL_US);
  delay(100);
  esp_deep_sleep_start();
}

void loop() {
  // Nie erreicht wegen Deep Sleep
}
