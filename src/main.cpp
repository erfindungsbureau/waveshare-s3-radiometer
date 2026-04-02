#include <Arduino.h>
#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <Preferences.h>
#include <GxEPD2_BW.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Wire.h>
#include <driver/i2s.h>
#include <driver/rtc_io.h>
#include <math.h>

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

// Built-in LED (GPIO3, active LOW – Waveshare ESP32-S3-ePaper-1.54 offizielles Schematic)
#define LED_PIN   3

// Audio – I2S Codec ES8311 (verifiziert aus offiziellem Waveshare GPIO-Tabelle)
#define AUDIO_PWR_PIN   42   // PA EN  (I2S-Spalte, active HIGH)
#define AUDIO_CTRL_PIN  46   // PA CTRL (I2S-Spalte, active HIGH)
#define I2S_MCLK_PIN    14
#define I2S_BCLK_PIN    15
#define I2S_LRCLK_PIN   38
#define I2S_DOUT_PIN    45
#define I2C_SDA_PIN     47
#define I2C_SCL_PIN     48
#define ES8311_ADDR     0x18
#define AUDIO_SAMPLE_RATE 16000
#define I2S_PORT        I2S_NUM_0

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
  digitalWrite(LED_PIN, LOW);   // Active LOW: LOW = an
  delay(50);
  digitalWrite(LED_PIN, HIGH);  // HIGH = aus
}

// ============================================================
// Audio (ES8311 I2S Codec, legacy driver/i2s.h API)
// ============================================================
void es8311_write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ES8311_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool audioInit() {
  pinMode(AUDIO_PWR_PIN, OUTPUT);
  pinMode(AUDIO_CTRL_PIN, OUTPUT);
  digitalWrite(AUDIO_PWR_PIN, LOW);    // PA EN (IO42) = P-MOSFET Gate, LOW = ON
  digitalWrite(AUDIO_CTRL_PIN, HIGH);  // PA CTRL (IO46) = NS4150B enable, HIGH = ON
  delay(100);

  // ── Schritt 1: I2S zuerst starten damit MCLK läuft ──────
  i2s_config_t i2s_cfg = {
    .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate          = AUDIO_SAMPLE_RATE,
    .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags     = 0,
    .dma_buf_count        = 4,
    .dma_buf_len          = 256,
    .use_apll             = true,
    .tx_desc_auto_clear   = true,
    .fixed_mclk           = 4096000,  // 256 × 16kHz
    .mclk_multiple        = I2S_MCLK_MULTIPLE_256,
    .bits_per_chan         = I2S_BITS_PER_CHAN_DEFAULT,
  };
  if (i2s_driver_install(I2S_PORT, &i2s_cfg, 0, NULL) != ESP_OK) {
    Serial.println("I2S: Driver install fehlgeschlagen");
    return false;
  }
  i2s_pin_config_t pin_cfg = {
    .mck_io_num   = I2S_MCLK_PIN,
    .bck_io_num   = I2S_BCLK_PIN,
    .ws_io_num    = I2S_LRCLK_PIN,
    .data_out_num = I2S_DOUT_PIN,
    .data_in_num  = I2S_PIN_NO_CHANGE,
  };
  if (i2s_set_pin(I2S_PORT, &pin_cfg) != ESP_OK) {
    Serial.println("I2S: Pin config fehlgeschlagen");
    i2s_driver_uninstall(I2S_PORT);
    return false;
  }
  delay(50);  // MCLK stabilisieren

  // ── Schritt 2: I2C Scan ──────────────────────────────────
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Serial.println("I2C Scan...");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0)
      Serial.printf("  I2C Gerät gefunden: 0x%02X\n", addr);
  }
  Serial.println("I2C Scan fertig");

  // ES8311 Init-Sequenz (DAC-only, verifiziert aus Waveshare Beispielcode)
  es8311_write(0x44, 0x08);  // noise gate off (2× für zuverlässigen ersten Boot)
  es8311_write(0x44, 0x08);
  es8311_write(0x01, 0x30);  // CLK Manager: MCLK enable
  es8311_write(0x02, 0x00);  // pre_div=1, pre_multi=1
  es8311_write(0x03, 0x10);  // ADC OSR=16
  es8311_write(0x16, 0x24);  // MIC gain=0dB
  es8311_write(0x04, 0x20);  // DAC OSR=32 (für 16 kHz)
  es8311_write(0x05, 0x00);  // adc_div=1, dac_div=1
  es8311_write(0x0B, 0x00);  // VMID on
  es8311_write(0x0C, 0x00);  // analog bias on
  es8311_write(0x10, 0x1F);  // headphone amp on
  es8311_write(0x11, 0x7F);  // speaker/line-out on
  es8311_write(0x00, 0x80);  // Codec on, Slave mode
  es8311_write(0x01, 0x3F);  // MCLK-Quelle: externer MCLK-Pin
  es8311_write(0x13, 0x10);  // analog LDO
  es8311_write(0x1B, 0x0A);  // HPF
  es8311_write(0x1C, 0x6A);  // HPF + Equalizer
  es8311_write(0x44, 0x58);  // DAC-Referenz aktiv

  // Takt: 16 kHz, MCLK=4.096 MHz (256×fs), 16-bit Philips I2S
  // LRCLK-Teiler = 256  →  REG07/08 = 0x00/0xFF
  // BCLK = 16000 × 16 × 2 = 512 kHz  →  MCLK/BCLK=8  →  bclk_div-1 = 7
  es8311_write(0x07, 0x00);
  es8311_write(0x08, 0xFF);
  es8311_write(0x06, 0x07);

  // Format: 16-bit Philips I2S; bit[6]=0 → DAC aktiv, ADC gemutet
  es8311_write(0x09, 0x0C);        // DAC: 16-bit, Philips, aktiv
  es8311_write(0x0A, 0x0C | 0x40); // ADC: 16-bit, Philips, gemutet

  // DAC-Pfad aktivieren
  es8311_write(0x0D, 0x01);
  es8311_write(0x0E, 0x02);
  es8311_write(0x12, 0x00);
  es8311_write(0x14, 0x1A);
  es8311_write(0x15, 0x40);
  es8311_write(0x37, 0x08);
  es8311_write(0x45, 0x00);
  es8311_write(0x17, 0xBF);
  es8311_write(0x32, 0xBF);  // DAC-Lautstärke ~75 %
  delay(20);
  return true;
}

void audioShutdown() {
  i2s_driver_uninstall(I2S_PORT);
  Wire.end();
  digitalWrite(AUDIO_PWR_PIN, HIGH);   // P-MOSFET OFF
  digitalWrite(AUDIO_CTRL_PIN, LOW);   // NS4150B disable
}

// Geigerzähler-Simulation: deviceCount Klicks verteilt über 5 Sekunden
void playGeigerClicks(int deviceCount) {
  if (deviceCount == 0) return;
  Serial.printf("Geiger-Audio: %d Gerät(e)\n", deviceCount);
  if (!audioInit()) return;

  // Klick-Buffer: 5 ms = 80 Samples @ 16 kHz, gedämpfte 3.2-kHz-Sinuswelle
  const int CLICK_SAMPLES = 80;
  int16_t clickBuf[CLICK_SAMPLES * 2];
  for (int i = 0; i < CLICK_SAMPLES; i++) {
    float t   = (float)i / AUDIO_SAMPLE_RATE;
    float env = expf(-t * 500.0f);
    int16_t s = (int16_t)(30000.0f * env * sinf(2.0f * M_PI * 3200.0f * t));
    clickBuf[i * 2]     = s;
    clickBuf[i * 2 + 1] = s;
  }

  // Stille: 10-ms-Chunks (160 Frames, zero-initialisiert)
  const int CHUNK = 160;
  static int16_t silence[CHUNK * 2];  // static → zero-initialisiert

  // Klick-Zeitpunkte via Poisson-Prozess (exponentialverteilte Abstände)
  const int TOTAL_MS    = 5000;
  const int MAX_CLICKS  = min(deviceCount, 200);
  float meanMs          = (float)TOTAL_MS / MAX_CLICKS;

  int   clickPos[200];   // Klick-Positionen in Samples
  int   nClicks = 0;
  float cumMs   = 0.0f;
  while (nClicks < MAX_CLICKS) {
    float u = (float)((esp_random() & 0xFFFE) + 1) / 65536.0f;  // (0,1]
    cumMs += -meanMs * logf(u);
    if (cumMs >= TOTAL_MS) break;
    clickPos[nClicks++] = (int)(cumMs * AUDIO_SAMPLE_RATE / 1000.0f);
  }

  // Wiedergabe: Stille in Chunks, Klick wenn fällig
  const int totalSamples = TOTAL_MS * AUDIO_SAMPLE_RATE / 1000;
  int curSample = 0, nextClick = 0;
  size_t written;

  while (curSample < totalSamples) {
    if (nextClick < nClicks && clickPos[nextClick] < curSample + CHUNK) {
      // Stille bis zum Klick schreiben
      int before = clickPos[nextClick] - curSample;
      if (before > 0) {
        i2s_write(I2S_PORT, silence, before * 4, &written, portMAX_DELAY);
        curSample += before;
      }
      // Klick ausgeben
      i2s_write(I2S_PORT, clickBuf, sizeof(clickBuf), &written, portMAX_DELAY);
      curSample += CLICK_SAMPLES;
      nextClick++;
    } else {
      int remaining = totalSamples - curSample;
      int chunk     = min(CHUNK, remaining);
      i2s_write(I2S_PORT, silence, chunk * 4, &written, portMAX_DELAY);
      curSample += chunk;
    }
  }

  audioShutdown();
  Serial.println("Geiger-Audio fertig");
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
  analogSetAttenuation(ADC_11db);  // ~3.3V Messbereich (offizieller Waveshare-Beispielcode)
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
  char voltageStr[5];
  sprintf(voltageStr, "%.1fV", voltage);
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
// ============================================================
// Scanning-Screen (nur beim manuellen Scan via PWR-Button)
// ============================================================
void showScanningScreen(int wifiCount, int bleCount, bool scanDone) {
  display.setRotation(0);
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);

    // "SCANNING" gross oben
    display.setFont(&FreeSansBold9pt7b);
    display.setTextSize(2);
    display.setCursor(10, 40);
    display.print(scanDone ? "RESULTS" : "SCANNING");

    // Trennlinie
    display.drawLine(0, 50, 200, 50, GxEPD_BLACK);

    // WiFi Anzahl
    display.setTextSize(1);
    display.setFont(&FreeSansBold9pt7b);
    display.setCursor(10, 90);
    display.print("WiFi:");
    display.setTextSize(3);
    display.setCursor(80, 95);
    display.print(wifiCount);

    // BLE Anzahl
    display.setTextSize(1);
    display.setFont(&FreeSansBold9pt7b);
    display.setCursor(10, 140);
    display.print("BLE:");
    display.setTextSize(3);
    display.setCursor(80, 145);
    display.print(bleCount);

    // Total
    display.setTextSize(1);
    display.drawLine(0, 160, 200, 160, GxEPD_BLACK);
    display.setCursor(10, 185);
    display.print("TOTAL: ");
    display.setTextSize(2);
    display.setCursor(90, 185);
    display.print(wifiCount + bleCount);

  } while (display.nextPage());
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

    // Batterie (oben rechts, genug Abstand zu "RADIOMETER")
    float battVoltage = getBatteryVoltage();
    int battPercent = getBatteryPercent(battVoltage);
    drawBattery(145, 2, battPercent, battVoltage);

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
    if (battPercent < 20)                                    display.print("LOW BATTERY");
    else if (isScanning)                                     display.print("Scanning...");
    else if (current.wifiCount > 0 || current.bleCount > 0) display.print("RADIOACTIVE");
    else                                                     display.print("RADIO INACTIVE");

  } while (display.nextPage());

  Serial.println("Display aktualisiert");
}

// ============================================================
// Scan + Display Update (gemeinsam für auto + manuell)
// ============================================================
void runScan(bool isManualScan) {
  bool playGeiger = isManualScan;

  Statistics hour = calculateStats(SCANS_PER_HOUR);
  Statistics day  = calculateStats(SCANS_PER_DAY);
  Statistics week = calculateStats(SCANS_PER_WEEK);

  isScanning = true;

  if (isManualScan) {
    blinkLED();
    showScanningScreen(0, 0, false);
  } else {
    bool needsFullRefresh = (scansUntilFullRefresh <= 1);
    updateDisplay(lastScan, hour, day, week, needsFullRefresh);
  }

  ScanData currentScan = performScan();
  lastScan = currentScan;
  isScanning = false;

  if (isManualScan) {
    showScanningScreen(currentScan.wifiCount, currentScan.bleCount, true);
    delay(3000);
  }

  if (!isManualScan) addToHistory(currentScan);

  hour = calculateStats(SCANS_PER_HOUR);
  day  = calculateStats(SCANS_PER_DAY);
  week = calculateStats(SCANS_PER_WEEK);

  Serial.printf("1h:  %.1f%% (%s)\n", hour.offlinePercent, formatTime(hour.offlineMinutes).c_str());
  Serial.printf("24h: %.1f%% (%s)\n", day.offlinePercent,  formatTime(day.offlineMinutes).c_str());
  Serial.printf("7d:  %.1f%% (%s)\n", week.offlinePercent, formatTime(week.offlineMinutes).c_str());

  updateDisplay(currentScan, hour, day, week, false);
  blinkLED();

  if (playGeiger) {
    playGeigerClicks(currentScan.wifiCount + currentScan.bleCount);
  }

  if (!isManualScan) {
    if (--scansUntilFullRefresh <= 0) scansUntilFullRefresh = 6;
    if (--scansUntilFlashSave  <= 0) { saveToFlash(); scansUntilFlashSave = 12; }
  }
}

// ============================================================
// Setup (einmalig beim Boot)
// ============================================================
void setup() {
  Serial.begin(115200);           // USB CDC
  Serial1.begin(115200, SERIAL_8N1, 44, 43); // Hardware UART: RX=IO44, TX=IO43
  delay(1000);

  bootCount++;
  Serial.printf("\n=== BOOT #%d (Waveshare ESP32-S3-ePaper-1.54, kein Deep Sleep) ===\n", bootCount);

  pinMode(BOOT_BTN, INPUT_PULLUP);
  pinMode(PWR_BTN,  INPUT_PULLUP);

  displayPowerOn();
  SPI.begin(EPD_SCK, -1, EPD_MOSI, EPD_CS);
  display.init(115200, true, 10, false, SPI, SPISettings(4000000, MSBFIRST, SPI_MODE0));
  delay(100);
  Serial.println("Display initialisiert");

  loadFromFlash();
  runScan(false);
}

// ============================================================
// Loop – While-Loop, kein delay, kein deep sleep
//   • LED blinkt kurz alle 10 Sek (Heartbeat)
//   • Button → Geiger-Sound + sofortiger Scan
//   • Auto-Scan alle 5 Min
// ============================================================
void loop() {
  const unsigned long SCAN_INTERVAL_MS  = SCAN_INTERVAL_US / 1000UL;
  const unsigned long HEARTBEAT_MS      = 10000UL; // LED-Blitz alle 10 Sek
  const unsigned long DEBOUNCE_MS       = 30UL;

  unsigned long nextScan      = millis() + SCAN_INTERVAL_MS;
  unsigned long nextHeartbeat = millis() + HEARTBEAT_MS;
  bool bootBtnWasHigh = true;
  bool pwrBtnWasHigh  = true;

  while (true) {
    unsigned long now = millis();

    // ── Heartbeat-LED ────────────────────────────────────────
    if (now >= nextHeartbeat) {
      digitalWrite(LED_PIN, LOW);   // LED an (active LOW)
      unsigned long t = millis();
      while (millis() - t < 20) {} // 20 ms Blitz (busy wait, kurz genug)
      digitalWrite(LED_PIN, HIGH);  // LED aus
      nextHeartbeat = millis() + HEARTBEAT_MS;
    }

    // ── Button-Erkennung (active LOW, Flanke HIGH→LOW) ───────
    bool bootNow = digitalRead(BOOT_BTN);
    bool pwrNow  = digitalRead(PWR_BTN);
    bool pressed = false;

    if ((bootBtnWasHigh && !bootNow) || (pwrBtnWasHigh && !pwrNow)) {
      // Entprellung
      unsigned long t = millis();
      while (millis() - t < DEBOUNCE_MS) {}
      if (!digitalRead(BOOT_BTN) || !digitalRead(PWR_BTN)) {
        pressed = true;
      }
    }
    bootBtnWasHigh = bootNow;
    pwrBtnWasHigh  = pwrNow;

    if (pressed) {
      Serial.println("Button → Geiger-Sound + manueller Scan");
      // Geiger-Sound VOR dem Scan als sofortiges Feedback
      playGeigerClicks(max(lastScan.wifiCount + lastScan.bleCount, 3));
      runScan(true);
      nextScan = millis() + SCAN_INTERVAL_MS;
    }

    // ── Auto-Scan alle 5 Min ─────────────────────────────────
    if (millis() >= nextScan) {
      Serial.println("Auto-Scan (5 min)");
      runScan(false);
      nextScan = millis() + SCAN_INTERVAL_MS;
    }
  }
}

