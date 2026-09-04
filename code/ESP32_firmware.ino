#include "esp_camera.h"
#include "esp_task_wdt.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_W    128
#define OLED_H    64
#define OLED_ADDR 0x3C
#define OLED_SDA  48
#define OLED_SCL  47

Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);
bool oledReady = false;

#define MAX_TEXT_LEN 128

#define BUTTON_PIN          40
#define BUTTON_ACTIVE_HIGH  true
#define BUTTON_DEBOUNCE_MS  30

#define USE_AUTO_EXPOSURE   false
#define EXPOSURE_VALUE      130
#define GAIN_VALUE          12

static const uint8_t SYNC0 = 0xA5;
static const uint8_t SYNC1 = 0x5A;
static const uint8_t TYPE_FRAME    = 0x01;
static const uint8_t TYPE_COORDS   = 0x02;
static const uint8_t TYPE_TARGET   = 0x03;
static const uint8_t TYPE_ARRIVED  = 0x04;
static const uint8_t TYPE_HOME     = 0x05;
static const uint8_t TYPE_STREAM   = 0x06;
static const uint8_t TYPE_STATUS   = 0x07;
static const uint8_t TYPE_EXPOSURE = 0x08;
static const uint8_t TYPE_TEXT     = 0x09;
static const uint8_t TYPE_BUTTON   = 0x0A;

static const uint8_t ST_NOT_HOMED = 0;
static const uint8_t ST_HOMING    = 1;
static const uint8_t ST_HOMED     = 2;
static const uint8_t ST_FAILED    = 3;

uint8_t crc8(const uint8_t *data, size_t len) {
  uint8_t c = 0x00;
  for (size_t i = 0; i < len; i++) {
    c ^= data[i];
    for (uint8_t b = 0; b < 8; b++)
      c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     15
#define SIOD_GPIO_NUM     4
#define SIOC_GPIO_NUM     5
#define Y9_GPIO_NUM       16
#define Y8_GPIO_NUM       17
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       12
#define Y5_GPIO_NUM       10
#define Y4_GPIO_NUM       8
#define Y3_GPIO_NUM       9
#define Y2_GPIO_NUM       11
#define VSYNC_GPIO_NUM    6
#define HREF_GPIO_NUM     7
#define PCLK_GPIO_NUM     13
#define FRAME_SIZE        FRAMESIZE_VGA
#define JPEG_QUALITY      12          // nizsza liczba = lepsza jakosc (0-63)
#define DEBUG_EVERY_N     30

const bool INVERT_DIR_A = false;
const bool INVERT_DIR_B = false;

const int dirA = 2;
const int stepA = 1;
const int dirB = 42;
const int stepB = 41;
const int buttonX = 14;
const int buttonY = 21;

const float homingJump = 150.0f;
const float unhomingJump = 300.0f;
const float homingJumpPrecise = 3.0f;
const int homingPause = 0;

const float SPEED_MIN = 80.0f;
const float SPEED_MAX = 80000.0f;
const float ACCEL_MAX = 500000.0f;

const float maxX = 7750;
const float maxY = 12700;

long posA = 0;
long posB = 0;
float posX;
float posY;
float ACCEL = ACCEL_MAX;

portMUX_TYPE targetMux = portMUX_INITIALIZER_UNLOCKED;
volatile float targetX = 0.0f;
volatile float targetY = 0.0f;
volatile uint32_t targetVersion = 0;

volatile bool arrivedFlag = false;

float refX = 0.0f, refY = 0.0f;
long refA = 0, refB = 0;

bool homed = false;
uint8_t homingState = ST_NOT_HOMED;
bool motionTaskStarted = false;

uint32_t frameId = 0;
bool streaming = false;

volatile bool     fingerFound   = false;
volatile uint16_t fingerX       = 0;
volatile uint16_t fingerY       = 0;
volatile uint32_t coordsFrameId = 0;
volatile uint32_t coordsCount   = 0;

void onFingerCoords(uint32_t id, bool found, uint16_t x, uint16_t y) {
  fingerFound   = found;
  fingerX       = x;
  fingerY       = y;
  coordsFrameId = id;
  coordsCount++;
}

static inline float axisX() { return refX + 0.5f * (float)((posA - refA) - (posB - refB)); }
static inline float axisY() { return refY + 0.5f * (float)((posA - refA) + (posB - refB)); }

void setTarget(float x, float y) {
  if (x < 0) x = 0;
  if (x > maxX) x = maxX;
  if (y < 0) y = 0;
  if (y > maxY) y = maxY;
  portENTER_CRITICAL(&targetMux);
  targetX = x;
  targetY = y;
  targetVersion++;
  portEXIT_CRITICAL(&targetMux);
}

void getTarget(float *x, float *y, uint32_t *ver) {
  portENTER_CRITICAL(&targetMux);
  *x = targetX;
  *y = targetY;
  *ver = targetVersion;
  portEXIT_CRITICAL(&targetMux);
}

void sendStatus() {
  uint8_t sw = 0;
  if (digitalRead(buttonX)) sw |= 0x01;
  if (digitalRead(buttonY)) sw |= 0x02;
  uint8_t body[3] = { TYPE_STATUS, homingState, sw };
  uint8_t pkt[6] = { SYNC0, SYNC1, body[0], body[1], body[2], crc8(body, 3) };
  Serial.write(pkt, sizeof(pkt));
}

void sendArrived() {
  uint16_t x = (uint16_t)lroundf(posX);
  uint16_t y = (uint16_t)lroundf(posY);
  uint8_t body[5] = { TYPE_ARRIVED,
                      (uint8_t)(x & 0xFF), (uint8_t)(x >> 8),
                      (uint8_t)(y & 0xFF), (uint8_t)(y >> 8) };
  uint8_t pkt[8] = { SYNC0, SYNC1, body[0], body[1], body[2], body[3], body[4],
                     crc8(body, 5) };
  Serial.write(pkt, sizeof(pkt));
}

uint16_t layoutText(const char *text, uint8_t size, bool draw, int16_t yOff) {
  const uint8_t cols = OLED_W / (6 * size);
  const uint8_t rows = OLED_H / (8 * size);
  uint16_t line = 0;
  uint16_t i = 0;

  while (text[i] != '\0') {
    while (text[i] == ' ') i++;
    if (text[i] == '\0') break;
    if (text[i] == '\n') { i++; line++; continue; }

    uint16_t start = i;
    uint16_t lastSpace = 0;
    bool haveSpace = false;
    uint8_t n = 0;

    while (text[i] != '\0' && text[i] != '\n' && n < cols) {
      if (text[i] == ' ') { lastSpace = i; haveSpace = true; }
      i++;
      n++;
    }

    uint16_t end = i;
    if (text[i] != '\0' && text[i] != '\n' && haveSpace) {
      end = lastSpace;
      i = lastSpace + 1;
    }

    if (draw && line < rows) {
      display.setCursor(0, yOff + (int16_t)line * 8 * size);
      for (uint16_t k = start; k < end; k++) display.write(text[k]);
    }
    line++;
    if (text[i] == '\n') i++;
  }
  return line;
}

void showText(const char *text) {
  if (!oledReady) return;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextWrap(false);

  uint8_t size = 2;
  if (layoutText(text, 2, false, 0) > OLED_H / (8 * 2)) size = 1;

  display.setTextSize(size);
  uint16_t lines = layoutText(text, size, false, 0);
  uint8_t rows = OLED_H / (8 * size);
  int16_t yOff = 0;
  if (lines < rows) yOff = ((int16_t)rows - (int16_t)lines) * 8 * size / 2;

  layoutText(text, size, true, yOff);
  display.display();
}

void setupOled() {
  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);
  oledReady = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (!oledReady) {
    Serial0.printf("OLED: brak odpowiedzi pod adresem 0x%02X (SDA=%d SCL=%d).\n",
                   OLED_ADDR, OLED_SDA, OLED_SCL);
    Serial0.println("Sprawdz okablowanie i adres. Reszta firmware dziala normalnie.");
    return;
  }
  display.clearDisplay();
  display.display();
  Serial0.println("OLED gotowy.");
}

void sendButton() {
  uint16_t x = (uint16_t)lroundf(posX);
  uint16_t y = (uint16_t)lroundf(posY);
  uint8_t body[5] = { TYPE_BUTTON,
                      (uint8_t)(x & 0xFF), (uint8_t)(x >> 8),
                      (uint8_t)(y & 0xFF), (uint8_t)(y >> 8) };
  uint8_t pkt[8] = { SYNC0, SYNC1, body[0], body[1], body[2], body[3], body[4],
                     crc8(body, 5) };
  Serial.write(pkt, sizeof(pkt));
}

void pollButton() {
  static bool lastPressed = false;
  static uint32_t lockoutUntil = 0;

  uint32_t now = millis();
  if ((int32_t)(now - lockoutUntil) < 0) return;

  bool pressed = (digitalRead(BUTTON_PIN) == HIGH) == BUTTON_ACTIVE_HIGH;
  if (pressed == lastPressed) return;

  lastPressed = pressed;
  lockoutUntil = now + BUTTON_DEBOUNCE_MS;
  if (pressed) {
    sendButton();
    Serial0.printf("Przycisk: wcisniety przy X=%.0f Y=%.0f\n", posX, posY);
  }
}

void applyExposure(uint16_t aec, uint8_t gain) {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) { Serial0.println("Brak uchwytu sensora."); return; }

  if (aec == 0xFFFF) {
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_gain_ctrl(s, 1);
    Serial0.println("Naswietlanie: AUTO");
  } else {
    if (aec > 1200) aec = 1200;
    if (gain > 30) gain = 30;
    s->set_exposure_ctrl(s, 0);
    s->set_aec2(s, 0);
    s->set_gain_ctrl(s, 0);
    s->set_aec_value(s, aec);
    s->set_agc_gain(s, gain);
    Serial0.printf("Naswietlanie: RECZNE aec=%u gain=%u\n", aec, gain);
  }
}

void stepPulse(uint8_t pin) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(4);
  digitalWrite(pin, LOW);
}

void moveAB(long dA, long dB) {
  digitalWrite(dirA, ((dA >= 0) != INVERT_DIR_A) ? HIGH : LOW);
  digitalWrite(dirB, ((dB >= 0) != INVERT_DIR_B) ? HIGH : LOW);
  delayMicroseconds(10);

  long aA = labs(dA);
  long aB = labs(dB);
  long total = (aA > aB) ? aA : aB;
  if (total == 0) return;

  long slave = (aA < aB) ? aA : aB;
  bool aIsMaster = (aA >= aB);
  long err = total / 2;

  int sgnA = (dA > 0) ? 1 : -1;
  int sgnB = (dB > 0) ? 1 : -1;

  for (long i = 0; i < total; i++) {
    float vAcc = sqrtf(SPEED_MIN * SPEED_MIN + 2.0f * ACCEL * (float)i);
    float vDec = sqrtf(SPEED_MIN * SPEED_MIN + 2.0f * ACCEL * (float)(total - i));
    float v = vAcc;
    if (vDec < v) v = vDec;
    if (SPEED_MAX < v) v = SPEED_MAX;
    unsigned long interval = (unsigned long)(1000000.0f / v);

    if (aIsMaster) {
      stepPulse(stepA);
      posA += sgnA;
      err -= slave;
      if (err < 0) { err += total; stepPulse(stepB); posB += sgnB; }
    } else {
      stepPulse(stepB);
      posB += sgnB;
      err -= slave;
      if (err < 0) { err += total; stepPulse(stepA); posA += sgnA; }
    }

    delayMicroseconds(interval);
  }
}

void moveToolhead(float x, float y, bool pominacOgraniczenia = false) {
  if ((posX + x < 0 || posX + x > maxX || posY + y < 0 || posY + y > maxY) && !pominacOgraniczenia)
  {
    Serial0.println("Incorrect coordinates.");
    return;
  }
  posX += x;
  posY += y;
  moveAB(y + x, y - x);
}

float runSegment(long dA, long dB, float vEntry, uint32_t *seenVer) {
  digitalWrite(dirA, ((dA >= 0) != INVERT_DIR_A) ? HIGH : LOW);
  digitalWrite(dirB, ((dB >= 0) != INVERT_DIR_B) ? HIGH : LOW);
  delayMicroseconds(10);

  long aA = labs(dA);
  long aB = labs(dB);
  long total = (aA > aB) ? aA : aB;
  if (total == 0) return SPEED_MIN;

  long slave = (aA < aB) ? aA : aB;
  bool aIsMaster = (aA >= aB);
  long err = total / 2;

  int sgnA = (dA > 0) ? 1 : -1;
  int sgnB = (dB > 0) ? 1 : -1;

  float v = (vEntry > SPEED_MIN) ? vEntry : SPEED_MIN;
  bool stopping = false;

  for (long i = 0; i < total; i++) {
    long remaining = total - i;

    float vStop = sqrtf(SPEED_MIN * SPEED_MIN + 2.0f * ACCEL * (float)remaining);
    float dt = 1.0f / v;
    float vNext;
    if (stopping) {
      vNext = v - ACCEL * dt;
    } else {
      vNext = v + ACCEL * dt;
      if (vNext > SPEED_MAX) vNext = SPEED_MAX;
    }
    if (vNext > vStop) vNext = vStop;
    if (vNext < SPEED_MIN) vNext = SPEED_MIN;
    v = vNext;

    if (aIsMaster) {
      stepPulse(stepA);
      posA += sgnA;
      err -= slave;
      if (err < 0) { err += total; stepPulse(stepB); posB += sgnB; }
    } else {
      stepPulse(stepB);
      posB += sgnB;
      err -= slave;
      if (err < 0) { err += total; stepPulse(stepA); posA += sgnA; }
    }

    delayMicroseconds((unsigned long)(1000000.0f / v));

    if (stopping && v <= SPEED_MIN + 0.5f) return SPEED_MIN;

    if (*seenVer != targetVersion) {
      float tx, ty;
      uint32_t ver;
      getTarget(&tx, &ty, &ver);
      *seenVer = ver;

      float cx = axisX();
      float cy = axisY();
      long ndA = lroundf((ty - cy) + (tx - cx));
      long ndB = lroundf((ty - cy) - (tx - cx));
      long nTotal = (labs(ndA) > labs(ndB)) ? labs(ndA) : labs(ndB);

      bool dirOkA = (aA == 0) || (ndA == 0) || (((ndA > 0) ? 1 : -1) == sgnA);
      bool dirOkB = (aB == 0) || (ndB == 0) || (((ndB > 0) ? 1 : -1) == sgnB);
      bool feasible = (nTotal > 0) &&
                      (v * v <= SPEED_MIN * SPEED_MIN + 2.0f * ACCEL * (float)nTotal);

      if (dirOkA && dirOkB && feasible) {
        return v;
      } else {
        stopping = true;
      }
    }
  }
  return v;
}

void motionTask(void *arg) {
  uint32_t ver;
  {
    float tx, ty;
    getTarget(&tx, &ty, &ver);
  }
  float v = SPEED_MIN;
  bool announced = false;

  for (;;) {
    float tx, ty;
    getTarget(&tx, &ty, &ver);

    long dA = lroundf((ty - axisY()) + (tx - axisX()));
    long dB = lroundf((ty - axisY()) - (tx - axisX()));

    if (dA == 0 && dB == 0) {
      v = SPEED_MIN;
      posX = axisX();
      posY = axisY();
      if (!announced) {
        announced = true;
        arrivedFlag = true;
      }
      vTaskDelay(1);
      continue;
    }

    announced = false;
    v = runSegment(dA, dB, v, &ver);
    posX = axisX();
    posY = axisY();
  }
}

void runHoming() {
  homingState = ST_HOMING;
  sendStatus();

  Serial0.printf("Homing start. Krancowki: X=%d Y=%d\n",
                 digitalRead(buttonX), digitalRead(buttonY));

  ACCEL = 30000;
  
  long searchLoops = 0;

  while (digitalRead(buttonX) == LOW) {
    moveToolhead(homingJump, 0.0f, true);
    delayMicroseconds(homingPause);
    searchLoops++;
    yield();
  }
  moveToolhead(-unhomingJump, 0.0f, true);
  Serial0.println("Homed x axis.");

  while (digitalRead(buttonY) == LOW) {
    moveToolhead(0.0f, homingJump, true);
    delayMicroseconds(homingPause);
    searchLoops++;
    yield();
  }
  Serial0.println("Homed y axis.");
  moveToolhead(0.0f, -unhomingJump, true);

  while (digitalRead(buttonX) == LOW) {
    moveToolhead(homingJumpPrecise, 0.0f, true);
    delayMicroseconds(homingPause);
    searchLoops++;
    yield();
  }
  Serial0.println("Finished correcting x axis.");

  while (digitalRead(buttonY) == LOW) {
    moveToolhead(0.0f, homingJumpPrecise, true);
    delayMicroseconds(homingPause);
    searchLoops++;
    yield();
  }
  Serial0.println("Finished homing.");
  moveToolhead(-unhomingJump, -unhomingJump, true);

  ACCEL = ACCEL_MAX;
  
  if (searchLoops == 0) {
    homingState = ST_FAILED;
    homed = false;
    Serial0.println("BLAD HOMINGU: zadna petla szukania sie nie wykonala.");
    Serial0.printf("Odczyt krancowek: buttonX=%d buttonY=%d\n",
                   digitalRead(buttonX), digitalRead(buttonY));
    Serial0.println("Sprawdz podlaczenie i logike (NO/NC) przelacznikow.");
    sendStatus();
    return;
  }

  posX = maxX;
  posY = maxY;
  refX = posX;  refY = posY;
  refA = posA;  refB = posB;
  setTarget(posX, posY);

#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_config_t twdt_cfg = {
    .timeout_ms = 10000,
    .idle_core_mask = (1 << 1),
    .trigger_panic = false
  };
  esp_task_wdt_reconfigure(&twdt_cfg);
#else
  disableCore0WDT();
#endif

  if (!motionTaskStarted) {
    xTaskCreatePinnedToCore(motionTask, "motion", 8192, NULL, 2, NULL, 0);
    motionTaskStarted = true;
  }

  homed = true;
  homingState = ST_HOMED;
  Serial0.printf("Homing OK (%ld obrotow petli szukania). Silnik ruchu na core 0.\n",
                 searchLoops);
  sendStatus();
}

void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode    = CAMERA_GRAB_LATEST;
  config.fb_location  = CAMERA_FB_IN_PSRAM;

  if (psramFound()) {
    config.frame_size   = FRAME_SIZE;
    config.jpeg_quality = JPEG_QUALITY;
    config.fb_count     = 2;
  } else {
    Serial0.println("UWAGA: brak PSRAM! Wlacz OPI PSRAM w Tools.");
    config.frame_size   = FRAMESIZE_QVGA;
    config.jpeg_quality = 15;
    config.fb_count     = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial0.printf("BLAD INICJALIZACJI KAMERY: 0x%x\n", err);
    while (true) { delay(1000); }
  }
  Serial0.println("Kamera OK");

#if USE_AUTO_EXPOSURE
  applyExposure(0xFFFF, 0);
#else
  applyExposure(EXPOSURE_VALUE, GAIN_VALUE);
#endif
}

#define MAX_FPS               18
#define MIN_FRAME_INTERVAL_MS (1000 / MAX_FPS)

#define FRAME_TX_TIMEOUT_MS   400

uint32_t framesDropped = 0;

bool writeAll(const uint8_t *data, size_t len) {
  size_t sent = 0;
  uint32_t t0 = millis();
  while (sent < len) {
    size_t n = Serial.write(data + sent, len - sent);
    if (n > 0) {
      sent += n;
      t0 = millis();
    } else {
      if (millis() - t0 > FRAME_TX_TIMEOUT_MS) return false;
      delay(1);
    }
  }
  return true;
}

void sendFrame(camera_fb_t *fb) {
  uint8_t header[11];
  header[0] = SYNC0;
  header[1] = SYNC1;
  header[2] = TYPE_FRAME;
  header[3] = (uint8_t)(frameId        & 0xFF);
  header[4] = (uint8_t)((frameId >> 8)  & 0xFF);
  header[5] = (uint8_t)((frameId >> 16) & 0xFF);
  header[6] = (uint8_t)((frameId >> 24) & 0xFF);
  uint32_t len = fb->len;
  header[7]  = (uint8_t)(len        & 0xFF);
  header[8]  = (uint8_t)((len >> 8)  & 0xFF);
  header[9]  = (uint8_t)((len >> 16) & 0xFF);
  header[10] = (uint8_t)((len >> 24) & 0xFF);

  if (Serial.availableForWrite() < (int)sizeof(header)) {
    framesDropped++;
    return;
  }

  if (!writeAll(header, sizeof(header)) || !writeAll(fb->buf, fb->len)) {
    framesDropped++;
  }
}

void pollRx() {
  static uint8_t buf[4 + MAX_TEXT_LEN];
  static uint16_t pos = 0;
  static uint16_t need = 0;

  while (Serial.available()) {
    uint8_t b = (uint8_t)Serial.read();

    if (pos == 0) {
      if (b == SYNC0) buf[pos++] = b;
    } else if (pos == 1) {
      if (b == SYNC1) buf[pos++] = b;
      else pos = (b == SYNC0) ? 1 : 0;
    } else if (pos == 2) {
      if (b == TYPE_COORDS)        { buf[pos++] = b; need = 12; }
      else if (b == TYPE_TARGET)   { buf[pos++] = b; need = 7;  }
      else if (b == TYPE_STREAM)   { buf[pos++] = b; need = 4;  }
      else if (b == TYPE_EXPOSURE) { buf[pos++] = b; need = 6;  }
      else if (b == TYPE_TEXT)     { buf[pos++] = b; need = 0;  }
      else if (b == TYPE_HOME)     {
        pos = 0;
        if (!homed) runHoming();
        else { sendStatus(); sendArrived(); }
      }
      else pos = (b == SYNC0) ? 1 : 0;
    } else {
      if (pos < sizeof(buf)) buf[pos] = b;
      pos++;

      if (buf[2] == TYPE_TEXT && pos == 4) {
        need = 4 + (uint16_t)buf[3];
      }

      if (need != 0 && pos == need) {
        if (buf[2] == TYPE_COORDS) {
          uint32_t id = (uint32_t)buf[3] | ((uint32_t)buf[4] << 8) |
                        ((uint32_t)buf[5] << 16) | ((uint32_t)buf[6] << 24);
          bool     found = buf[7] != 0;
          uint16_t x = (uint16_t)buf[8]  | ((uint16_t)buf[9]  << 8);
          uint16_t y = (uint16_t)buf[10] | ((uint16_t)buf[11] << 8);
          onFingerCoords(id, found, x, y);
        } else if (buf[2] == TYPE_TARGET) {
          uint16_t x = (uint16_t)buf[3] | ((uint16_t)buf[4] << 8);
          uint16_t y = (uint16_t)buf[5] | ((uint16_t)buf[6] << 8);
          if (homed) setTarget((float)x, (float)y);
          else { Serial0.println("TARGET zignorowany: brak homingu."); sendStatus(); }
        } else if (buf[2] == TYPE_STREAM) {
          streaming = (buf[3] != 0);
          Serial0.printf("Strumien klatek: %s\n", streaming ? "ON" : "OFF");
        } else if (buf[2] == TYPE_EXPOSURE) {
          uint16_t aec = (uint16_t)buf[3] | ((uint16_t)buf[4] << 8);
          applyExposure(aec, buf[5]);
        } else if (buf[2] == TYPE_TEXT) {
          uint16_t n = buf[3];
          if (n > MAX_TEXT_LEN) n = MAX_TEXT_LEN;
          char txt[MAX_TEXT_LEN + 1];
          memcpy(txt, buf + 4, n);
          txt[n] = '\0';
          showText(txt);
          Serial0.printf("OLED: \"%s\"\n", txt);
        }
        pos = 0;
        need = 0;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(FRAME_TX_TIMEOUT_MS);
  Serial0.begin(115200);
  delay(500);
  Serial0.println("\n=== ESP32-S3 finger cam + CNC (protokol v3) ===");

  pinMode(dirA, OUTPUT);
  pinMode(stepA, OUTPUT);
  pinMode(dirB, OUTPUT);
  pinMode(stepB, OUTPUT);
  pinMode(buttonX, INPUT_PULLUP);
  pinMode(buttonY, INPUT_PULLUP);
  pinMode(BUTTON_PIN, BUTTON_ACTIVE_HIGH ? INPUT_PULLDOWN : INPUT_PULLUP);
  digitalWrite(stepA, LOW);
  digitalWrite(stepB, LOW);
  digitalWrite(dirA, LOW);
  digitalWrite(dirB, LOW);
  setupOled();

  setupCamera();

  Serial0.printf("Krancowki po starcie: buttonX=%d buttonY=%d\n",
                 digitalRead(buttonX), digitalRead(buttonY));
  Serial0.println("Strumien klatek WYLACZONY. Czekam na komendy z PC.");
}

void loop() {
  pollRx();
  pollButton();

  if (arrivedFlag) {
    arrivedFlag = false;
    sendArrived();
  }

  if (!streaming) {
    delay(2);
    return;
  }
  static uint32_t lastFrameMs = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastFrameMs < MIN_FRAME_INTERVAL_MS) {
    delay(1);
    return;
  }
  lastFrameMs = nowMs;

  camera_fb_t *fb = esp_camera_fb_get();
  if (fb) {
    sendFrame(fb);
    esp_camera_fb_return(fb);
    frameId++;
  }

  static uint32_t t0 = millis();
  if (frameId > 0 && frameId % DEBUG_EVERY_N == 0) {
    uint32_t now = millis();
    if (now - t0 > 500) {
      float fps = DEBUG_EVERY_N * 1000.0f / (float)(now - t0);
      t0 = now;
      Serial0.printf("TX fps=%.1f | porzucone=%lu | odp=%lu | palec: found=%d x=%u y=%u "
                     "| CNC: X=%.0f Y=%.0f\n",
                     fps, (unsigned long)framesDropped, (unsigned long)coordsCount,
                     (int)fingerFound, fingerX, fingerY, posX, posY);
    }
  }
}
