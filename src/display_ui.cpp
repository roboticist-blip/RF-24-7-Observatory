#include "display_ui.h"
#include "config.h"

#if !CFG_ENABLE_DISPLAY
bool uiInit(SPIClass&) { return false; }
void uiUpdate(uint64_t) {}
void uiSetBacklight(bool) {}
void uiForceRedraw() {}
#else

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include "shared_state.h"
#include "time_manager.h"
#include "system_health.h"
#include "environment.h"

namespace {
constexpr uint16_t rgb(uint8_t r, uint8_t g, uint8_t b) { return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)); }
const uint16_t C_BG = rgb(8, 8, 10), C_FG = 0xFFFF, C_GREY = rgb(150, 150, 155), C_DIM = rgb(80, 82, 90), C_GRID = rgb(30, 32, 38),
               C_ACC = rgb(0, 200, 255), C_OK = rgb(60, 220, 110), C_WARN = rgb(255, 176, 0), C_CRIT = rgb(255, 50, 50),
               C_HOLD = rgb(70, 100, 130);
// 16-entry waterfall palette (RGB565): near-black -> blue -> cyan -> white
const uint16_t PAL[16] = {0x0841, 0x0885, 0x08C9, 0x094D, 0x09F2, 0x02D7, 0x03FB, 0x051E,
                          0x061F, 0x1E9F, 0x571F, 0x7F5F, 0xA79F, 0xCFDF, 0xEFFF, 0xFFFF};

static_assert(CFG_UI_ROTATION >= 0 && CFG_UI_ROTATION <= 3, "TFT rotation must be 0..3");
constexpr bool PORTRAIT = (CFG_UI_ROTATION % 2 == 0);
constexpr int UI_W = PORTRAIT ? 240 : 320, UI_H = PORTRAIT ? 320 : 240;
constexpr int PLOT_X = 30, PLOT_W = UI_W - 32, PLOT_Y = PORTRAIT ? 48 : 26;
constexpr int PLOT_H = PORTRAIT ? 112 : 80, STRIP_H = 16;
constexpr int WF_Y = PORTRAIT ? 176 : 128, WF_ROWS = 64, WF_STRIDE = (RF_POINTS + 1) / 2;
constexpr int STAT_Y = PORTRAIT ? 240 : 192;
static_assert(PLOT_X + PLOT_W <= UI_W && WF_Y + WF_ROWS <= UI_H, "graph bounds");
static_assert(PLOT_H % STRIP_H == 0 && WF_ROWS % STRIP_H == 0, "strip height must divide regions");

Adafruit_ST7789 s_tft(&g_hspi, PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
constexpr const char* TFT_DRIVER = "ST7789";
bool s_ready = false;
uint16_t s_line[PLOT_W * STRIP_H];            // partial line buffer reused for spectrum and waterfall strips (6.5 KB portrait)
uint8_t s_wf[WF_ROWS][WF_STRIDE];             // 4-bit intensity indices
uint8_t s_wfHead = 0, s_wfCount = 0;
bool s_wfFull = true;
uint8_t s_binForX[PLOT_W];
uint16_t s_xOfBin[RF_POINTS];
uint8_t s_yCur[PLOT_W], s_yHold[PLOT_W];
uint8_t s_hold[RF_POINTS];
int s_vmin = CFG_UI_RSSI_MIN_DBM, s_vmax = CFG_UI_RSSI_MAX_DBM;
int s_gridY[5], s_tickX[5];
bool s_scaleDirty = true;
uint8_t s_scalePending = 0;
int s_pendMin = 0, s_pendMax = 0;
SweepSnapshot s_snap;
uint32_t s_lastSeq = 0;
uint64_t s_lastGraphMs = 0, s_lastTextMs = 0;
bool s_busUsed = false;

struct Field { int16_t x, y; uint8_t w, size; char last[24]; uint16_t fg; };
enum { F_SD, F_SYS, F_TIME, F_SUB1, F_SUB2, F_NOISE, F_PEAK, F_OCC, F_TEMP, F_PRESS, F_EVT, F_COUNT };
#if CFG_UI_ROTATION % 2 == 0
Field F[F_COUNT] = {
  {122, 4, 7, 1, "", 0}, {6, 14, 9, 1, "", 0}, {66, 14, 10, 1, "", 0},
  {6, 34, 10, 1, "", 0}, {78, 34, 10, 1, "", 0},
  {6, 255, 6, 2, "", 0}, {84, 255, 6, 2, "", 0}, {162, 255, 6, 2, "", 0},
  {6, 291, 12, 1, "", 0}, {84, 291, 12, 1, "", 0}, {162, 291, 12, 1, "", 0},
};
#else
Field F[F_COUNT] = {
  {122, 4, 7, 1, "", 0},  {172, 4, 9, 1, "", 0},  {256, 4, 10, 1, "", 0},
  {186, 14, 10, 1, "", 0}, {256, 14, 10, 1, "", 0},
  {6, 204, 6, 2, "", 0},  {112, 204, 6, 2, "", 0}, {218, 204, 4, 2, "", 0},
  {6, 231, 9, 1, "", 0},  {112, 231, 13, 1, "", 0}, {218, 231, 16, 1, "", 0},
};
#endif

void putField(Field& f, const char* s, uint16_t fg) {
  char tmp[24];
  snprintf(tmp, sizeof(tmp), "%-*s", (int)f.w, s);
  tmp[f.w < sizeof(tmp) ? f.w : sizeof(tmp) - 1] = 0;
  if (f.fg == fg && strcmp(tmp, f.last) == 0) return;
  strcpy(f.last, tmp);
  f.fg = fg;
  s_tft.setTextSize(f.size);
  s_tft.setTextColor(fg, C_BG);                 // opaque glyphs: no clear-then-draw flicker
  s_tft.setCursor(f.x, f.y);
  s_tft.print(tmp);
  s_busUsed = true;
}

void label(int x, int y, const char* t, uint16_t c) {
  s_tft.setTextSize(1);
  s_tft.setTextColor(c, C_BG);
  s_tft.setCursor(x, y);
  s_tft.print(t);
}

// SD and TFT share this bus. Only call from setup or the service task,
// after SD operations have returned. Reassert MADCTL before each render batch.
void selectTftOrientation() {
  digitalWrite(PIN_SD_CS, HIGH);
  s_tft.setRotation(CFG_UI_ROTATION);
  s_busUsed = true;
}

void pushRegion(int x, int y, int w, int h) {
  // Direct address-window writes bypass normal GFX clipping.
  if (x < 0 || y < 0 || w <= 0 || h <= 0 || x + w > UI_W || y + h > UI_H ||
      (uint32_t)w * (uint32_t)h > sizeof(s_line) / sizeof(s_line[0])) return;
  s_tft.startWrite();
  s_tft.setAddrWindow(x, y, w, h);
  s_tft.writePixels(s_line, (uint32_t)w * (uint32_t)h);
  s_tft.endWrite();
  s_busUsed = true;
}

int floor10(int v) { return v >= 0 ? (v / 10) * 10 : -(((-v) + 9) / 10) * 10; }
int ceil10(int v) { return -floor10(-v); }

int yMap(uint8_t lvl) {
  const int vmax2 = s_vmax * 2, span = (s_vmax - s_vmin) * 2;
  const int hd = (int)lvl - 255;
  int y = ((vmax2 - hd) * (PLOT_H - 1)) / span;
  return y < 0 ? 0 : (y > PLOT_H - 1 ? PLOT_H - 1 : y);
}

void buildTables() {
  for (int x = 0; x < PLOT_W; x++) s_binForX[x] = (uint8_t)(((uint32_t)x * (RF_POINTS - 1) + (PLOT_W - 1) / 2) / (PLOT_W - 1));
  for (int i = 0; i < (int)RF_POINTS; i++) s_xOfBin[i] = (uint16_t)(((uint32_t)i * (PLOT_W - 1) + (RF_POINTS - 1) / 2) / (RF_POINTS - 1));
  for (int k = 0; k < 5; k++) {
    s_gridY[k] = k * (PLOT_H - 1) / 4;
    s_tickX[k] = k * (PLOT_W - 1) / 4;
  }
}

void drawFreqLabels() {
  char t[12];
  for (int k = 0; k < 5; k++) {
    if (k & 1) continue; // five grid lines, three non-overlapping labels in either orientation
    const uint32_t f = RF_START_HZ + (uint32_t)((uint64_t)(RF_STOP_HZ - RF_START_HZ) * k / 4);
    snprintf(t, sizeof(t), "%lu.%02lu", (unsigned long)(f / 1000000UL), (unsigned long)((f % 1000000UL) / 10000UL));
    int x = PLOT_X + s_tickX[k] - 18;
    if (k == 0) x = PLOT_X;
    if (k == 4) x = PLOT_X + PLOT_W - 36;
    label(x, PLOT_Y + PLOT_H + 3, t, C_GREY);
  }
  s_busUsed = true;
}

void drawYLabels() {
  char t[8];
  for (int k = 0; k < 5; k++) {
    const int v = s_vmax - k * (s_vmax - s_vmin) / 4;
    snprintf(t, sizeof(t), "%-4d", v);
    int y = PLOT_Y + s_gridY[k] - 3;
    if (y < PLOT_Y - 2) y = PLOT_Y - 2;
    label(2, y, t, C_GREY);
  }
  s_scaleDirty = false;
  s_busUsed = true;
}

void drawStatic() {
  s_tft.fillScreen(C_BG);
  label(6, 4, "RF-24/7", C_ACC);
  label(62, 4, "RX-ONLY", C_OK);
  s_tft.drawFastHLine(0, PORTRAIT ? 44 : 23, UI_W, C_GRID);
  char t[40];
  snprintf(t, sizeof(t), "%lu.%02lu-%lu.%02lu MHz RBW %lu.%lukHz", (unsigned long)(RF_START_HZ / 1000000UL), (unsigned long)((RF_START_HZ % 1000000UL) / 10000UL),
           (unsigned long)(RF_STOP_HZ / 1000000UL), (unsigned long)((RF_STOP_HZ % 1000000UL) / 10000UL), (unsigned long)(CFG_RF_RXBW_HZ / 1000),
           (unsigned long)((CFG_RF_RXBW_HZ % 1000) / 100));
  // Leave the right-hand status fields free in landscape.
  if (!PORTRAIT) { char* rbw = strstr(t, " RBW"); if (rbw) *rbw = '\0'; }
  label(6, PORTRAIT ? 24 : 14, t, C_DIM);
  drawFreqLabels();
  drawYLabels();
  label(4, WF_Y + 2, "WF", C_DIM);
  s_tft.drawFastHLine(0, WF_Y - 1, UI_W, C_GRID);
  s_tft.drawFastHLine(0, STAT_Y, UI_W, C_GRID);
  if (PORTRAIT) {
    label(6, 244, "NOISE dBm", C_GREY);
    label(84, 244, "PEAK dBm", C_GREY);
    label(162, 244, "OCC %", C_GREY);
    label(6, 279, "TEMP", C_GREY);
    label(84, 279, "PRESSURE", C_GREY);
    label(162, 279, "EVENTS", C_GREY);
  } else {
    label(6, 194, "NOISE", C_GREY);  label(80, 212, "dBm", C_DIM);
    label(112, 194, "PEAK", C_GREY); label(186, 212, "dBm", C_DIM);
    label(218, 194, "OCCUPANCY", C_GREY); label(268, 212, "%", C_DIM);
    label(6, 222, "TEMP", C_GREY); label(112, 222, "PRESSURE", C_GREY); label(218, 222, "EVENTS", C_GREY);
  }
  for (int i = 0; i < F_COUNT; i++) { F[i].last[0] = 1; F[i].last[1] = 0; F[i].fg = 0; }   // force text repaint
  s_wfFull = true;
}

// ---------------------------------------------------------------- spectrum
void updateHold() {
  for (uint16_t i = 0; i < RF_POINTS; i++) {
    const uint8_t l = s_snap.raw[i] == RF_RAW_INVALID ? 0 : rfRawToLvl(s_snap.raw[i]);
    uint8_t h = s_hold[i];
    h = h > CFG_UI_HOLD_DECAY_LVL ? (uint8_t)(h - CFG_UI_HOLD_DECAY_LVL) : 0;
    s_hold[i] = l > h ? l : h;
  }
}

void updateScale() {
#if CFG_UI_SCALE_ADAPTIVE
  const int noise = ((int)(s_snap.g.med_q8 >> 8) - 255) / 2;
  uint8_t mx = 0;
  for (uint16_t i = 0; i < RF_POINTS; i++) if (s_hold[i] > mx) mx = s_hold[i];
  const int peak = ((int)mx - 255) / 2;
  int tmin = floor10(noise - 10), tmax = ceil10(peak + 5);
  if (tmax - tmin < 40) tmax = tmin + 40;
  if (tmax > 0) tmax = 0;
  if (tmin < -130) tmin = -130;
  if (tmax - tmin < 40) tmax = tmin + 40;
  if (tmin == s_vmin && tmax == s_vmax) { s_scalePending = 0; return; }
  if (s_lastSeq <= 1) { s_vmin = tmin; s_vmax = tmax; s_scaleDirty = true; return; }
  if (tmin == s_pendMin && tmax == s_pendMax) s_scalePending++;
  else { s_pendMin = tmin; s_pendMax = tmax; s_scalePending = 1; }
  if (s_scalePending >= 4) { s_vmin = tmin; s_vmax = tmax; s_scaleDirty = true; s_scalePending = 0; }   // hysteresis against scale hunting
#endif
}

void computeColumns(const uint8_t* lvlOrRaw, bool isRaw, uint8_t* yCol) {
  uint8_t yb[RF_POINTS];
  for (uint16_t i = 0; i < RF_POINTS; i++) {
    uint8_t l = lvlOrRaw[i];
    if (isRaw) l = (l == RF_RAW_INVALID) ? 0 : rfRawToLvl(l);
    yb[i] = (uint8_t)yMap(l);
  }
  for (int x = 0; x < PLOT_W; x++) {
    int i = (int)(((uint32_t)x * (RF_POINTS - 1)) / (PLOT_W - 1));
    if (i > (int)RF_POINTS - 2) i = RF_POINTS - 2;
    int x0 = s_xOfBin[i], x1 = s_xOfBin[i + 1];
    if (x1 <= x0) { yCol[x] = yb[i]; continue; }
    int xx = x < x0 ? x0 : (x > x1 ? x1 : x);
    yCol[x] = (uint8_t)(yb[i] + ((int)yb[i + 1] - (int)yb[i]) * (xx - x0) / (x1 - x0));
  }
}

void drawSpan(const uint8_t* y, int x, int y0, uint16_t color) {
  const int prev = x > 0 ? y[x - 1] : y[x];
  int lo = prev < y[x] ? prev : y[x], hi = prev < y[x] ? y[x] : prev;
  if (hi < y0 || lo >= y0 + STRIP_H) return;
  if (lo < y0) lo = y0;
  if (hi > y0 + STRIP_H - 1) hi = y0 + STRIP_H - 1;
  for (int yy = lo; yy <= hi; yy++) s_line[(yy - y0) * PLOT_W + x] = color;
}

void drawSpectrum() {
  computeColumns(s_snap.raw, true, s_yCur);
  computeColumns(s_hold, false, s_yHold);
  for (int s = 0; s < PLOT_H / STRIP_H; s++) {
    const int y0 = s * STRIP_H;
    for (int i = 0; i < PLOT_W * STRIP_H; i++) s_line[i] = C_BG;
    for (int k = 0; k < 5; k++) {
      const int g = s_gridY[k];
      if (g >= y0 && g < y0 + STRIP_H) for (int x = 0; x < PLOT_W; x++) s_line[(g - y0) * PLOT_W + x] = C_GRID;
      for (int r = 0; r < STRIP_H; r++) s_line[r * PLOT_W + s_tickX[k]] = C_GRID;
    }
    for (int x = 0; x < PLOT_W; x++) drawSpan(s_yHold, x, y0, C_HOLD);
    for (int x = 0; x < PLOT_W; x++) drawSpan(s_yCur, x, y0, C_ACC);
    pushRegion(PLOT_X, PLOT_Y + y0, PLOT_W, STRIP_H);
  }
}

// ---------------------------------------------------------------- waterfall
inline uint8_t wfGet(uint8_t row, uint16_t bin) { const uint8_t v = s_wf[row][bin >> 1]; return (bin & 1) ? (uint8_t)(v >> 4) : (uint8_t)(v & 0x0F); }

void wfPush() {
  s_wfHead = (uint8_t)((s_wfHead + 1) % WF_ROWS);
  if (s_wfCount < WF_ROWS) s_wfCount++;
  memset(s_wf[s_wfHead], 0, WF_STRIDE);
  for (uint16_t i = 0; i < RF_POINTS; i++) {
    uint8_t idx = 0;
    if (s_snap.raw[i] != RF_RAW_INVALID) {
      const int ex = (int)rfRawToLvl(s_snap.raw[i]) - (int)s_snap.noise_lvl[i];
      if (ex > CFG_UI_WF_DEAD_LVL) { int q = 1 + (ex - CFG_UI_WF_DEAD_LVL) / CFG_UI_WF_STEP_LVL; idx = (uint8_t)(q > 15 ? 15 : q); }
    }
    s_wf[s_wfHead][i >> 1] |= (i & 1) ? (uint8_t)(idx << 4) : idx;
  }
}

void wfBuildRow(uint8_t ringRow, uint16_t* dst) {
  for (int x = 0; x < PLOT_W; x++) dst[x] = PAL[wfGet(ringRow, s_binForX[x])];
}

void drawWaterfall() {
#if CFG_UI_WATERFALL_SCROLL
  for (int s = 0; s < WF_ROWS / STRIP_H; s++) {
    for (int r = 0; r < STRIP_H; r++) {
      const int logical = s * STRIP_H + r;                   // 0 = newest, at the top
      uint16_t* d = &s_line[r * PLOT_W];
      if (logical < s_wfCount) wfBuildRow((uint8_t)((s_wfHead + WF_ROWS - logical) % WF_ROWS), d);
      else for (int x = 0; x < PLOT_W; x++) d[x] = C_BG;
    }
    pushRegion(PLOT_X, WF_Y + s * STRIP_H, PLOT_W, STRIP_H);
  }
#else
  // in-place raster: only the new row and a moving marker row touch the bus
  if (s_wfFull) {
    for (int s = 0; s < WF_ROWS / STRIP_H; s++) {
      for (int r = 0; r < STRIP_H; r++) {
        const int row = s * STRIP_H + r;
        if (row < s_wfCount || s_wfCount == WF_ROWS) wfBuildRow((uint8_t)row, &s_line[r * PLOT_W]);
        else for (int x = 0; x < PLOT_W; x++) s_line[r * PLOT_W + x] = C_BG;
      }
      pushRegion(PLOT_X, WF_Y + s * STRIP_H, PLOT_W, STRIP_H);
    }
    s_wfFull = false;
  }
  wfBuildRow(s_wfHead, s_line);
  pushRegion(PLOT_X, WF_Y + s_wfHead, PLOT_W, 1);
  for (int x = 0; x < PLOT_W; x++) s_line[x] = C_DIM;
  pushRegion(PLOT_X, WF_Y + (s_wfHead + 1) % WF_ROWS, PLOT_W, 1);
#endif
}

// ---------------------------------------------------------------- text
void updateTexts(uint64_t nowMs) {
  char b[24];
  switch (g_metrics.sd_state) {
    case SD_OK: putField(F[F_SD], "SD OK", C_OK); break;
    case SD_ABSENT: putField(F[F_SD], "NO SD", C_WARN); break;
    case SD_ERROR: putField(F[F_SD], "SD ERR", C_WARN); break;
    case SD_FULL: putField(F[F_SD], "SD FULL", C_WARN); break;
    default: putField(F[F_SD], "SD OFF", C_DIM); break;
  }
  if (g_metrics.rf_state == RF_FAULT || g_metrics.rf_state == RF_INIT) putField(F[F_SYS], "RF FAIL", C_CRIT);   // red only for critical
  else if (g_metrics.rf_state == RF_RECOVERING) putField(F[F_SYS], "RF RECOV", C_WARN);
  else {
    const HealthState hs = healthLast();
    putField(F[F_SYS], hs == HEALTH_OK ? "SYS OK" : (hs == HEALTH_WARN ? "SYS WARN" : "SYS FAULT"), hs == HEALTH_OK ? C_OK : (hs == HEALTH_WARN ? C_WARN : C_CRIT));
  }
  if (timeSynced()) {
    const uint32_t e = timeEpoch() % 86400UL;
    snprintf(b, sizeof(b), "%02lu:%02lu:%02luZ", (unsigned long)(e / 3600), (unsigned long)((e / 60) % 60), (unsigned long)(e % 60));
    putField(F[F_TIME], b, C_FG);
  } else {
    char u[12];
    timeFormatUptime(u, sizeof(u), nowMs);
    snprintf(b, sizeof(b), "UP %s", u);
    putField(F[F_TIME], b, C_GREY);
  }
  if (s_lastSeq == 0) {
    putField(F[F_SUB1], "SW --", C_DIM); putField(F[F_SUB2], "", C_DIM);
    putField(F[F_NOISE], "--", C_FG); putField(F[F_PEAK], "--", C_FG); putField(F[F_OCC], "--", C_FG);
  } else {
    snprintf(b, sizeof(b), "SW %lu", (unsigned long)s_snap.id);
    putField(F[F_SUB1], b, C_GREY);
    snprintf(b, sizeof(b), "OPEN %u", s_snap.events_open);
    putField(F[F_SUB2], b, s_snap.events_open ? C_WARN : C_GREY);
    snprintf(b, sizeof(b), "%.1f", (s_snap.g.med_q8 / 256.0f - 255.0f) * 0.5f);
    putField(F[F_NOISE], b, C_FG);
    snprintf(b, sizeof(b), "%.1f", rfLvlToDbm(s_snap.g.peak_sweep_lvl));
    putField(F[F_PEAK], b, C_FG);
    snprintf(b, sizeof(b), "%u", s_snap.g.occ_avg_pct);
    putField(F[F_OCC], b, s_snap.g.occ_avg_pct >= 50 ? C_WARN : C_FG);
  }
  EnvData env;
  sharedGetEnv(env);
  if (env.valid) {
    snprintf(b, sizeof(b), "%.1f C", env.t_c100 / 100.0f); putField(F[F_TEMP], b, C_FG);
    snprintf(b, sizeof(b), "%.1f hPa", env.p_pa / 100.0f); putField(F[F_PRESS], b, C_FG);
  } else {
    putField(F[F_TEMP], "-- C", C_DIM); putField(F[F_PRESS], "-- hPa", C_DIM);
  }
  snprintf(b, sizeof(b), "%lu", (unsigned long)g_metrics.events_started);
  putField(F[F_EVT], b, C_FG);
}
}  // namespace

bool uiInit(SPIClass&) {
  if (!g_ctl.allow_display) return false;
  pinMode(PIN_TFT_CS, OUTPUT);
  digitalWrite(PIN_TFT_CS, HIGH);
  s_tft.init(240, 320, SPI_MODE0);
  s_tft.setSPISpeed(CFG_UI_SPI_HZ); // applies to drawing after library initialization
  selectTftOrientation();
  if (s_tft.width() != UI_W || s_tft.height() != UI_H) return false;
  s_tft.setTextWrap(false);
  Serial.printf("TFT configured: driver=%s, rotation=%u, logical=%dx%d, SPI=%lu Hz (software geometry)\n", TFT_DRIVER, (unsigned)CFG_UI_ROTATION, s_tft.width(), s_tft.height(), (unsigned long)CFG_UI_SPI_HZ);
  buildTables();
  memset(s_hold, 0, sizeof(s_hold));
  memset(s_wf, 0, sizeof(s_wf));
  drawStatic();
  uiSetBacklight(true);
  s_ready = true;
  return true;
}

void uiSetBacklight(bool on) {
#if CFG_TFT_BL_ENABLE
  static bool init = false;
  if (!init) { pinMode(PIN_TFT_BL, OUTPUT); init = true; }
  if (!on && !CFG_TFT_BL_SWITCHABLE) on = true;   // never "switch off" a backlight that is not really switched
  digitalWrite(PIN_TFT_BL, on ? HIGH : LOW);
  g_ctl.bl_state = on;
#else
  g_ctl.bl_state = true;
  (void)on;
#endif
}

void uiForceRedraw() {
  if (!s_ready) return;
  g_ctl.tft_busy = true;
  selectTftOrientation();
  drawStatic();
  // Restore the latest snapshot without adding an artificial waterfall row.
  if (s_lastSeq != 0) {
    drawSpectrum();
    if (s_wfCount) drawWaterfall();
  }
  updateTexts(timeMonoMs());
  g_ctl.tft_ops++;
  g_ctl.tft_busy = false;
}

void uiUpdate(uint64_t nowMs) {
  if (!s_ready) return;
  const uint32_t seq = sharedSweepSeq();
  const bool fresh = seq != 0 && seq != s_lastSeq;
  s_busUsed = false;
  g_ctl.tft_busy = true;
  if (fresh && (nowMs - s_lastGraphMs) >= CFG_UI_MIN_GRAPH_MS) {
    selectTftOrientation();
    s_lastSeq = sharedGetSweep(s_snap);
    s_lastGraphMs = nowMs;
    updateHold();
    updateScale();
    if (s_scaleDirty) drawYLabels();
    drawSpectrum();
    wfPush();
    drawWaterfall();
    updateTexts(nowMs);
    s_lastTextMs = nowMs;
  } else if ((nowMs - s_lastTextMs) >= CFG_UI_TEXT_MS) {
    selectTftOrientation();
    s_lastTextMs = nowMs;
    updateTexts(nowMs);
  }
  if (s_busUsed) g_ctl.tft_ops++;
  g_ctl.tft_busy = false;
}
#endif
