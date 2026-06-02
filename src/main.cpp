#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <B612_Regular5pt7b.h>
#include <hardware/pio.h>
#include <hardware/timer.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TFT_CS  21
#define TFT_DC  19
#define TFT_RST 18
#define TFT_SCK 26
#define TFT_MOSI 27
#define BTN_PIN  0
#define LED_PIN   25
#define POT_PIN   28  // Potentiometer — GPIO 28 (ADC2)

const int ENC_A = 10;
const int ENC_B = 11;
const int ENC_BTN = 12;

Adafruit_ST7735 tft(&SPI1, TFT_CS, TFT_DC, TFT_RST);

int dispW = 0;
int dispH = 0;

volatile float pitch=0, roll=0, hdg=0, alt=0, ias=0, vs=0;
const float PITCH_SCALE = 1.5f;
volatile float aoa=0, gLoad=1.0f, groundSpeed=0;
volatile float sp=0;  // mach number
volatile float engineRpmLeft=0, engineRpmRight=0;
volatile float windSpeedKt=0, windDirDeg=0;
volatile bool isStalling = false;
volatile bool isOverspeed = false;
volatile bool enginesOn = true;
volatile bool parkingBrake = false;
volatile float gearPosition  = 0.0f;
volatile float flapsPosition    = 0.0f;   // 0-1 normalised
volatile float spoilersPosition = 0.0f;   // 0-1 normalised
volatile char gpwsAlarm[32] = "";
volatile bool  groundContact = false;
volatile float haglFt = 0.0f;  // height above ground level (AGL) in feet

// ---- throttle (potentiometer) ----
volatile float potThrottle = 0.0f;  // 0.0-1.0 from pot
static int lastPotRaw = -1;          // track change

// ---- cross-core flags ----
volatile uint8_t activeTab   = 0;
volatile bool forceRedraw    = false;
volatile bool telemetryUpdated = false;
volatile bool apEnabled      = false;
volatile bool clearOnRedraw  = false;


// ---- display tuning ----
const uint16_t COL_SKY    = 0x34BF;  // RGB565: ~#3399FF sky blue
const uint16_t COL_GROUND = 0xCA40;  // RGB565: ~#C8A000 warm sandy tan
const uint16_t COL_PANEL  = 0x2164;
const uint16_t COL_PANEL_EDGE = 0x7BEF;
const uint16_t COL_PANEL_DARK = 0x0861;
const uint16_t COL_MARK   = ST77XX_WHITE;
const uint16_t COL_ACCENT = 0x07FF;  // cyan
const uint16_t COL_LADDER_DIM = 0xC618;
const int TAPE_W = 27;  // width of airspeed and altitude tapes on sides of horizon

bool inRoundedRect(int x, int y, int left, int top, int width, int height, int radius){
  int right = left + width - 1;
  int bottom = top + height - 1;

  if(x < left || x > right || y < top || y > bottom){
    return false;
  }

  int innerLeft = left + radius;
  int innerRight = right - radius;
  int innerTop = top + radius;
  int innerBottom = bottom - radius;

  if((x >= innerLeft && x <= innerRight) || (y >= innerTop && y <= innerBottom)){
    return true;
  }

  int dx = 0;
  int dy = 0;
  if(x < innerLeft) dx = innerLeft - x;
  else if(x > innerRight) dx = x - innerRight;
  if(y < innerTop) dy = innerTop - y;
  else if(y > innerBottom) dy = y - innerBottom;

  return (dx * dx + dy * dy) <= (radius * radius);
}

static inline uint32_t now_ms(){
  return (uint32_t)to_ms_since_boot(get_absolute_time());
}

bool checkButtonPress(int pin) {
  struct PinState { bool last; uint32_t edgeMs; };
  static PinState states[32] = {};  // one slot per GPIO (Pico has 0-29)
  PinState &s = states[pin];
  bool pressed = (digitalRead(pin) == LOW);
  uint32_t now = now_ms();
  bool fired = (!pressed && s.last && (now - s.edgeMs >= 25));
  if(pressed != s.last) s.edgeMs = now;
  s.last = pressed;
  return fired;
}

// Encoder selection mode: -1 = none, 0 = HDG, 1 = ALT, 2 = IAS

volatile int encMode   = -1;   // was: volatile int encMode   = -1;
volatile int targetHdg = 0;    // was: int targetHdg = 0;
volatile int targetAlt = 0;    // was: int targetAlt = 0;
volatile int targetIas = 0;   // knots
volatile int32_t encAccum  = 0; 
volatile uint8_t encLastAB = 0;
static const int8_t encQTable[16] = {
   0,+1,-1, 0,
  -1, 0, 0,+1,
  +1, 0, 0,-1,
   0,-1,+1, 0
};
struct repeating_timer encTimer;
bool encTimerCb(struct repeating_timer *t){
  uint8_t ab    = (gpio_get(ENC_A) << 1) | gpio_get(ENC_B);
  uint8_t idx2  = (encLastAB << 2) | ab;
  encLastAB     = ab;
  encAccum     += encQTable[idx2];
  return true;
}

// AP flicker guard: after receiving an ACK, ignore telemetry AP field for this many ms
volatile uint32_t apAckLockUntilMs = 0;


char buf[320]; int idx=0;
uint32_t lastTelemetryMs = 0;

// ---- parse ----
void parseLine(char *s){
  // Field-indexed CSV tokenizer — handles mixed numeric/string fields by position.
  // JS output order (0-based):
  //  0 pitch  1 roll  2 hdg  3 alt  4 ias  5 vs  6 mach  7 aoa  8 gLoad  9 qnh
  // 10 turnRate(unused)  11 groundSpeedKt  12 isStalling  13 isOverspeed
  // 14 enginesOn  15 parkingBrake  16-18 reserved(0)
  // 19 gpwsAlarm(str)  20 totalThrust  21 leftThrust  22 rightThrust
  // 23 engineRpm  24 engineRpmLeft  25 engineRpmRight
  // 26 gearPos  27 flapsPos  28 spoilersPos
  // 29 windSpeedKt  30 windDirDeg  31 haglFt  32 groundContact  33 apOn

  // Strip trailing CR/LF
  char *nl = strpbrk(s, "\r\n");
  if(nl) *nl = '\0';

  const int MAX_FIELDS = 40;
  char *fields[MAX_FIELDS] = {};
  int   nFields = 0;
  char *p = s;
  while(*p && nFields < MAX_FIELDS){
    fields[nFields++] = p;
    char *comma = strchr(p, ',');
    if(!comma) break;
    *comma = '\0';
    p = comma + 1;
  }

  if(nFields < 6) return;

  auto getF = [&](int i) -> float {
    if(i >= nFields || !fields[i] || !*fields[i]) return 0.0f;
    char *e = nullptr;
    float v = strtof(fields[i], &e);
    return (e != fields[i]) ? v : 0.0f;
  };
  auto getS = [&](int i, volatile char *dst, int dstLen)
  {
    if (i >= nFields || !fields[i])
    {
      dst[0] = '\0';
      return;
    }
    strncpy((char*)dst, fields[i], dstLen - 1);
    dst[dstLen - 1] = '\0';
  };

  // Reset boolean flags to safe defaults before field assignment
  // (getF returns 0.0 for missing fields, which would make enginesOn=false)
  isStalling  = false;
  isOverspeed = false;
  enginesOn   = true;
  parkingBrake = false;
  groundContact = false;

  pitch          = getF(0);
  roll           = getF(1);
  hdg            = getF(2);
  alt            = getF(3);
  ias            = getF(4);
  vs             = getF(5);
  sp = getF(6);   // mach number
  aoa            = getF(7);
  gLoad          = getF(8);
  // getF(10) = turnRate — not displayed, skipped
  groundSpeed    = getF(11);
  isStalling     = getF(12) > 0.5f;
  isOverspeed    = getF(13) > 0.5f;
  enginesOn      = getF(14) > 0.5f;
  parkingBrake   = getF(15) > 0.5f;
  getS(19, gpwsAlarm,    sizeof(gpwsAlarm));
  engineRpmLeft  = getF(24);
  engineRpmRight = getF(25);
  gearPosition     = getF(26);
  flapsPosition    = getF(27);
  spoilersPosition = getF(28);
  windSpeedKt      = getF(29);
  windDirDeg       = getF(30);
  haglFt           = getF(31);  // height above ground level (AGL) in feet
  groundContact    = getF(32) > 0.5f;
  // Only update apEnabled from telemetry if not within an ACK lock window.
  // This prevents the flickering caused by the telemetry stream momentarily
  // contradicting a just-received ACK,AP,ON/OFF before GeoFS updates ap.on.
  if(now_ms() >= apAckLockUntilMs){
    apEnabled = getF(33) > 0.5f;
  }

  lastTelemetryMs = now_ms();  // Track when telemetry was last received
  telemetryUpdated = true; // signal display to redraw

}

const uint32_t TELEMETRY_TIMEOUT_MS = 500;  // If no telemetry for 500ms, consider disconnected

// Forward declarations
// Horizon renderer — paints EVERY pixel of the panel each call, no pre-clear needed.
//
// Coordinate system:
//   Screen Y increases downward. Top of panel: dy = y - cy < 0. Bottom: dy > 0.
//   At zero pitch/roll, top = SKY, bottom = GROUND.
//
// Horizon plane equation for a pixel at (dx, dy) from panel centre:
//   f(dx,dy) = dx*sr + dy*cr - pitchOffset
//   f > 0  →  GROUND side
//   f < 0  →  SKY side
//   f == 0 →  horizon line
//
// Verification at zero pitch/roll (sr=0, cr=1, pitchOffset=0):
//   f = dy  →  dy<0 (top) is SKY ✓, dy>0 (bottom) is GROUND ✓
//
// Pitch: positive = nose-up → horizon moves down → pitchOffset shifts boundary down
//   f = dy - pitchOffset  →  more rows have dy < pitchOffset → more sky ✓
//
// Roll (negated for pilot view): left bank in GeoFS (negative htr[2]) shows
//   as right-tilt on display (real cockpit convention).
void drawHorizonFast(float pitch, float roll, int x0, int y0, int w, int h){
  const int cornerRadius = 8;  // Unified radius — matches post-draw clip and border
  float rad = (-roll) * DEG_TO_RAD;  // negate: pilot-view (left bank shows right-tilt)
  float sr  = sinf(rad), cr = cosf(rad);
  int   cx  = x0 + w / 2;
  int   cy  = y0 + h / 2;
  float pitchOffset = -pitch * PITCH_SCALE;

  for(int y = y0; y < y0 + h; y++){
    float dy = (float)(y - cy);

    // Always clip rounded corners — compute x bounds for this scanline
    int xLeft  = x0;
    int xRight = x0 + w;

    // Top corner rows: shrink xLeft/xRight according to circle geometry
    if(y < y0 + cornerRadius){
      int dy2 = y0 + cornerRadius - 1 - y;
      int dx  = (int)sqrtf((float)(cornerRadius * cornerRadius - dy2 * dy2));
      xLeft  = x0 + (cornerRadius - 1 - dx);
      xRight = x0 + w - (cornerRadius - 1 - dx);
    } else if(y >= y0 + h - cornerRadius){
      int dy2 = y - (y0 + h - cornerRadius);
      int dx  = (int)sqrtf((float)(cornerRadius * cornerRadius - dy2 * dy2));
      xLeft  = x0 + (cornerRadius - 1 - dx);
      xRight = x0 + w - (cornerRadius - 1 - dx);
    }
    int rowW = xRight - xLeft;
    if(rowW <= 0) continue;

    // Black out margin pixels on this row (left and right of rounded window)
    if(xLeft > x0)
      tft.drawFastHLine(x0, y, xLeft - x0, ST77XX_BLACK);
    if(xRight < x0 + w)
      tft.drawFastHLine(xRight, y, (x0 + w) - xRight, ST77XX_BLACK);

    if(fabsf(sr) < 0.015f){
      float f0 = dy * cr - pitchOffset;
      if(fabsf(f0) <= 1.2f)
        tft.drawFastHLine(xLeft, y, rowW, COL_MARK);
      else
        tft.drawFastHLine(xLeft, y, rowW, f0 < 0 ? COL_SKY : COL_GROUND);
      continue;
    }

    // Crossing: dx*sr + dy*cr - pitchOffset = 0  →  xCross = cx + (pitchOffset - dy*cr)/sr
    float xCrossF = (float)cx + (pitchOffset - dy * cr) / sr;
    int xi = (int)xCrossF;

    // Full-row cases — crossing outside clipped row
    if(xi + 1 < xLeft){
      float f = (float)(xLeft - cx) * sr + dy * cr - pitchOffset;
      tft.drawFastHLine(xLeft, y, rowW, f < 0 ? COL_SKY : COL_GROUND);
      continue;
    }
    if(xi >= xRight){
      float f = (float)(xRight - 1 - cx) * sr + dy * cr - pitchOffset;
      tft.drawFastHLine(xLeft, y, rowW, f < 0 ? COL_SKY : COL_GROUND);
      continue;
    }

    float fLeft = (float)(xLeft - cx) * sr + dy * cr - pitchOffset;
    uint16_t leftCol  = (fLeft < 0) ? COL_SKY : COL_GROUND;
    uint16_t rightCol = (fLeft < 0) ? COL_GROUND : COL_SKY;

    int leftEnd = min(xi, xRight);
    if(leftEnd > xLeft)
      tft.drawFastHLine(xLeft, y, leftEnd - xLeft, leftCol);

    int sa = max(xi, xLeft);
    int sb = min(xi + 1, xRight - 1);
    if(sb >= sa)
      tft.drawFastHLine(sa, y, sb - sa + 1, COL_MARK);

    int rx = max(xi + 2, xLeft);
    if(rx <= xRight - 1)
      tft.drawFastHLine(rx, y, xRight - rx, rightCol);
  }
}

void drawPitchLadder(Adafruit_GFX &g, float pitch, float roll, int cx, int cy, int xLeft, int xRight, int yTop, int yBottom){
  // Matches horizon convention exactly:
  //   rad = -roll*DEG_TO_RAD,  s_r = sin(-roll), c_r = cos(-roll)
  //   pitchOffset = -pitch * PITCH_SCALE  (same sign as horizon)
  //   sky-normal on screen = direction that increases f = (sr, cr) normalised
  //   "toward sky" = DECREASING f, so move in direction -(sr, cr) = (-sr, -cr)
  //   But with rad=-roll: sr=-sin(roll), cr=cos(roll)
  //   sky-normal screen = (-sr, -cr) = (sin(roll), -cos(roll)) ✓ (upper-right for right bank)
  //
  //   Tick tangent (along horizon): perpendicular to normal = (cr, sr) rotated 90°
  //   = (-cr_perp, sr_perp) — but simpler: horizon direction = (c_r, -(-s_r)) = (c_r, s_r)
  //   Wait — let's just derive from scratch cleanly:
  //   horizon line: dx*sr + dy*cr - pitchOffset = 0, parametric: (t*cr, -t*sr) from centre
  //   So tick direction = (cr, -sr) in (dx,dy) = screen (c_r, -s_r) ✓
  float rad = (-roll) * DEG_TO_RAD;
  float s_r = sinf(rad), c_r = cosf(rad);   // s_r = -sin(roll), c_r = cos(roll)
  const int cornerRadius = 8;

  for(int p = -60; p <= 60; p += 10){
    if(p == 0) continue;

    // This pitch line sits where pitchOffset = -p * PITCH_SCALE.
    // Delta from aircraft symbol pitchOffset to this line's pitchOffset:
    //   delta = (-p*scale) - (-pitch*scale) = (pitch-p)*scale
    // The pitch line centre in screen is displaced by delta along the
    // sky-normal direction (-sr, -cr):
    float delta = (float)(pitch - p) * PITCH_SCALE;
    int px = cx + (int)(delta * (-s_r));
    int py = cy + (int)(delta * (-c_r));

    if(py < yTop - 8 || py > yBottom + 8) continue;

    int len = (abs(p) % 30 == 0) ? 28 : 16;
    uint16_t lineColor = (abs(p) % 30 == 0) ? COL_MARK : COL_LADDER_DIM;

    // Tick direction along horizon tangent: (c_r, -s_r)
    // At zero roll (s_r=0, c_r=1): tick is horizontal (1, 0) ✓
    // At +30° right bank (s_r=-sin30=-0.5, c_r=cos30=0.87):
    //   tick = (0.87, +0.5) → right end is LOWER → horizon tilts right-down ✓ (pilot view)
    int x1 = px - (int)((len / 2) * c_r);
    int y1 = py + (int)((len / 2) * s_r);   // note: +s_r (was -s_r before)
    int x2 = px + (int)((len / 2) * c_r);
    int y2 = py - (int)((len / 2) * s_r);   // note: -s_r (was +s_r before)

    if(!inRoundedRect(px, py, xLeft, yTop, xRight - xLeft + 1, yBottom - yTop + 1, cornerRadius)){
      continue;
    }
    g.drawLine(x1, y1, x2, y2, lineColor);

    // Labels outside tick ends, clipped
    if(y1 >= yTop && y1 <= yBottom - 6 && x1 - 10 >= xLeft && x1 <= xRight){
      if(inRoundedRect(x1, y1, xLeft, yTop, xRight - xLeft + 1, yBottom - yTop + 1, cornerRadius)){
        g.setTextColor(lineColor); g.setCursor(x1 - 10, y1 + 4); g.print(abs(p));  // fixed Y
      }
    }
    if(y2 >= yTop && y2 <= yBottom - 6 && x2 + 2 >= xLeft && x2 + 12 <= xRight){
      if(inRoundedRect(x2, y2, xLeft, yTop, xRight - xLeft + 1, yBottom - yTop + 1, cornerRadius)){
        g.setTextColor(lineColor); g.setCursor(x2 + 2, y2 + 4); g.print(abs(p));  // fixed Y
      }
    }
  }
}

// Arc-style bank angle indicator drawn at the top of the horizon area.
// cx = horizontal centre of horizon panel, arcY = vertical centre of arc.
void drawRollScale(Adafruit_GFX &g, float roll, int cx, int arcY){
  const int R = 28;          // sized to sit cleanly at top of horizon
  
  // Draw arc from -60° to +60° in bank-angle space
  // Each bank angle a maps to screen angle (90+a)° from positive-x axis
  // so that 0° bank points straight up.
  const int arcAngles[] = {-60,-45,-30,-20,-10,0,10,20,30,45,60};
  const int nAng = 11;
  for(int i = 0; i < nAng; i++){
    int a = arcAngles[i];
    float screenAngle = (float)(90 + a) * DEG_TO_RAD; // 0° bank → top
    float cs = cos(screenAngle), sn = sin(screenAngle);
    int tickLen = (a == 0 || abs(a) == 60) ? 7 : (abs(a) == 30 ? 5 : 3);
    uint16_t tcol = (a == 0 || abs(a) == 30 || abs(a) == 60) ? 0xFFFF : COL_ACCENT;
    int ox = cx + (int)(R * cs);
    int oy = arcY - (int)(R * sn);  // minus because screen-y is inverted
    int ix = cx + (int)((R - tickLen) * cs);
    int iy = arcY - (int)((R - tickLen) * sn);
    g.drawLine(ix, iy, ox, oy, tcol);
  }

  // Moving triangle pointer — negated roll matches pilot-view horizon
  float screenAngle = (float)(90 - roll) * DEG_TO_RAD;
  float cs = cos(screenAngle), sn = sin(screenAngle);
  int tx  = cx + (int)((R - 1) * cs);
  int ty  = arcY - (int)((R - 1) * sn);
  int bx1 = cx + (int)((R - 6) * cos(screenAngle - 0.20f));
  int by1 = arcY - (int)((R - 6) * sin(screenAngle - 0.20f));
  int bx2 = cx + (int)((R - 6) * cos(screenAngle + 0.20f));
  int by2 = arcY - (int)((R - 6) * sin(screenAngle + 0.20f));
  g.fillTriangle(tx, ty, bx1, by1, bx2, by2, ST77XX_YELLOW);

  // Fixed centre-top reference mark (always points up) — small line
  g.drawFastVLine(cx, arcY - R - 2, 5, 0xFFFF);
}

void drawAircraftSymbol(Adafruit_GFX &g, int cx, int cy);
void drawWindIndicator(int cx, int cy, int r, float windSpd, float windDir, float heading);
void drawAltitudeTape(int x, int yTop, int yBottom, int cy);
void drawAirspeedTape(int x, int yTop, int yBottom, int cy);
void drawRollScale(Adafruit_GFX &g, float roll, int cx, int arcY);
void drawPitchLadder(Adafruit_GFX &g, float pitch, float roll, int cx, int cy, int xLeft, int xRight, int yTop, int yBottom);
void formatAltitudeCompact(float altitudeFeet, char *out, size_t outSize);
void telemetryTask(void *pvParameters);
void inputTask(void *pvParameters);
void displayTask(void *pvParameters);  // pinned to core 1

void clearContentArea(){
  tft.fillRect(0, 12, dispW, dispH - 12, ST77XX_BLACK);
}

void drawTopInfoBar(int xLimit){
  const char *tabs[3] = {"FLT", "NAV", "ADA"};
  tft.fillRect(0, 0, xLimit, 12, COL_PANEL_DARK);
  tft.drawFastHLine(0, 12, xLimit, COL_PANEL_EDGE);
  tft.setTextSize(1);
  tft.setTextColor(COL_ACCENT, COL_PANEL_DARK);
  tft.setCursor(2, 9);  // fixed: top-left tab label
  tft.print(tabs[activeTab]);

  if(activeTab == 0){
    tft.setTextColor(COL_ACCENT, COL_PANEL_DARK);
    tft.setCursor(xLimit - 38, 9);
    tft.print("G: ");
    tft.print(gLoad, 1);
  } else if(activeTab == 2){
    const char *apStr = apEnabled ? "AP ON" : "AP OFF";
    uint16_t apColor = apEnabled ? 0x07E0 : 0xF800;
    tft.setTextColor(apColor, COL_PANEL_DARK);
    tft.setCursor(xLimit - 38, 9);
    tft.print(apStr);
  }
}

// HSI compass rose — rotates so current heading is always at 12 o'clock.
void drawHeadingIndicator(int cx, int cy, int r, float hdg, int targetHdgVal){
  // ── Outer bezel ring (dark gray) ──────────────────────────────────────
  tft.drawCircle(cx, cy, r + 1, 0x4208);
  tft.drawCircle(cx, cy, r, 0x7BEF);
  
  // ── Rotating tick marks and labels ──────────────────────────────────
  // Formula: (d - hdg - 90)*DEG_TO_RAD puts degree `d` at correct screen angle
  // so that `hdg` is always at top (12 o'clock).
  for(int d = 0; d < 360; d += 5){
    float a   = (d - hdg - 90.0f) * DEG_TO_RAD;
    float ca  = cosf(a), sa = sinf(a);
    bool  maj = (d % 30 == 0);
    bool  med = (d % 10 == 0) && !maj;
    int   tlen = maj ? 8 : (med ? 4 : 2);
    uint16_t tcol = maj ? ST77XX_WHITE : 0x8410;

    int ox = cx + (int)(r * ca);
    int oy = cy + (int)(r * sa);
    int ix = cx + (int)((r - tlen) * ca);
    int iy = cy + (int)((r - tlen) * sa);
    tft.drawLine(ix, iy, ox, oy, tcol);

    // Labels at 30° intervals — Boeing style: "N 03 06 E 12 15 S 21 24 W 30 33"
    if(maj){
      bool isCard = (d % 90 == 0);
      int  lx = cx + (int)((r - 16) * ca);
      int  ly = cy + (int)((r - 16) * sa);

      tft.setTextSize(1);
      if(isCard){
        const char *card = (d==0)?"N":(d==90)?"E":(d==180)?"S":"W";
        uint16_t cc = (d==0) ? 0xFFC0 : 0x07FF;  // N = yellow, rest cyan
        tft.setTextColor(cc, ST77XX_BLACK);
        tft.setCursor(lx - (int)(strlen(card)*3), ly+3);
        tft.print(card);
      } else {
        char lbl[3];
        snprintf(lbl, sizeof(lbl), "%02d", d/10);
        tft.setTextColor(0xFFFF, ST77XX_BLACK);   // white for heading numbers
        tft.setCursor(lx - (int)(strlen(lbl)*3), ly+3);
        tft.print(lbl);
      }
    }
  }

  // ── Fixed lubber line at top (aircraft heading reference) ───────────
  tft.fillTriangle(cx-4, cy-r+9, cx+4, cy-r+9, cx, cy-r, 0xFFC0);  // Yellow reference
  tft.drawFastVLine(cx, cy-r-2, 3, 0xFFC0);

  // ── Heading bug (target) ────────────────────────────────────────────
  if(encMode == 0 && targetHdgVal >= 0){
    float ba = (targetHdgVal - hdg - 90.0f) * DEG_TO_RAD;
    int bx = cx + (int)(r * cosf(ba));
    int by = cy + (int)(r * sinf(ba));
    int bix = cx + (int)((r-7) * cosf(ba));
    int biy = cy + (int)((r-7) * sinf(ba));
    // Chevron pointing inward (lime green)
    float perp = ba + 1.5708f;
    int px = (int)(3.5f * cosf(perp)), py = (int)(3.5f * sinf(perp));
    tft.drawLine(bx+px, by+py, bix, biy, 0x07E0);
    tft.drawLine(bx-px, by-py, bix, biy, 0x07E0);
    tft.drawLine(bx+px, by+py, bx-px, by-py, 0x07E0);
  }

  // ── Centre: large heading number (bright cyan) ────────────────────────
  tft.fillRect(cx-18, cy-9, 36, 18, ST77XX_BLACK);
  tft.drawRect(cx-18, cy-9, 36, 18, 0x07FF);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  char hdgStr[4];
  snprintf(hdgStr, sizeof(hdgStr), "%03d", ((int)hdg + 360) % 360);
  tft.setCursor(cx - 18, cy+5);  // 3 chars × 12px / 2 = 18px
  tft.print(hdgStr);
}

void drawNavPage(int viewW, int viewH, bool fullRedraw){
  if(fullRedraw) clearContentArea();

  // ── Layout ───────────────────────────────────────────────────────────────
  // y=13..25  top strip: HDG centre | IAS left | ALT right
  // y=26..103 arc compass (semicircle, centre at bottom-centre of this zone)
  // y=104..127 bottom strip: wind left | VS centre | GS right

  // ── Top strip ────────────────────────────────────────────────────────────
  const int topY  = 13;
  const int topH  = 12;
  const int botY  = viewH - 24;  // bottom strip starts 24px from bottom
  const int arcY0 = topY + topH + 1;   // arc area top
  const int arcY1 = botY - 1;          // arc area bottom

  tft.fillRect(0, topY, viewW, topH, COL_PANEL_DARK);
  tft.drawFastHLine(0, topY + topH, viewW, COL_PANEL_EDGE);

  // HDG centred, IAS left, ALT right in top strip
  char hdgStr[10], iasStr[10], altStr[10];
  snprintf(hdgStr, sizeof(hdgStr), "%03d", ((int)hdg + 360) % 360);
  snprintf(iasStr, sizeof(iasStr), "%d", (int)ias);
  snprintf(altStr, sizeof(altStr), "%d", (int)alt);
  tft.setTextSize(1);
  tft.setTextColor(0x07FF, COL_PANEL_DARK);
  tft.setCursor(2, topY + 9);
  tft.print(iasStr);
  tft.setTextColor(0xFFFF, COL_PANEL_DARK);
  tft.setCursor((viewW - (int)(strlen(hdgStr)*6)) / 2, topY + 9);
  tft.print(hdgStr);
  tft.setTextColor(0x07FF, COL_PANEL_DARK);
  tft.setCursor(viewW - 2 - (int)(strlen(altStr)*6), topY + 9);
  tft.print(altStr);

  // ── Bottom strip ─────────────────────────────────────────────────────────
  tft.fillRect(0, botY, viewW, viewH - botY, COL_PANEL_DARK);
  tft.drawFastHLine(0, botY, viewW, COL_PANEL_EDGE);

  // ── Arc compass area ─────────────────────────────────────────────────────
  // Centre is at bottom-centre of arc zone, radius fills the zone height.
  // Semicircle opens UPWARD — cy is at arcY1 (bottom of arc zone).
  int cx = viewW / 2;
  int cy = arcY1;           // arc centre at bottom of arc zone
  int radius = arcY1 - arcY0;  // radius = height of arc zone
  if(radius < 20) radius = 20;

  // Clear arc area every frame (needed for rotating ticks — no flicker fix available)
  tft.fillRect(0, arcY0, viewW, arcY1 - arcY0 + 1, ST77XX_BLACK);

  // ── Helper: draw a semicircular arc (upper half only, clipped to arcY0..arcY1) ──
  // angle: 0=right, 90=up, 180=left in screen coords
  // We sweep deg from 0..180 mapping to screen angles where 0deg = right, 90deg = up, 180deg = left
  auto drawArc = [&](int r, uint16_t col, int step){
    int prevX = -1, prevY = -1;
    for(int deg = 0; deg <= 180; deg += step){
      // deg=0 → right endpoint, deg=90 → top, deg=180 → left endpoint
      float a = (float)deg * DEG_TO_RAD;
      int ox = cx + (int)roundf((float)r * cosf(a));
      int oy = cy - (int)roundf((float)r * sinf(a));
      if(oy < arcY0) oy = arcY0;
      // Skip out-of-bounds pixels to avoid vertical lines at screen edges
      if(ox < 0 || ox >= viewW) { prevX = -1; continue; }
      if(prevX >= 0 && abs(ox-prevX)<=2 && abs(oy-prevY)<=2)
        tft.drawLine(prevX, prevY, ox, oy, col);
      else
        tft.drawPixel(ox, oy, col);
      prevX = ox; prevY = oy;
    }
  };

  // ── 3 concentric arcs exactly like Airbus ND ─────────────────────────────
  // Outer  = full radius  (white)  — main compass arc with ticks
  // Middle = 2/3 radius   (dim)    — range ring
  // Inner  = 1/3 radius   (dimmer) — inner range ring
  drawArc(radius,         0x7BEF, 1);           // outer — white-ish
  drawArc(radius * 2 / 3, 0x4208, 2);           // middle range ring
  drawArc(radius * 1 / 3, 0x2945, 2);           // inner range ring

  // ── Heading ticks on OUTER arc ────────────────────────────────────────────
  for(int d = 0; d < 360; d += 5){
    float relDeg = (float)((d - (int)hdg + 540) % 360) - 180.0f;
    if(relDeg < -90.0f || relDeg > 90.0f) continue;
    float a = (90.0f - relDeg) * DEG_TO_RAD;
    float ca = cosf(a), sa = sinf(a);
    int ox = cx + (int)roundf((float)radius * ca);
    int oy = cy - (int)roundf((float)radius * sa);
    if(oy > cy || oy < arcY0 || ox < 1 || ox > viewW - 2) continue;

    bool maj = (d % 30 == 0);
    bool med = (d % 10 == 0) && !maj;
    int tlen = maj ? 7 : (med ? 4 : 2);
    uint16_t tcol = maj ? ST77XX_WHITE : 0x7BEF;

    int ix = cx + (int)roundf((float)(radius - tlen) * ca);
    int iy = cy - (int)roundf((float)(radius - tlen) * sa);
    tft.drawLine(ix, iy, ox, oy, tcol);

    if(maj){
      int lx = cx + (int)roundf((float)(radius - 13) * ca);
      int ly = cy - (int)roundf((float)(radius - 13) * sa);
      if(ly < arcY0 + 2) continue;
      tft.setTextSize(1);
      if(d % 90 == 0){
        const char *card = (d==0)?"N":(d==90)?"E":(d==180)?"S":"W";
        uint16_t cc = (d==0) ? 0xFFC0 : 0x07FF;
        tft.setTextColor(cc, ST77XX_BLACK);
        tft.setCursor(lx - (int)(strlen(card)*3), ly + 3);
        tft.print(card);
      } else {
        char lbl[3];
        snprintf(lbl, sizeof(lbl), "%02d", d / 10);
        tft.setTextColor(0xFFFF, ST77XX_BLACK);
        tft.setCursor(lx - (int)(strlen(lbl)*3), ly + 3);
        tft.print(lbl);
      }
    }
  }

  // ── Lubber line (fixed yellow tick at top of arc = current heading) ───────
  tft.fillTriangle(cx-3, arcY0+6, cx+3, arcY0+6, cx, arcY0, 0xFFC0);

  // ── Aircraft symbol fixed at arc centre ───────────────────────────────────
  tft.drawFastHLine(cx-7, cy-3, 14, 0xFFC0);   // wings
  tft.drawFastVLine(cx,   cy-6,  4, 0xFFC0);   // fuselage
  tft.drawFastHLine(cx-3, cy-1,  6, 0xFFC0);   // tail

  // ── Track line (magenta) — offset from heading by drift angle ───────────
  float trackOffset = (groundSpeed > 1.0f) ? (aoa * 0.5f) : 0.0f;
  // trackOffset is degrees right of heading. Use same mapping as ticks:
  float trackA = (90.0f - trackOffset) * DEG_TO_RAD;
  int trx = cx + (int)((radius - 4) * cosf(trackA));
  int trY = cy - (int)((radius - 4) * sinf(trackA));
  if(trY <= cy) tft.drawLine(cx, cy, trx, trY, 0xF81F);

  // ── Selected heading bug (green chevron on outer arc) ─────────────────────
  if(encMode == 0 || apEnabled){
    int bugHdg = targetHdg;
    float bugRel = (float)((bugHdg - (int)hdg + 540) % 360) - 180.0f;
    if(bugRel >= -90.0f && bugRel <= 90.0f) {
      float ba = (90.0f - bugRel) * DEG_TO_RAD;  // same mapping as ticks
      int bx  = cx + (int)(radius * cosf(ba));
      int by  = cy - (int)(radius * sinf(ba));
      int bix = cx + (int)((radius-6) * cosf(ba));
      int biy = cy - (int)((radius-6) * sinf(ba));
      float perp = ba + 1.5708f;
      int px = (int)(3.0f*cosf(perp)), py = (int)(3.0f*sinf(perp));
      if(by <= cy){  // only draw if in upper half
        tft.drawLine(bx+px, by+py, bix, biy, 0x07E0);
        tft.drawLine(bx-px, by-py, bix, biy, 0x07E0);
        tft.drawLine(bx+px, by+py, bx-px, by-py, 0x07E0);
      }
    }
  }

  // ── Bottom strip — left: wind | right: IAS / VS / AP target ─────────────
  // Wind: r=7, centre at x=18, y=botY+12
  drawWindIndicator(18, botY + 12, 7, windSpeedKt, windDirDeg, hdg);

  // Right column: x=90 gives 70px for text
  // Row 1 (botY+8):  IAS value cyan
  // Row 2 (botY+16): VS value cyan  
  // Row 3 (botY+24): AP target yellow (only when encoder active)
  tft.setTextSize(1);

  char vsLbl[10];
  snprintf(vsLbl,  sizeof(vsLbl),  "VS %+d", (int)vs);
  tft.setTextColor(0x07FF, COL_PANEL_DARK);
  tft.setCursor(90, botY + 16); tft.print(vsLbl);

  // AP target: show when encoder active, yellow with label
  if(encMode >= 0){
    char tgtVal[10];
    const char *tgtLbl;
    if     (encMode == 0){ tgtLbl="HDG"; snprintf(tgtVal,sizeof(tgtVal),">%03d",targetHdg); }
    else if(encMode == 1){ tgtLbl="ALT"; snprintf(tgtVal,sizeof(tgtVal),">%d",  targetAlt); }
    else                 { tgtLbl="IAS"; snprintf(tgtVal,sizeof(tgtVal),">%d",  targetIas); }
    char apLine[16]; snprintf(apLine, sizeof(apLine), "%s%s", tgtLbl, tgtVal);
    tft.setTextColor(0xFFC0, COL_PANEL_DARK);
    tft.setCursor(90, botY + 24); tft.print(apLine);
  }
}

void drawWindIndicator(int cx, int cy, int r, float windSpd, float windDir, float heading){
  tft.fillRect(cx - r - 2, cy - r - 2, (r + 2) * 2 + 6, r * 2 + 24, ST77XX_BLACK);

  tft.setTextSize(1);
  tft.setTextColor(0x7BEF, ST77XX_BLACK);
  tft.setCursor(5, 122);
  tft.print("WND");

  if(windSpd < 1.0f){
    tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
    tft.setCursor(cx - 9, cy + 4);  // fixed: "CALM" centred in circle
    tft.print("CALM");
    return;
  }

  // Arrow direction: wind comes FROM windDir (absolute).
  // Rotate by heading so the arrow is north-up relative: display angle = windDir - heading.
  // Screen: 0deg = up (-Y), so subtract 90deg from math angle.
  float relDeg  = windDir - heading;
  float arrowRad = (relDeg - 90.0f) * DEG_TO_RAD;
  float ca = cosf(arrowRad), sa = sinf(arrowRad);

  // Tail end (upwind) → tip end (downwind, i.e. where wind goes TO)
  // Arrow points FROM the wind source direction toward aircraft
  int tipX  = cx + (int)((r - 1) * ca);
  int tipY  = cy + (int)((r - 1) * sa);
  int tailX = cx - (int)((r - 1) * ca);
  int tailY = cy - (int)((r - 1) * sa);
  tft.drawLine(tailX, tailY, tipX, tipY, ST77XX_CYAN);

  // Arrowhead at tip
  float perpRad = arrowRad + 1.5708f;
  int ah1x = tipX - (int)(4 * ca) + (int)(3 * cosf(perpRad));
  int ah1y = tipY - (int)(4 * sa) + (int)(3 * sinf(perpRad));
  int ah2x = tipX - (int)(4 * ca) - (int)(3 * cosf(perpRad));
  int ah2y = tipY - (int)(4 * sa) - (int)(3 * sinf(perpRad));
  tft.fillTriangle(tipX, tipY, ah1x, ah1y, ah2x, ah2y, ST77XX_CYAN);

  // Small cross/barb at tail end
  tft.drawFastHLine(tailX - 2, tailY, 5, ST77XX_CYAN);

  // Subtle circle
  tft.drawCircle(cx, cy, r, 0x2945);

  // Two-line text below arrow for clarity — direction then speed
  // Line 1: heading in degrees e.g. "270" (right-aligned from cx)
  char dirTxt[6], spdTxt[8];
  snprintf(dirTxt, sizeof(dirTxt), "%03d", ((int)windDir + 360) % 360);
  snprintf(spdTxt, sizeof(spdTxt), "%dKT", (int)windSpd);
  tft.setTextSize(1);

  // Direction line
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(cx - (int)(strlen(dirTxt) * 3), cy + r + 9);
  tft.print(dirTxt);

  // Speed line (cyan, one row below)
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(cx - (int)(strlen(spdTxt) * 3), cy + r + 17);
  tft.print(spdTxt);
}

void drawFltPage(int viewW, int viewH, bool fullRedraw){
  // Clear on full redraw to avoid stale pixels after tab/button actions.
  if(fullRedraw){
    clearContentArea();
  }

  const int contentTop    = 12;
  const int contentBottom = viewH - 1;
  const int tapeTop       = contentTop;
  const int centerMargin  = 22;
  const int centerYOffset = -10;  // negative moves the horizon up
  int tapeBottom          = contentBottom - centerMargin - 2;  // Adjusted after bottom section

  const int centerX  = TAPE_W;
  const int centerW  = viewW - (TAPE_W * 2);
  
  // Center box with margins on top/bottom
  const int centerYBase = contentTop + centerMargin;
  const int centerY  = centerYBase + centerYOffset;
  const int centerH  = contentBottom - contentTop - (centerMargin * 2);
  const int centerCx = centerX + centerW / 2;
  const int centerCy = centerY + centerH / 2;
  const int centerCyBase = centerYBase + centerH / 2;
  const int horizonRadius = 8;   // Must match cornerRadius inside drawHorizonFast

  const int arcY = centerY + 40;  // roll scale arc centre (38px radius + 2px margin)

  // 1. Horizon — clips its own corners internally; no pre-clear or post-clip needed
  drawHorizonFast(pitch, roll, centerX, centerY, centerW, centerH);

  // 2. Pitch ladder (drawn on top of horizon, clipped to centre strip)
  drawPitchLadder(tft, pitch, roll, centerCx, centerCy,
                  centerX + 2, centerX + centerW - 2,
                  centerY + 2, centerY + centerH - 2);

  // 3. Roll arc indicator
  drawRollScale(tft, roll, centerCx, arcY);

  // 4. Fixed aircraft symbol
  drawAircraftSymbol(tft, centerCx, centerCy);

  // 5. Cockpit-style box border for center
  tft.drawRoundRect(centerX - 1, centerY - 1, centerW + 2, centerH + 2, horizonRadius, 0x3186);
  tft.drawFastHLine(centerX, centerY + 1, centerW, 0x2104);
  tft.drawRoundRect(centerX, centerY, centerW, centerH, horizonRadius - 2, COL_PANEL_EDGE);

  int topSectionH = (centerY - 12) - 2;
  if(topSectionH > 0 && fullRedraw){
    tft.fillRect(centerX, 12, centerW, topSectionH, COL_PANEL_DARK);
  }

  // 7. Bottom section — Compass scale with Track Diamond (full width)
  int botSectionY = centerYBase + centerH + 1;
  int botSectionH = centerMargin - 2;
  tft.fillRect(0, botSectionY, viewW, botSectionH, COL_PANEL_DARK);
  tft.drawRect(0, botSectionY, viewW, botSectionH, COL_PANEL_EDGE);

  // Ensure tapes stop right above the bottom section (no gap artifacts)
  tapeBottom = botSectionY - 1;
  
  // Heading/Track scale (horizontal, full width)
  int compassCx = viewW / 2;
  int compassCy = botSectionY + botSectionH / 2;
  tft.setTextSize(1);
  
  // Draw heading degree markings across full width
  for(int d = -80; d <= 80; d += 5){
    int hdgVal = ((int)hdg + d + 360) % 360;
    int tickX = compassCx + d * 2;  // 2 pixels per degree
    if(tickX < 2 || tickX > viewW - 2) continue;
    
    bool isMajor = (hdgVal % 30 == 0);
    int tickLen = isMajor ? 7 : (hdgVal % 10 == 0 ? 5 : 2);
    uint16_t tickCol = isMajor ? 0xFFFF : COL_MARK;
    tft.drawFastVLine(tickX, compassCy - tickLen / 2 - 1, tickLen, tickCol);
    
    if(isMajor){
      char hdgLbl[3];
      snprintf(hdgLbl, sizeof(hdgLbl), "%02d", hdgVal / 10);
      tft.setTextColor(0xFFFF, COL_PANEL_DARK);
      tft.setCursor(tickX - 6, compassCy + 10);  // 2 chars × 6px / 2 = 6px
      tft.print(hdgLbl);
    }
  }
  
  // Track Diamond (green) — actual ground track
  // Simple calculation: track angle offset from heading based on wind
  float trackOffset = (groundSpeed > 1.0f) ? (aoa * 0.5f) : 0;  // Approximation
  int trackDiamondX = compassCx + (int)(trackOffset * 2.0f);
  
  if(trackDiamondX >= 4 && trackDiamondX <= viewW - 4){
    int diamondSide = 5;
    tft.drawLine(trackDiamondX, compassCy - diamondSide, trackDiamondX + diamondSide, compassCy, 0x07E0);  // Green
    tft.drawLine(trackDiamondX + diamondSide, compassCy, trackDiamondX, compassCy + diamondSide, 0x07E0);
    tft.drawLine(trackDiamondX, compassCy + diamondSide, trackDiamondX - diamondSide, compassCy, 0x07E0);
    tft.drawLine(trackDiamondX - diamondSide, compassCy, trackDiamondX, compassCy - diamondSide, 0x07E0);
  }
  
  // Center heading bug (yellow lubber line pointing up)
  tft.fillTriangle(compassCx - 2, compassCy - 5, compassCx + 2, compassCy - 5, compassCx, compassCy - 2, 0xFFC0);

  // 6. Side tapes (each draws its own background, no pre-clear needed)
  drawAirspeedTape(0,               tapeTop, tapeBottom, centerCyBase);
  drawAltitudeTape(viewW - TAPE_W,  tapeTop, tapeBottom, centerCyBase);
}

void formatThrustCompact(float value, char *out, size_t outSize){
  if(fabs(value) >= 1000.0f){
    snprintf(out, outSize, "%.1fk", value / 1000.0f);
  } else {
    snprintf(out, outSize, "%d", (int)value);
  }
}



void drawAdaPage(int viewW, int viewH, bool fullRedraw){
  if(fullRedraw) clearContentArea();

  const bool stallWarn    = isStalling;
  const bool ovspd        = isOverspeed;
  const bool engOff       = !enginesOn;
  const bool prkBrk       = parkingBrake;
  const bool bankWarn     = fabsf(roll) > 60.0f;
  const bool aoaWarn      = !groundContact && aoa > 10.0f;
  const bool highGWarn    = gLoad > 7.0f;
  const bool negGWarn     = !groundContact && gLoad < 0.2f;
  const bool sinkWarn     = !groundContact && vs < -1000.0f;
  const bool lowAltWarn   = !groundContact && haglFt > 0 && haglFt < 500.0f && vs < -200.0f;
  const bool splDeployed  = spoilersPosition > 0.75f;
  const bool splArmed     = spoilersPosition > 0.25f && spoilersPosition < 0.75f;
  const bool splWarn      = splDeployed || splArmed;
  const bool splRed       = splDeployed;
  const bool anyWarn      = stallWarn||ovspd||engOff||prkBrk||bankWarn||aoaWarn||
                            highGWarn||negGWarn||sinkWarn||lowAltWarn||(gpwsAlarm[0]!=0);

  const uint16_t AMBER = 0xFD20;
  const uint16_t DIM   = 0x2945;
  const int contentBottom = viewH - 1;

  // ── Landscape layout: LEFT column = warnings, RIGHT column = systems ──
  // Split screen vertically: left half = warn panel, right half = sys table
  const int SPLIT = viewW / 2;  // x=80 dividing line

  // ── GPWS lookup ───────────────────────────────────────────────────────
  struct { const char *id; const char *label; uint16_t col; bool critical; } gpwsMap[] = {
    {"whoopwhooppullup",              "PULL UP",      0xF800, true},
    {"pullup",                        "PULL UP",      0xF800, true},
    {"terrainterrainwhoopwhooppullup","TERR PULL UP", 0xF800, true},
    {"terrainterrainpullup",          "TERR PULL UP", 0xF800, true},
    {"terrainterrain",                "TERRAIN",      0xF800, true},
    {"sinkrate",                      "SINK RATE",    0xF800, true},
    {"dontsink",                      "DON'T SINK",   0xFD20, false},
    {"toolowgear",                    "TOO LOW GEAR", 0xFD20, false},
    {"toolowterrain",                 "TOO LOW TERR", 0xFD20, false},
    {"toolowflaps",                   "TOO LOW FLAP", 0xFD20, false},
    {"glideslopeloud",                "GLIDE SLOPE",  0xF800, true},
    {"glideslope",                    "GLIDE SLOPE",  0xFD20, false},
    {"bankAngle",                     "BANK ANGLE",   0xFD20, false},
    {"apdisconnect",                  "AP DISC",      0xFD20, false},
  };
  const char *gpwsLabel = nullptr;
  uint16_t gpwsCol = 0xFFFF;
  bool gpwsCritical = false;
  for(auto &g : gpwsMap){
    if(strcmp((const char*)gpwsAlarm, g.id)==0){
      gpwsLabel=g.label; gpwsCol=g.col; gpwsCritical=g.critical; break;
    }
  }
  bool gpwsActive = (gpwsLabel != nullptr);

  // ── LEFT: Banner + status lamps ──────────────────────────────────────
  const int bannerY = 13;
  const int bannerH = 12;
  uint16_t bannerBg = (gpwsActive && gpwsCritical) ? 0xF800 :
                       gpwsActive                   ? AMBER  :
                       stallWarn                     ? 0xF800 :
                       anyWarn                       ? AMBER  : 0x0229;
  uint16_t bannerFg = (anyWarn && !stallWarn && !gpwsActive) ? ST77XX_BLACK : ST77XX_WHITE;

  tft.fillRect(0, bannerY, SPLIT - 1, bannerH, bannerBg);
  tft.drawRect(0, bannerY, SPLIT - 1, bannerH, anyWarn ? ST77XX_WHITE : 0x4208);
  tft.setTextSize(1);

  const char *bannerTxt = gpwsActive ? gpwsLabel :
                          stallWarn  ? "!! STALL !!" :
                          anyWarn    ? "MASTER CAUTION" : "NORMAL";
  tft.setTextColor(bannerFg, bannerBg);
  // centre in left half
  int bTxtX = (SPLIT - 1 - (int)(strlen(bannerTxt) * 6)) / 2;
  if(bTxtX < 1) bTxtX = 1;
  tft.setCursor(bTxtX, bannerY + 9);
  tft.print(bannerTxt);

  // Status lamps — 2 columns of 5, each row 10px tall
  struct StatusLine { const char *txt; bool on; uint16_t col; };
  StatusLine leftLamps[] = {
    {"STALL",  stallWarn,  0xF800},
    {"OVSPD",  ovspd,      0xF800},
    {"BANK",   bankWarn,   AMBER},
    {"AOA",    aoaWarn,    AMBER},
    {"HIGH G", highGWarn,  AMBER},
  };
  StatusLine rightLamps[] = {
    {"ENG OFF", engOff,    0xF800},
    {"PRK BRK", prkBrk,   AMBER},
    {"SINK",    sinkWarn,  AMBER},
    {"LALT",    lowAltWarn,AMBER},
    {"SPLR",    splWarn,   (uint16_t)(splRed ? 0xF800 : AMBER)},
  };

  const int lampY0  = bannerY + bannerH + 2;
  const int lampRowH = 10;
  const int colW = (SPLIT - 2) / 2;

  tft.setTextSize(1);
  for(int i = 0; i < 5; i++){
    int y = lampY0 + i * lampRowH;
    if(y + lampRowH > contentBottom) break;
    tft.setTextColor(leftLamps[i].on  ? leftLamps[i].col  : DIM, ST77XX_BLACK);
    tft.setCursor(2, y + 8);
    tft.print(leftLamps[i].txt);
    tft.setTextColor(rightLamps[i].on ? rightLamps[i].col : DIM, ST77XX_BLACK);
    tft.setCursor(2 + colW, y + 8);
    tft.print(rightLamps[i].txt);
  }

  // Vertical divider
  tft.drawFastVLine(SPLIT, 13, viewH - 13, 0x3186);

  // ── RIGHT: Systems table ──────────────────────────────────────────────
  // In landscape (160x128): SPLIT=80, right half=80px wide.
  // 6 rows in (viewH-13)=115px → ~19px per row, fits with 1px gaps.
  const int NUM_ROWS = 6;
  const int SX    = SPLIT + 1;
  const int SW    = viewW - SX - 1;
  const int ROW_H = (viewH - 14) / NUM_ROWS;
  const int LBL_W = 26;  // label column width (fits 4 chars at textSize=1)

  char bufRpmL[8], bufRpmR[8], bufGear[8], bufFlaps[8], bufSpoilers[8], bufThrPct[8];
  snprintf(bufRpmL,   sizeof(bufRpmL),   "%d",   (int)engineRpmLeft);
  snprintf(bufRpmR,   sizeof(bufRpmR),   "%d",   (int)engineRpmRight);
  snprintf(bufThrPct, sizeof(bufThrPct), "%d%%", (int)(potThrottle * 100.0f));
  snprintf(bufGear,   sizeof(bufGear),
    gearPosition < 0.05f ? "UP" : gearPosition > 0.95f ? "DOWN" : "MOVE");
  int flapsStep = (int)flapsPosition;
  int flapsMax  = (int)roundf((flapsPosition - flapsStep) * 100.0f);
  snprintf(bufFlaps, sizeof(bufFlaps), "%d/%d", flapsStep, flapsMax);
  snprintf(bufSpoilers, sizeof(bufSpoilers),
    spoilersPosition < 0.25f ? "RET" :
    spoilersPosition < 0.75f ? "ARM" : "DEP");

  struct SysRow { const char *lbl; const char *val; uint16_t valCol; };
  SysRow rows[6] = {
    {"LRPM", bufRpmL,   (uint16_t)(engineRpmLeft  > 3000 ? 0x07E0 : 0xFD20)},
    {"RRPM", bufRpmR,   (uint16_t)(engineRpmRight > 3000 ? 0x07E0 : 0xFD20)},
    {"THR",  bufThrPct, (uint16_t)(potThrottle > 0.9f ? 0x07E0 : potThrottle > 0.5f ? 0xFD20 : 0x07FF)},
    {"GEAR", bufGear,   (uint16_t)(gearPosition < 0.05f ? 0xF800 : gearPosition > 0.95f ? 0x07E0 : 0xFD20)},
    {"FLAP", bufFlaps,  (uint16_t)(flapsPosition > 0.05f ? 0xFD20 : 0x8410)},
    {"SPLR", bufSpoilers,(uint16_t)(spoilersPosition > 0.75f ? 0xF800 : spoilersPosition > 0.25f ? 0xFD20 : 0x8410)},
  };

  tft.setTextSize(1);
  for(int i = 0; i < NUM_ROWS; i++){
    int ry = 13 + i * ROW_H;
    if(ry + ROW_H > contentBottom) break;
    tft.fillRect(SX, ry, SW, ROW_H, 0x0861);
    tft.fillRect(SX, ry, LBL_W, ROW_H, 0x10A2);
    tft.drawRect(SX, ry, SW, ROW_H, 0x3186);
    tft.drawFastVLine(SX + LBL_W, ry + 1, ROW_H - 2, 0x3186);
    tft.setTextColor(0x07FF, 0x10A2);
    tft.setCursor(SX + 2, ry + ROW_H - 3);
    tft.print(rows[i].lbl);
    tft.setTextColor(rows[i].valCol, 0x0861);
    int vx = SX + LBL_W + 4;
    tft.setCursor(vx, ry + ROW_H - 3);
    tft.print(rows[i].val);
  }
}

void formatAltitudeCompact(float altitudeFeet, char *out, size_t outSize){
  if(altitudeFeet < 1000.0f){
    snprintf(out, outSize, "%3d", (int)altitudeFeet % 1000);
  } else {
    float kValue = altitudeFeet / 1000.0f;
    if(kValue < 10.0f){
      snprintf(out, outSize, "%.1fk", kValue);
    } else {
      snprintf(out, outSize, "%.0fk", kValue);
    }
  }
}

void drawAirspeedTape(int x, int yTop, int yBottom, int cy){
  tft.fillRect(x, yTop, TAPE_W, yBottom - yTop + 1, 0x8410);
  tft.drawRect(x, yTop, TAPE_W, yBottom - yTop + 1, COL_PANEL_EDGE);
  tft.fillRect(x + 1, yTop + 1, TAPE_W - 2, 8, COL_PANEL);
  tft.setTextColor(COL_ACCENT, COL_PANEL);

  bool machMode = (sp >= 0.6f);

  if(machMode){
    // ── Mach mode (above M0.60) ──────────────────────────────────────────
    tft.setCursor(x + 1, yTop + 9);
    tft.print("MCH");
    // Ticks every 0.01M, labels every 0.05M e.g. "0.75", "0.80", "0.85"
    for(float v = sp - 0.15f; v <= sp + 0.15f; v += 0.01f){
      if(v < 0) continue;
      int y = cy + (int)((sp - v) * 700.0f);
      if(y < yTop + 10 || y > yBottom - 2) continue;
      int vi = (int)roundf(v * 100);
      bool major = (vi % 5 == 0);
      tft.drawFastHLine(x + TAPE_W - (major?9:5), y, major?7:3, COL_MARK);
      if(major){
        char ms[8];
        snprintf(ms, sizeof(ms), "%.2f", v);
        int tx = x + TAPE_W - 3 - (int)(strlen(ms)*6);
        if(tx < x+1) tx = x+1;
        tft.setTextColor(COL_MARK, 0x8410);
        tft.setCursor(tx, y+4);
        tft.print(ms);
      }
    }
    tft.fillRect(x + 1, cy - 6, TAPE_W - 2, 12, ST77XX_BLACK);
    tft.drawRect(x + 1, cy - 6, TAPE_W - 2, 12, COL_ACCENT);
    tft.fillTriangle(x + TAPE_W - 1, cy, x + TAPE_W + 4, cy - 3, x + TAPE_W + 4, cy + 3, COL_ACCENT);
    char mStr[8];
    snprintf(mStr, sizeof(mStr), "%.2f", sp);
    tft.setTextColor(COL_ACCENT, ST77XX_BLACK);
    tft.setCursor(x + 1, cy + 4);
    tft.print(mStr);
  } else {
    // ── IAS mode ─────────────────────────────────────────────────────────
    tft.setCursor(x + 3, yTop + 9);
    tft.print("SPD");
    int base = ((int)ias / 10) * 10;
    for(int v = base - 50; v <= base + 50; v += 10){
      int y = cy + (int)((ias - v) * 1.1f);
      if(y < yTop + 10 || y > yBottom - 2) continue;
      bool major = (v % 20 == 0);
      tft.drawFastHLine(x + TAPE_W - (major?9:6), y, major?7:4, COL_MARK);
      if(major){
        tft.setTextColor(COL_MARK, 0x8410);
        char vs2[5]; snprintf(vs2, sizeof(vs2), "%d", v);
        int tx = x + TAPE_W - 11 - (int)(strlen(vs2)*6);
        if(tx < x+2) tx = x+2;
        tft.setCursor(tx, y+4);
        tft.print(vs2);
      }
    }
    tft.fillRect(x+1, cy-6, TAPE_W-2, 12, ST77XX_BLACK);
    tft.drawRect(x+1, cy-6, TAPE_W-2, 12, COL_ACCENT);
    tft.fillTriangle(x+TAPE_W-1, cy, x+TAPE_W+4, cy-3, x+TAPE_W+4, cy+3, COL_ACCENT);
    tft.setTextColor(COL_ACCENT, ST77XX_BLACK);
    char iasStr[6]; snprintf(iasStr, sizeof(iasStr), "%3d", (int)ias);
    tft.setCursor(x+3, cy+4);
    tft.print(iasStr);
  }
}

void drawAltitudeTape(int x, int yTop, int yBottom, int cy){
  tft.fillRect(x, yTop, TAPE_W, yBottom - yTop + 1, 0x8410);  // Grey background
  tft.drawRect(x, yTop, TAPE_W, yBottom - yTop + 1, COL_PANEL_EDGE);
  tft.fillRect(x + 1, yTop + 1, TAPE_W - 2, 8, COL_PANEL);
  tft.setTextColor(COL_ACCENT, COL_PANEL);
  tft.setCursor(x + 3, yTop + 9);  // fixed Y: tape header label
  tft.print("ALT");

  // Scale tuned for compact tape width and readable spacing
  const float ALT_PX_PER_FT = 0.06f;
  int base = ((int)alt / 100) * 100;

  tft.setTextSize(1);
  for(int v = base - 700; v <= base + 700; v += 100){
    if(v < 0) continue;
    int y = cy + (int)((alt - v) * ALT_PX_PER_FT);
    if(y < yTop + 10 || y > yBottom - 2) continue;

    bool major = (v % 200 == 0);
    tft.drawFastHLine(x, y, major ? 7 : 4, COL_MARK);
    if(major){
      char lbl[8];
      if(v >= 10000) snprintf(lbl, sizeof(lbl), "%dK", v / 1000);
      else if(v % 1000 == 0) snprintf(lbl, sizeof(lbl), "%dk", v / 1000);
      else snprintf(lbl, sizeof(lbl), "%d", v / 100);
      tft.setTextColor(COL_MARK, 0x8410);  // Grey background
      int lx = x + TAPE_W - 2 - (int)(strlen(lbl)*6);
      if(lx < x + 7) lx = x + 7;
      tft.setCursor(lx, y + 4);  // fixed Y: ALT tick label centred on tick
      tft.print(lbl);
    }
  }

  // Current altitude window with pointer into attitude area
  tft.fillRect(x + 1, cy - 6, TAPE_W - 2, 12, ST77XX_BLACK);
  tft.drawRect(x + 1, cy - 6, TAPE_W - 2, 12, COL_ACCENT);
  tft.fillTriangle(x, cy, x - 4, cy - 3, x - 4, cy + 3, COL_ACCENT);
  tft.setTextColor(COL_ACCENT, ST77XX_BLACK);
  char altText[8];
  formatAltitudeCompact(alt, altText, sizeof(altText));
  int tx = x + TAPE_W - 3 - (int)(strlen(altText)*6);
  if(tx < x + 2) tx = x + 2;
  tft.setCursor(tx, cy + 4);
  tft.print(altText);

  // RA (Radio Altitude) readout below tape — shown below 2500ft AGL, like real PFD
  if(haglFt > 0 && haglFt < 2500.0f && !groundContact){
    char raStr[8];
    snprintf(raStr, sizeof(raStr), "RA%d", (int)haglFt);
    // Small cyan label just below the tape bottom
    tft.fillRect(x, yBottom + 1, TAPE_W, 9, ST77XX_BLACK);
    tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
    int rax = x + TAPE_W - 2 - (int)(strlen(raStr)*6);
    if(rax < x) rax = x;
    tft.setCursor(rax, yBottom + 2);
    tft.print(raStr);
  }
}

void updateLedWarning(){
  uint32_t now = now_ms();
  
  // Check if telemetry is disconnected (no data for 500ms)
  bool isConnected = (now - lastTelemetryMs) < TELEMETRY_TIMEOUT_MS;
  
  if(!isConnected){
    // Disconnected: LED off
    digitalWrite(LED_PIN, LOW);
    return;
  }

  // Warning flags (aligned with ADA logic)
  bool stallWarn  = isStalling;
  bool ovspd      = isOverspeed;
  bool engOff     = !enginesOn;
  bool prkBrk     = parkingBrake;
  bool bankWarn   = fabsf(roll) > 45.0f;
  bool aoaWarn    = aoa > 10.0f;
  bool highGWarn  = gLoad > 7.0f;
  bool negGWarn   = gLoad < 0.2f;
  bool sinkWarn   = vs < -1000.0f;
  bool lowAltWarn = !groundContact && haglFt > 0 && haglFt < 500.0f && vs < -200.0f;
  bool splWarn    = spoilersPosition > 0.1f;

  bool gpwsActive = (gpwsAlarm[0] != 0);
  bool gpwsCritical = false;
  if(gpwsActive){
    if(strcmp((const char*)gpwsAlarm, "whoopwhooppullup") == 0 ||
       strcmp((const char*)gpwsAlarm, "pullup") == 0 ||
       strcmp((const char*)gpwsAlarm, "terrainterrainwhoopwhooppullup") == 0 ||
       strcmp((const char*)gpwsAlarm, "terrainterrainpullup") == 0 ||
       strcmp((const char*)gpwsAlarm, "terrainterrain") == 0 ||
       strcmp((const char*)gpwsAlarm, "sinkrate") == 0 ||
       strcmp((const char*)gpwsAlarm, "glideslopeloud") == 0){
      gpwsCritical = true;
    }
  }

  // Severity modes: 0=steady, 1=slow, 2=medium, 3=fast
  int mode = 0;
  if(stallWarn || gpwsCritical){
    mode = 3;
  } else if(ovspd || sinkWarn || lowAltWarn){
    mode = 2;
  } else if(bankWarn || aoaWarn || highGWarn || negGWarn || gpwsActive){
    mode = 1;
  }

  static uint32_t modeStartMs = 0;
  static int lastMode = -1;
  if(mode != lastMode){
    lastMode = mode;
    modeStartMs = now;
  }

  if(mode == 0){
    digitalWrite(LED_PIN, HIGH);
    return;
  }

  // Critical warnings: mostly ON with a short OFF pulse (visible rapid blink).
  uint32_t periodMs = (mode == 3) ? 200 : (mode == 2) ? 300 : 500;
  uint32_t onMs     = (mode == 3) ? 150 : (mode == 2) ? 150 : 250;
  uint32_t phase = (now - modeStartMs) % periodMs;
  digitalWrite(LED_PIN, phase < onMs ? HIGH : LOW);
}

void drawAircraftSymbol(Adafruit_GFX &g, int cx, int cy){
  g.drawFastHLine(cx-14, cy, 10, ST77XX_YELLOW);
  g.drawFastHLine(cx+4, cy, 10, ST77XX_YELLOW);
  g.drawFastVLine(cx, cy-3, 7, ST77XX_YELLOW);
}

void setup(){
  Serial.begin(115200);
  pinMode(BTN_PIN, INPUT_PULLUP);  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  analogReadResolution(12);  // Pico: 12-bit ADC (0-4095)
  pinMode(POT_PIN, INPUT);

  gpio_init(ENC_A); gpio_set_dir(ENC_A, GPIO_IN); gpio_pull_up(ENC_A);
  gpio_init(ENC_B); gpio_set_dir(ENC_B, GPIO_IN); gpio_pull_up(ENC_B);
  pinMode(ENC_BTN, INPUT_PULLUP);
  encLastAB = (gpio_get(ENC_A) << 1) | gpio_get(ENC_B);
  add_repeating_timer_us(-500, encTimerCb, nullptr, &encTimer);

  SPI1.setSCK(TFT_SCK);   // GP26 = SPI1 SCK
  SPI1.setTX(TFT_MOSI);   // GP27 = SPI1 MOSI
  SPI1.begin();
  
  tft.initR(INITR_BLACKTAB);
  tft.fillScreen(ST77XX_BLACK);
  tft.setRotation(1); 
  
  dispW = tft.width();
  dispH = tft.height();
  forceRedraw = true;
  tft.setFont(&B612_Regular5pt7b);
  tft.setTextSize(2);
  
  uint16_t colors[] = {ST77XX_RED, ST77XX_GREEN, ST77XX_CYAN, ST77XX_YELLOW};
  for(int i = 0; i < 4; i++){
    tft.setTextColor(colors[i]);
    tft.setCursor(dispW / 2 - 45, dispH / 2 - 15);
    tft.println("GeoPanel");
    delay(200);
    tft.fillScreen(ST77XX_BLACK);
    delay(100);
  }
  
  tft.fillScreen(ST77XX_BLACK);

  // SMP FreeRTOS on RP2040: pin each task to its core.
  // Core 0 (0x01): telemetry + input  — light tasks, fast response
  // Core 1 (0x02): display            — heavy SPI, isolated so it never
  //                                     blocks input polling
  TaskHandle_t h;
  xTaskCreate(displayTask,   "display",   8192, NULL, 1, &h);
  vTaskCoreAffinitySet(h, (1 << 1));  // core 1 only

  xTaskCreate(telemetryTask, "telemetry", 4096, NULL, 3, &h);
  vTaskCoreAffinitySet(h, (1 << 0));  // core 0 only

  xTaskCreate(inputTask,     "input",     4096, NULL, 4, &h);
  vTaskCoreAffinitySet(h, (1 << 0));  // core 0 only
}

void telemetryTask(void *pvParameters){
  (void)pvParameters;
  for(;;){
    while(Serial.available()){
      char c = Serial.read();
      if(c == '\n'){
        if(idx > 0){
          buf[idx] = 0;
          if(strncmp(buf, "ACK,AP,", 7) == 0){
            apEnabled = (buf[7] == 'O' && buf[8] == 'N');
            apAckLockUntilMs = now_ms() + 300;  // suppress telemetry overwrite for 300ms
            forceRedraw = true;
          } else {
            parseLine(buf);
          }
        }
        idx = 0;
      } else if(idx < 319){
        buf[idx++] = c;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void inputTask(void *pvParameters)
{
  (void)pvParameters;
  for (;;)
  {
    uint32_t now = now_ms();

    {
      uint32_t saved = save_and_disable_interrupts();
      int32_t acc = encAccum;
      encAccum = 0;
      restore_interrupts(saved);

      static int32_t rem = 0;
      rem += acc;
      int detents = rem / 2;
      rem %= 4;

      if (detents != 0 && encMode >= 0)
      {
        int dir = (detents > 0) ? 1 : -1;
        int steps = min(abs(detents), 5);
        if (encMode == 0)
        {
          int prev = targetHdg;
          targetHdg = (targetHdg + dir * steps + 360) % 360;
          if (apEnabled && targetHdg != prev)
          {
            Serial.print("SET,HDG,");
            Serial.println(targetHdg);
          }
        }
        else if (encMode == 1)
        {
          int prev = targetAlt;
          targetAlt += dir * steps * 50;
          if (targetAlt < 0)
            targetAlt = 0;
          if (apEnabled && targetAlt != prev)
          {
            Serial.print("SET,ALT,");
            Serial.println(targetAlt);
          }
        }
        else if (encMode == 2)
        {
          int prev = targetIas;
          targetIas += dir * steps * 5;
          if (targetIas < 0)
            targetIas = 0;
          if (apEnabled && targetIas != prev)
          {
            Serial.print("SET,IAS,");
            Serial.println(targetIas);
          }
        }
        forceRedraw = true;
      }
    }

  if (checkButtonPress(ENC_BTN))
  {
    int prevMode = encMode;
    encMode = (encMode < 2) ? encMode + 1 : -1;
    if (encMode == 0 && prevMode != 0)
      targetHdg = (int)hdg;
    if (encMode == 1 && prevMode != 1)
      targetAlt = (int)alt;
    if (encMode == 2 && prevMode != 2)
      targetIas = (int)ias;
    Serial.print("MODE:");
    Serial.println(encMode);
    forceRedraw = true;
    clearOnRedraw = true;
  }

  // top button
  if (checkButtonPress(BTN_PIN))
  {
    activeTab = (activeTab + 1) % 3;
    forceRedraw = true;
    clearOnRedraw = true;
  }

  // ── Potentiometer throttle ────────────────────────────────────────────
  // Read 12-bit ADC (0-4095), map to 0.0-1.0, send SET,THR only on change
  static uint32_t lastPotSendMs = 0;
  if (now - lastPotSendMs >= 50)
  { // check every 50ms
    lastPotSendMs = now;
    int raw = analogRead(POT_PIN); // 0-4095 on Pico (12-bit)
    // Deadband: only update if changed by more than ~1% (40 counts)
    if (abs(raw - lastPotRaw) > 40 || lastPotRaw < 0)
    {
      lastPotRaw = raw;
      float throttleVal = raw / 4095.0f; // 0.0-1.0
      potThrottle = throttleVal;
      // Send to GeoFS: two decimal places is plenty
      char thrMsg[20];
      snprintf(thrMsg, sizeof(thrMsg), "SET,THR,%.2f", throttleVal);
      Serial.println(thrMsg);
    }
  }

  vTaskDelay(pdMS_TO_TICKS(2));
  }  // for(;;)
}

// ── displayTask: add a 30ms frame gate so it doesn't spin at 1ms ──
void displayTask(void *pvParameters){
  // Pinned to core 1 by vTaskCoreAffinitySet in setup().
  // Core 0 tasks write volatile globals; we just read and render.
  (void)pvParameters;
  static uint8_t  prevTab    = 255;
  static uint32_t lastDrawMs = 0;
  for(;;){
    updateLedWarning();

    uint32_t now    = now_ms();
    bool tabChanged = (activeTab != prevTab);
    bool fr         = forceRedraw;
    bool telem      = telemetryUpdated;

    if(!tabChanged && !fr && !telem && (now - lastDrawMs < 200)){
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    if(fr)    forceRedraw      = false;
    if(telem) telemetryUpdated = false;
    lastDrawMs = now;

    bool fullRedraw = tabChanged || clearOnRedraw || fr;
    if(fullRedraw) clearOnRedraw = false;
    prevTab = activeTab;

    if(activeTab == 0)      drawFltPage(dispW, dispH, fullRedraw);
    else if(activeTab == 1) drawNavPage(dispW, dispH, fullRedraw);
    else                    drawAdaPage(dispW, dispH, fullRedraw);
    drawTopInfoBar(dispW);
  }
}
void loop(){
  vTaskDelay(pdMS_TO_TICKS(1000));
}