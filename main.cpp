#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_NeoPixel.h>

#define TFT_CS  26
#define TFT_DC  22
#define TFT_RST 21
#define BTN_PIN  28
#define AP_BTN_PIN 25 // for autopilot
#define LED_PIN  23
#define NEOPIXEL_PIN 24

Adafruit_ST7735 tft(&SPI, TFT_CS, TFT_DC, TFT_RST);

int dispW = 0;
int dispH = 0;

// ---- telemetry ----
float pitch=0, roll=0, hdg=0, alt=0, ias=0, vs=0;
float aoa=0, gLoad=1.0f, groundSpeed=0;
bool isStalling = false;
bool isOverspeed = false;

// ---- display ----
const uint16_t COL_SKY    = 0x2D9F;
const uint16_t COL_GROUND = 0x7A20;
const uint16_t COL_PANEL  = 0x2164;
const uint16_t COL_PANEL_EDGE = 0x7BEF;
const uint16_t COL_PANEL_DARK = 0x0861;
const uint16_t COL_MARK   = ST77XX_WHITE;
const uint16_t COL_ACCENT = 0x07FF;
const int TAPE_W = 30;

uint8_t activeTab = 0;

// ---- serial ----
char buf[320]; int idx=0;
uint32_t lastTelemetryMs = 0;

// ---- neopixel ----
Adafruit_NeoPixel strip(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
uint32_t lastBlink = 0;
bool blinkState = false;

// ---- parsing ----
void parseLine(char *s){
  char *p = s;
  char *end = nullptr;
  float v[10]={0};

  for(int i=0;i<10;i++){
    v[i]=strtof(p,&end);
    if(end==p) break;
    p=end;
    while(*p==','||*p==' ') p++;
  }

  pitch=v[0];
  roll=v[1];
  hdg=v[2];
  alt=v[3];
  ias=v[4];
  vs=v[5];
  aoa=v[6];
  gLoad=v[7];
  groundSpeed=v[8];

  lastTelemetryMs = millis();
}

// ---- horizon ----
void drawHorizonFast(float pitch, float roll, int x0, int y0, int w, int h){
  float rad = (-roll) * DEG_TO_RAD;
  float sr  = sinf(rad), cr = cosf(rad);
  int cx = x0 + w/2;
  int cy = y0 + h/2;
  float po = pitch * 2.0f;

  for(int y=y0;y<y0+h;y++){
    float dy = y - cy;
    float f = dy*cr - po;
    tft.drawFastHLine(x0,y,w, f>0 ? COL_GROUND : COL_SKY);
  }
}

// ---- tapes ----
void drawAirspeedTape(int x,int yTop,int yBottom,int cy){
  tft.fillRect(x,yTop,TAPE_W,yBottom-yTop,COL_PANEL_DARK);
  tft.drawRect(x,yTop,TAPE_W,yBottom-yTop,COL_PANEL_EDGE);

  char b[6];
  sprintf(b,"%3d",(int)ias);
  tft.setCursor(x+4,cy-3);
  tft.setTextColor(COL_ACCENT);
  tft.print(b);
}

void drawAltitudeTape(int x,int yTop,int yBottom,int cy){
  tft.fillRect(x,yTop,TAPE_W,yBottom-yTop,COL_PANEL_DARK);
  tft.drawRect(x,yTop,TAPE_W,yBottom-yTop,COL_PANEL_EDGE);

  char b[8];
  sprintf(b,"%d",(int)alt);
  tft.setCursor(x+4,cy-3);
  tft.setTextColor(COL_ACCENT);
  tft.print(b);
}

// ---- pages ----
void drawFltPage(){
  int cx = dispW/2;
  int cy = dispH/2;

  drawHorizonFast(pitch,roll,TAPE_W,10,dispW-2*TAPE_W,dispH-10);

  drawAirspeedTape(0,10,dispH,cy);
  drawAltitudeTape(dispW-TAPE_W,10,dispH,cy);
}

void drawNavPage(){
  tft.fillRect(0,10,dispW,dispH,ST77XX_BLACK);

  int cx = dispW/2;
  int cy = dispH/2;

  tft.drawCircle(cx,cy,30,COL_MARK);

  tft.setCursor(cx-10,cy);
  tft.print((int)hdg);
}

// ---- top bar ----
void drawTopInfoBar(){
  tft.fillRect(0,0,dispW,10,COL_PANEL_DARK);
  tft.setCursor(2,1);
  tft.setTextColor(COL_ACCENT);

  if(activeTab==0) tft.print("FLT");
  else tft.print("NAV");

  if(activeTab==0){
    tft.setCursor(dispW-40,1);
    tft.print("G ");
    tft.print(gLoad,1);
  } else {
    tft.setCursor(30,1);
    tft.print("HDG ");
    tft.print((int)hdg);
  }
}

// ---- neopixel ----
void updateNeo(){
  uint32_t now=millis();

  if(now-lastTelemetryMs>500){
    strip.setPixelColor(0,0);
  } else if(isStalling){
    if(now-lastBlink>300){ blinkState=!blinkState; lastBlink=now; }
    strip.setPixelColor(0, blinkState?strip.Color(255,0,0):0);
  } else {
    strip.setPixelColor(0, strip.Color(0,255,0));
  }
  strip.show();
}

// ---- setup ----
void setup(){
  Serial.begin(115200);
  pinMode(BTN_PIN,INPUT_PULLUP);
  pinMode(AP_BTN_PIN,INPUT_PULLUP);
  pinMode(LED_PIN,OUTPUT);

  strip.begin();
  strip.show();

  SPI.begin();
  tft.initR(INITR_MINI160x80_PLUGIN);
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);

  dispW = tft.width();
  dispH = tft.height();
}

// ---- loop ----
void loop(){

  while(Serial.available()){
    char c = Serial.read();
    if(c=='\n'){
      buf[idx]=0;
      parseLine(buf);
      idx=0;
    } else if(idx<319){
      buf[idx++]=c;
    }
  }

  if(digitalRead(BTN_PIN)==LOW){
    delay(150);
    activeTab = (activeTab+1)%2;
  }

  digitalWrite(LED_PIN, digitalRead(BTN_PIN)==LOW);

  updateNeo();

  drawTopInfoBar();

  if(activeTab==0) drawFltPage();
  else drawNavPage();
}
