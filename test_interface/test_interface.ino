// Graficka jednotka XY 4140 - test rozhrani
// Konektor 6 pinu: 1 PEN | 2 STEP | 3 X/Y | 4 +/- | 5 READY | 6 GND
//
// Arduino Nano ATmega328P 5V - pripojeni primo na 5V TTL, bez prevodniku
//   PEN   D2   LOW = pero nahoru, HIGH = pero dolu
//   STEP  D3   LOW = impulz (krok), HIGH = pauza
//   X/Y   D4   LOW = CART (Y), HIGH = PAPER (X)
//   +/-   D5   LOW = smer A, HIGH = smer B
//   READY D6   vystup, 4k7 v serii do pinu 5 plotteru (slaby, nebrani plotteru stahnout linku)
//   READY D7   vstup, 10k v serii z pinu 5 plotteru (chrani MCU pred 5.5V z vysokonapetoveho vyonu)
//   GND   GND  spojena se zemnou plotteru
//   tlacitka D8 a D9 na GND, INPUT_PULLUP
//
// Pin 5 je jednosmerne napajena linka: D6 ji ridi, D7 ji cte.
// Serial 115200 je v tomto testu vyzadovan, pro odhad casu a chyb.

#include <XY4140.h>

#define PIN_PEN     2
#define PIN_STEP    3
#define PIN_XY      4
#define PIN_DIR     5
#define PIN_RDY_OUT 6
#define PIN_RDY_IN  7
#define BTN_NEXT    8
#define BTN_STOP    9
#define PIN_LED     13

#define DIR_INVERT   0
#define USE_MAGNET   1
#define SPEED_US     1000
#define PEN_SETTLE   10000
#define HOME_BACKOFF 50
#define HOME_MAX     4000

static XY4140 drv;
static int stage = 0;
static uint8_t stopLow = 0;

// Volano pri krokovani, aby STOP fungoval i uprostred dlouhe useky
static bool abortHook() {
  if (digitalRead(BTN_STOP) == LOW) {
    stopLow++;
    if (stopLow >= 2) return true;
  } else {
    stopLow = 0;
  }
  return false;
}

static void setLed(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(PIN_LED, HIGH); delay(120);
    digitalWrite(PIN_LED, LOW);  delay(120);
  }
}

static void stageSelfTest() {
  Serial.println(F("-- STAGE 0: self-test --"));
  drv.idle();
  digitalWrite(PIN_RDY_OUT, HIGH);
  delayMicroseconds(SPEED_US);
  Serial.print(F("READY pri D6=HIGH, D7="));
  Serial.println(digitalRead(PIN_RDY_IN) ? F("HIGH (nepripraveno)") : F("LOW (ready)"));
  digitalWrite(PIN_RDY_OUT, LOW);
  delayMicroseconds(SPEED_US);
  Serial.print(F("READY pri D6=LOW,  D7="));
  Serial.println(digitalRead(PIN_RDY_IN) ? F("HIGH") : F("LOW (nucene, zapis vynucen)"));
  drv.idle();
  Serial.println(F("0 = LED nesviti nebo pulzuje"));
  setLed(1);
}

static void stagePenTest() {
  Serial.println(F("-- STAGE 1: pero --"));
  Serial.println(F("3x magnet: kratce LOW, pak 1.2s HIGH"));
  for (int i = 0; i < 3; i++) {
    digitalWrite(PIN_PEN, LOW);  delay(600);
    digitalWrite(PIN_PEN, HIGH); delay(1200);
  }
  digitalWrite(PIN_PEN, LOW); delay(600);
  Serial.println(F("pero se ma v magnetu udrzat 1.2s, jinak spatna polarita"));
  setLed(2);
}

static void stageHoming() {
  Serial.println(F("-- STAGE 2: homing --"));
  uint32_t t = millis();
  int32_t n = drv.home();
  Serial.print(F("kroku do konce drahy: "));
  Serial.println(n);
  Serial.print(F("doba: ")); Serial.print(millis() - t); Serial.println(F(" ms"));
  if (n == XY4140::HOME_ALREADY) {
    Serial.println(F("READY je rovnou ready - spinac zaznamenany hned za startem"));
  } else if (n == XY4140::HOME_FAILED) {
    Serial.print(F("spinac nereagoval za ")); Serial.println(HOME_MAX);
  } else if (n > 1000 && n < 2500) {
    Serial.println(F("~1850 ocekavano: osa Y a smer OK"));
  } else if (n >= 2500) {
    Serial.println(F(">2500: mozna X/Y jsou prohozene"));
  } else {
    Serial.println(F("malo kroku: spinac za startem nebo spatny smer"));
  }
  setLed(3);
}

static void stageAxes() {
  Serial.println(F("-- STAGE 3: osy a smer --"));
  drv.home();
  int32_t x0 = drv.positionX(), y0 = drv.positionY();
  drv.lineTo(x0, y0 + 300);
  drv.lineTo(x0 + 200, y0 + 300);
  drv.lineTo(x0 + 200, y0);
  drv.lineTo(x0, y0);
  drv.park();
  Serial.println(F("obdelnik 20x30mm vlevo nahore od domu"));
  Serial.println(F("ma byt normalni, ne zrcadleny"));
  Serial.print(F("DIR_INVERT=")); Serial.print(DIR_INVERT);
  Serial.println(F("  pri spatnem smeru sem vloz 1 a nahraj znovu"));
  setLed(4);
}

static void stageFrame() {
  Serial.println(F("-- STAGE 4: plocha --"));
  uint32_t t = millis();
  drv.home();
  drv.lineTo(XY4140::MAX_X, 0);
  drv.lineTo(XY4140::MAX_X, XY4140::MAX_Y);
  drv.lineTo(0, XY4140::MAX_Y);
  drv.lineTo(0, 0);
  drv.lineTo(XY4140::MAX_X, XY4140::MAX_Y);
  drv.lineTo(XY4140::MAX_X, 0);
  for (int i = 0; i <= 10; i++) {
    int32_t v = (int32_t)XY4140::MAX_Y * i / 10;
    drv.lineTo(1300, v);
    drv.lineTo(1400, v);
  }
  drv.lineTo(0, 0);
  for (int i = 0; i <= 10; i++) {
    int32_t v = (int32_t)XY4140::MAX_X * i / 10;
    drv.lineTo(v, 0);
    drv.lineTo(v, 100);
  }
  drv.lineTo(0, 0);
  drv.lineTo(0, 800);
  int32_t cx = 800, cy = 925, r = 500;
  for (int a = 0; a < 360; a += 2) {
    double rad = a * 0.01745329251994;
    int32_t px = cx + (int32_t)(r * sin(rad));
    int32_t py = cy + (int32_t)(r * cos(rad));
    drv.lineTo(px, py);
  }
  drv.park();
  Serial.print(F("doba: ")); Serial.print(millis() - t); Serial.println(F(" ms"));
  setLed(5);
}

void setup() {
  pinMode(BTN_NEXT, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);

  drv.begin(PIN_PEN, PIN_STEP, PIN_XY, PIN_DIR, PIN_RDY_OUT, PIN_RDY_IN);
  drv.setDirInvert(DIR_INVERT);
  drv.setMagPen(USE_MAGNET);
  drv.setStepDelay(SPEED_US);
  drv.setPenSettle(PEN_SETTLE);
  drv.setHomingBackoff(HOME_BACKOFF);
  drv.setHomingMaxSteps(HOME_MAX);
  drv.setAbortHook(abortHook);

  Serial.begin(115200);
  Serial.println(F("XY4140 test ready - stisk D8 pro dalsi etap"));
  stageSelfTest();
}

void loop() {
  if (digitalRead(BTN_STOP) == LOW) {
    drv.requestStop();
    drv.park();
    drv.idle();
    Serial.println(F("STOP"));
    delay(500);
  }
  if (digitalRead(BTN_NEXT) == LOW) {
    delay(50);
    while (digitalRead(BTN_NEXT) == LOW) {}
    drv.clearStop();
    switch (stage) {
      case 0: stagePenTest(); break;
      case 1: stageHoming();  break;
      case 2: stageAxes();    break;
      case 3: stageFrame();   break;
    }
    stage++;
    if (stage > 4) {
      stage = 0;
      Serial.println(F("-- restart --"));
      stageSelfTest();
    }
  }
}
