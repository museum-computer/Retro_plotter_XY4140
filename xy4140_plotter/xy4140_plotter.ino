// Graficka jednotka XY 4140 - tisk ze SD kartky (HPGL)
//
// Arduino Nano ATmega328P 5V
//   PEN   D2  |  STEP D3  |  X/Y D4  |  +/- D5  |  READY D6 (4k7 ven)  |  READY D7 (10k dovnitr)
//   GND   GND plotteru
//   tlacitko D8 = dalsi soubor, tlacitko D9 = STOP, obe na GND
//   microSD na SPI: CS D10, MOSI D11, MISO D12, SCK D13
//   stavova LED na A0 (D13 je SPI SCK) + 1k rezistor na GND
//
// Soubory: jen v koreni SD karty (podadresare se neprochazi), poradi adresare.
// Doporucene kratke ASCII nazvy, protoze SD knihovna cte jen 8.3 jmeno.
// Tlacitko D8: v IDLE spusti dalsi soubor, pri tisku ukonci kresbu predcasne.
// SELFTEST_A 1: po zapnuti vykresli pismeno A a obrys plochy, SD ani tlacitka
// se nepouzivaji. Pro praci s HPGL z karty nastav 0.
#include <SPI.h>
#include <SD.h>
#include <XY4140.h>

#define PIN_PEN     2
#define PIN_STEP    3
#define PIN_XY      4
#define PIN_DIR     5
#define PIN_RDY_OUT 6
#define PIN_RDY_IN  7
#define BTN_NEXT    8
#define BTN_STOP    9
#define SD_CS       10
#define PIN_LED     A0

#define DIR_INVERT   0
#define USE_MAGNET   1
#define SPEED_US     1000
#define PEN_SETTLE   10000
#define HOME_BACKOFF 50
#define HOME_MAX     4000

// 1 krok = 0.1 mm, HPGL jednotka = 0.025 mm, tedy 4 jednotky na krok
#define HPGL_UNITS_PER_STEP 4

// Plocha XY 4140 podle referencniho kodu je MAX_X x MAX_Y = 2600 x 1850 kroku
// (260 x 185 mm). Kresby z archivu 2018-11-12 jsou ale delane na platno
// 185 x 261 mm, tedy s osami prohozenymi. Pro tyhle kresby je potreba
// SWAP_AXES 1, pro vlastni kresby generatoru (make_samples.ps1) nech 0.
// Rozmery skutecne zmer na puzde v etape 4 testovaciho sketche a pri padne
// potreby uprav MAX_X / MAX_Y v XY4140.h.
#define SWAP_AXES 0
#define X_FLIP 0
#define Y_FLIP 0
#define AUTO_ADVANCE 0

// 1 = pri zapnuti vykresli velke pismeno A a ramec plochy, bez SD karty
// a bez tlacitek. 0 = normalni rezim, cte HPGL z SD a ridi se tlacitkem D8.
#define SELFTEST_A 1

#define RCHUNK      96
#define CHARS_PER_PASS 200
#define NAME_LEN    14

static const uint8_t ST_IDLE = 0;
static const uint8_t ST_PRINTING = 1;
static const uint8_t ST_DONE = 2;
static const uint8_t ST_ERROR = 3;

static XY4140 drv;
static File src;

static uint8_t state = ST_ERROR;
static int8_t nextIndex = 0;

struct Btn { uint8_t pin; uint8_t st; uint32_t mark; };
static Btn btnNext = { BTN_NEXT, 0, 0 };
static Btn btnStop = { BTN_STOP, 0, 0 };

static uint8_t ledOn = 1, ledDiv = 2, ledPhase = 0;
static uint32_t ledMark = 0;

static char rbuf[RCHUNK];
static uint8_t rpos = 0, rlen = 0;
static uint8_t fileEof = 0;

static char cmd = 0;
static char num[12];
static uint8_t numLen = 0, inNum = 0;
static int8_t numSign = 0;
static int32_t args[2];
static uint8_t argCount = 0;
static int32_t curX = 0, curY = 0;
static uint8_t relative = 0, penDown = 0;

static bool btnEvent(Btn &b) {
  if (digitalRead(b.pin) == LOW) {
    if (b.st == 0) { b.st = 1; b.mark = millis(); }
    else if (b.st == 1 && millis() - b.mark >= 60) { b.st = 2; return true; }
  } else {
    b.st = 0;
  }
  return false;
}

// Volano ridicem pri krokovani, aby STOP fungoval i uprostred dlouhe useky.
// Tlacitko musi byt nizko ve dvou po sobe jdoucich dotazech, jinak by se
// zachytilo kmitani.
static uint8_t stopLow = 0;

static bool abortHook() {
  if (digitalRead(BTN_STOP) == LOW) {
    stopLow++;
    if (stopLow >= 2) return true;
  } else {
    stopLow = 0;
  }
  return false;
}

static void ledPattern(uint8_t on, uint8_t div) {
  ledOn = on;
  ledDiv = div ? div : 1;
}

static void serviceLed() {
  uint32_t now = millis();
  if (now - ledMark < 120) return;
  ledMark = now;
  ledPhase = (ledPhase + 1) % ledDiv;
  digitalWrite(PIN_LED, ledPhase < ledOn ? HIGH : LOW);
}

// SD knihovna vraci jen 8.3 jmeno z adresare. Dlouhe jmeno se na FAT zkraci
// na ALIAS~1 a cast pripony se pri tom ztrati (.hpgl -> .hp nebo .hpg),
// proto se bere jen zacatek pripony a prijimaji se i zkratene varianty.
static bool hasPlotExt(const char *name) {
  if (name[0] == '.') return false;
  const char *dot = strrchr(name, '.');
  if (dot == NULL) return false;
  dot++;
  char e[4];
  uint8_t n = 0;
  while (dot[n] != 0 && n < 3) {
    char c = dot[n];
    if (c >= 'A' && c <= 'Z') c = (char)(c + 32);
    e[n] = c;
    n++;
  }
  e[n] = 0;
  if (n == 0) return false;
  if (e[0] == 'p' && (e[1] == 0 || e[1] == 'l')) return true;
  if (e[0] == 'h' && (e[1] == 0 || e[1] == 'p' || e[1] == 'g')) return true;
  return false;
}

static int8_t countFiles() {
  int8_t n = 0;
  File root = SD.open("/");
  if (!root) return 0;
  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;
    if (!entry.isDirectory() && hasPlotExt(entry.name())) n++;
    entry.close();
  }
  root.close();
  return n;
}

static bool findNthFile(int8_t idx, char *out) {
  int8_t n = 0;
  bool found = false;
  File root = SD.open("/");
  if (!root) return false;
  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;
    if (!entry.isDirectory() && hasPlotExt(entry.name())) {
      if (n == idx) {
        strncpy(out, entry.name(), NAME_LEN - 1);
        out[NAME_LEN - 1] = 0;
        found = true;
        entry.close();
        break;
      }
      n++;
    }
    entry.close();
  }
  root.close();
  return found;
}

static void resetParser() {
  rpos = 0; rlen = 0; fileEof = 0;
  cmd = 0; numLen = 0; inNum = 0; numSign = 0; argCount = 0;
  curX = 0; curY = 0; relative = 0; penDown = 0;
}

static int readChar() {
  if (rpos < rlen) return (uint8_t)rbuf[rpos++];
  if (fileEof) return -1;
  int n = src.read(rbuf, RCHUNK);
  if (n <= 0) { fileEof = 1; return -1; }
  rlen = (uint8_t)n;
  rpos = 1;
  return (uint8_t)rbuf[0];
}

// Dvojice souradnic v jednom prikazu. HPGL posila v jednom PD/PU cely n rad
// dvojic (polyline), takze kazda dvojice se zpracuje hned, jakmile je
// kompletni, a prikaz konci strednikem nebo novym pismenem.
static bool isMoveCmd(char c) {
  return c == 'U' || c == 'D' || c == 'A' || c == 'R';
}

static void doMove() {
  int32_t tx, ty;
  if (relative) {
    tx = curX + args[0];
    ty = curY + args[1];
  } else {
    tx = args[0];
    ty = args[1];
  }
  curX = tx;
  curY = ty;

  int32_t dx = tx / HPGL_UNITS_PER_STEP;
  int32_t dy = ty / HPGL_UNITS_PER_STEP;
  int32_t sx = SWAP_AXES ? dy : dx;
  int32_t sy = SWAP_AXES ? dx : dy;
  if (X_FLIP) sx = XY4140::MAX_X - sx;
  if (Y_FLIP) sy = XY4140::MAX_Y - sy;
  if (sx < 0) sx = 0;
  if (sy < 0) sy = 0;
  if (sx > XY4140::MAX_X) sx = XY4140::MAX_X;
  if (sy > XY4140::MAX_Y) sy = XY4140::MAX_Y;

  if (penDown) drv.lineTo(sx, sy);
  else         drv.moveTo(sx, sy);
}

static void finishPair() {
  if (argCount == 2) {
    if (isMoveCmd(cmd)) doMove();
    argCount = 0;
  }
}

// cislo se ulozi do dvojice, predtim se dokonci predchozi par
static void commitNumber() {
  if (!inNum && numLen == 0) return;
  int32_t v = 0;
  for (uint8_t i = 0; i < numLen; i++) {
    if (num[i] < '0' || num[i] > '9') break;
    v = v * 10 + (num[i] - '0');
  }
  if (numSign < 0) v = -v;
  numLen = 0; inNum = 0; numSign = 0;
  finishPair();
  if (argCount < 2) args[argCount++] = v;
}

static void finishCommand() {
  if (cmd == 0) return;
  commitNumber();
  if (isMoveCmd(cmd)) {
    finishPair();
  } else if (cmd == 'S') {
    if (argCount >= 1) penDown = args[0] > 0 ? 1 : 0;
  }
  cmd = 0;
  argCount = 0;
}

static void startCommand(char c) {
  finishCommand();
  cmd = c;
  switch (c) {
    case 'I': curX = 0; curY = 0; relative = 0; penDown = 0; break;
    case 'U': penDown = 0; break;
    case 'D': penDown = 1; break;
    case 'A': relative = 0; break;
    case 'R': relative = 1; break;
    default: break;
  }
}

static void startNumber() {
  finishPair();
  numLen = 0; inNum = 0; numSign = 0;
}

static void handleChar(char c) {
  if (c >= 'a' && c <= 'z') c = (char)(c - 32);
  if (c >= '0' && c <= '9') {
    if (!inNum) startNumber();
    if (numLen < sizeof(num) - 1) num[numLen++] = c;
    inNum = 1;
    return;
  }
  if (c == '.') {
    if (!inNum) startNumber();
    if (numLen < sizeof(num) - 1) num[numLen++] = c;
    inNum = 1;
    return;
  }
  if (c == '-' || c == '+') {
    if (!inNum) startNumber();
    if (numLen == 0) numSign = (c == '-') ? -1 : 1;
    inNum = 1;
    return;
  }
  if (c == ',') { commitNumber(); return; }
  if (c == ';') { finishCommand(); return; }
  if (c >= 'A' && c <= 'Z') { startCommand(c); return; }
}

static void stopPrint() {
  drv.requestStop();
  drv.clearStop();
  drv.park();
  drv.idle();
  src.close();
  state = ST_IDLE;
  ledPattern(1, 10);
}

static void startPrint() {
  int8_t total = countFiles();
  if (total <= 0) { state = ST_ERROR; ledPattern(1, 2); return; }
  if (nextIndex >= total) nextIndex = 0;
  if (nextIndex < 0) nextIndex = 0;

  char name[NAME_LEN];
  if (!findNthFile(nextIndex, name)) { state = ST_ERROR; ledPattern(1, 2); return; }
  src = SD.open(name);
  if (!src) { state = ST_ERROR; ledPattern(1, 2); return; }

  drv.clearStop();
  drv.home();
  resetParser();
  state = ST_PRINTING;
  ledPattern(1, 1);
}

static void finishPrint() {
  drv.park();
  drv.idle();
  src.close();
  state = ST_DONE;
  ledPattern(1, 3);
  nextIndex++;
  if (AUTO_ADVANCE) startPrint();
}

static void pumpParser() {
  uint16_t budget = CHARS_PER_PASS;
  int c;
  while (budget-- > 0) {
    if (drv.stopRequested()) { stopPrint(); return; }
    c = readChar();
    if (c < 0) {
      finishCommand();
      finishPrint();
      return;
    }
    handleChar((char)c);
  }
}

static void service() {
  if (btnEvent(btnStop)) {
    if (state == ST_PRINTING) stopPrint();
  }
  if (btnEvent(btnNext)) {
    if (state == ST_PRINTING) {
      stopPrint();
    } else {
      startPrint();
    }
  }
  serviceLed();
}

// Pismeno A do obdrezniku (x1,y1)-(x2,y2) v krocich: dve nohy a pricka.
// Vypocet z obdrezniku, aby se pismeno veslo do libovolne velikosti.
static void drawLetterA(int32_t x1, int32_t y1, int32_t x2, int32_t y2) {
  int32_t cx = (x1 + x2) / 2;
  int32_t w = (x2 - x1) / 2;
  int32_t h = y2 - y1;
  int32_t lx = cx - w, rx = cx + w;      // paty noh
  int32_t bar = y1 + h / 3;             // vyska pricky
  int32_t bl = cx - w + w / 3;          // kraje pricky na nohach
  int32_t br = cx + w - w / 3;
  drv.park();
  drv.moveTo(lx, y1);
  drv.lineTo(cx, y2);
  drv.lineTo(rx, y1);
  drv.moveTo(bl, bar);
  drv.lineTo(br, bar);
}

// Obrys plochy s rezervou, kontrola celyho svehu a orientace
static void drawFrame(int32_t reserve) {
  int32_t x2 = XY4140::MAX_X - reserve;
  int32_t y2 = XY4140::MAX_Y - reserve;
  drv.park();
  drv.moveTo(reserve, reserve);
  drv.lineTo(x2, reserve);
  drv.lineTo(x2, y2);
  drv.lineTo(reserve, y2);
  drv.lineTo(reserve, reserve);
}

// Startovni zkouska bez SD karty: homing, pismeno, obrys plochy
static void runSelfTest() {
  drv.home();
  drv.moveTo(0, 0);
  drawLetterA(600, 150, 2000, 1700);
  drv.park();
  drawFrame(60);
  drv.park();
}

void setup() {
  pinMode(BTN_NEXT, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  drv.begin(PIN_PEN, PIN_STEP, PIN_XY, PIN_DIR, PIN_RDY_OUT, PIN_RDY_IN);
  drv.setDirInvert(DIR_INVERT);
  drv.setMagPen(USE_MAGNET);
  drv.setStepDelay(SPEED_US);
  drv.setPenSettle(PEN_SETTLE);
  drv.setHomingBackoff(HOME_BACKOFF);
  drv.setHomingMaxSteps(HOME_MAX);
  drv.setAbortHook(abortHook);
  drv.idle();

  if (SELFTEST_A) {
    // rezim zkousky: jen kresba, SD karta ani tlacitka se nepouzivaji
    ledPattern(1, 1);
    runSelfTest();
    state = ST_DONE;
    ledPattern(1, 3);
    return;
  }

  if (!SD.begin(SD_CS)) {
    state = ST_ERROR;
    ledPattern(1, 2);
    return;
  }
  nextIndex = 0;
  state = ST_IDLE;
  ledPattern(1, 10);
}

void loop() {
  if (SELFTEST_A) { serviceLed(); return; }
  service();
  if (state == ST_PRINTING) pumpParser();
}
