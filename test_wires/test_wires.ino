// XY 4140 - automaticke hledani dratu na konektoru pripravkem
//
// Bezi hned po napajeni. Tlacitka D8 a D9 nejsou zapojeny a nepouzivaji se:
// uzivatel ma zamerne obe ruce zanepraspnute a hleda draty pripojenim.
//
// Osnoveni propojeni, zmerene na pripravku (odpor ke kontaktu 2 = GND pri
// vypnutem napajeni) a overene chovanim stroje. Cislovani kontaktu na plosce
// neni cislovani pinu konektoru XP5:
//
//   kontakt 2  -> GND   XP5 pin 6
//   kontakt 5  -> D3    MOV, XP5 pin 2, potvrzen: bez nej se papir nepodava
//   kontakt 6  -> D4    X/Y, XP5 pin 3, potvrzen: L jede kaletka, H jede papir
//   kontakt 3  -> D5    +/-, XP5 pin 4, potvrzen: da se jet na obe strany
//   kontakt 4  -> D2    PEN, XP5 pin 1, urceno vyborem, zatim nepotvrzeno
//   kontakt 1  -> D7    READY, XP5 pin 5, vystup pripravku, jen se cte
//
// D6 se na pripravku nezapojuje a nepouziva: READY je jednosmerne vystup
// pripravku, ridici vystup Nano by s nim soupal. Proto se D6 vraci na vstup
// hned po drv.begin().
//
// PEN neni spinac pera, ale primo spinac pera grafopstrojitaru: podle popisu
// H znamena pero dolu a L nahoru. Kdyz na D2 nereaguje, je bud chyba
// v tomto vyberu, nebo nepomoci stisk knohly PEN na skrince.
//
// POZOR: pripravcek nesmi nic posilat do vypnuteho grafopstrojitaru. Tok pak
// tece zpet pres ochranne diody vstupu a napaji logiku pripravku z napeti
// Nano. Motor jen vrci a nema silu, ale logika je tim napajena napolo a je to
// nebezpecne. Kdyz je grafopstrojitar vypnuty, pouzije se PROBE_MODE 2 nebo
// se pripravcek odpoji. Na vstupy se strida 1 kOhm, protoze vstupy berou 2 mA.
//
// PROBE_MODE 5 (aktualni): smer podavani papiru. Osa X/Y je vybrana napevno
// na H (papir), +/- se strida: 1 s chodu jednim smerem, pauza 1 s, 1 s chodu
// druhym smerem, pauza 1 s. Po kazdem chodu se cte D7.
//
// PROBE_MODE 6: hledani leveho kraje kaletky. Podle popisu je pro zapis
// potreba nejlevejsi poloha kaletky, jinak je zapis blokovan a pero nehne.
// X/Y = L (motor kaletky), +/- = L (doleva), krokuje se primo na D3. Pri
// zmene D7 se hned zastavi a odjede o HOME_BACKOFF kroku od dorazu.
//
// PROBE_MODE 4: prohledani polarity pera. PEN je podle popisu primo spinac
// pera, H = dolu, takze se strida uroven na D2 a pri kazdem prehozeni se cte
// D7. Kdyz ready=0, zapis je blokovan a pero nehne.
//
// PROBE_MODE 0: chodiste tam a zpet po obou osach, aby se pri pristupu k
// pripravku nedojelo do dorazu. Pred kazdym chodem se ceka na READY na D7.
//
// PROBE_MODE 2: bezpecny rezim, piny D2..D7 jsou vstupy a na grafopstrojitar
// se nic neposila. Pouziva se pri praci s multimetrem nebo kdyz hrozi
// prehrati motoru.
//
// PROBE_MODE 1: majak, vsechny ctyry signalni piny D2, D3, D4, D5 jedou
// soucasne, 1 s HIGH a 1 s LOW. Nepouzivat: ridi takze PEN i X/Y, coz pri
// vypnutem napajeni zbytecne hreje motor a zpetne napaji logiku.
//
// Pen je stale nahore drzen magnetem, aby se pri hledani neskrabalo. Homing se
// nedela, protoze vychodiste neni zname a konecovy spinac jeste nezapojeny.
//
// Vypnuti je vypinacem sitoveho napajeni na grafopstrojite, jinak ukonceni
// neni. Seriovy vypis je jen pro protokol, chod na nem nezavisi.

#include <XY4140.h>

#define PIN_PEN     2
#define PIN_STEP    3
#define PIN_XY      4
#define PIN_DIR     5
#define PIN_RDY_OUT 6
#define PIN_RDY_IN  7
#define PIN_LED     13

#define DIR_INVERT  0
#define STEP_US     5000
#define PEN_SETTLE  10000

#define ONE_CM      100
#define LEG_STEPS   100   // kroku na chod; pri STEP_US 5000 je to 1 s chodu
#define LEG_PAUSE   2000  // pauza mezi chody, aby motor nemel trvaly nabeh
#define RANGE       300
#define READY_WAIT_MS 2000 // jak dlouho se ceka na READY, ne zacne krokovat
#define RDY_HEARTBEAT_MS 1000   // how often to print the D7 level while searching
#define START_WAIT_MS  6000   // grace time after reset, hands may be busy
#define PROBE_MODE       5     // 0 = chodiste, 1 = majak, 2 = odpojeno
                                // 3 = pero, 4 = prohledani polarity pera,
                                // 5 = smer podavani, 6 = hledani leveho kraje
#define BEACON_MS     1000     // delka jedne faze majaku: 1 s HIGH, 1 s LOW
#define PEN_TEST_MS   1000     // delka faze pri zkousce pera na D2
#define DIR_STEPS     100      // kroku na fazi, pri STEP_US 5000 je to 1 s chodu
#define DIR_PAUSE_MS  1000     // pauza po fazi, aby motor nemel trvaly nabeh
#define HOME_STEPS    100      // kroku na vypis pri hledani leveho kraje
#define HOME_BACKOFF   20      // odjezd od dorazu, kdyz D7 zmeri hranu
#define HOME_MAX     1800      // pojistka, delka osy kaletky je 1800 kroku

static XY4140 drv;
static int32_t vx, vy;
static uint8_t legIndex = 0;
static uint32_t nextAt = 0;
static uint8_t autoMode = 1;
static uint8_t xyAxis = XY4140::AXIS_CART;   // jen pro rucni prikazy + a -
static uint8_t penDown = 0;
static uint8_t homed = 0;
static uint32_t rdyAt = 0;

static int32_t relX() { return vx - RANGE; }
static int32_t relY() { return vy - RANGE; }

static void reportPos() {
  if (homed) {
    Serial.print(F("abs X=")); Serial.print(drv.positionX());
    Serial.print(F(" Y=")); Serial.println(drv.positionY());
  } else {
    Serial.print(F("od zapnuti X=")); Serial.print(relX());
    Serial.print(F(" Y=")); Serial.println(relY());
  }
}

static void reportPins() {
  Serial.print(F("PEN="));     Serial.print(digitalRead(PIN_PEN));
  Serial.print(F(" STEP="));   Serial.print(digitalRead(PIN_STEP));
  Serial.print(F(" XY="));     Serial.print(digitalRead(PIN_XY));
  Serial.print(F(" DIR="));    Serial.print(digitalRead(PIN_DIR));
  Serial.print(F(" RDY_IN=")); Serial.print(digitalRead(PIN_RDY_IN));
  Serial.println();
}

// Hledani READY: na D7 se jen poslouche, D6 se vubec nedotyka. Kontakt 1 je
// podle popisu vystup pripravku (H = nepripraveno, L = pripraveno), takze
// ridici vystup Nano by s nim soupal a pulzovat ho nema smysl.
static void probeReady() {
  if ((int32_t)(millis() - rdyAt) < 0) return;

  rdyAt = millis() + RDY_HEARTBEAT_MS;
  Serial.print(F("RDY_IN="));
  Serial.println(digitalRead(PIN_RDY_IN));
}

static void showHelp() {
  Serial.println(F("bezi samo, tlacitka nejsou potrebna"));
  Serial.println(F("prikazy jen pro protokol, male pismeno:"));
  Serial.println(F("  m  zastavit / rozjet automaticky chod"));
  Serial.println(F("  v  vypsat stavy pinu"));
  Serial.println(F("  +  -  posun o 1 cm po aktualni ose"));
  Serial.println(F("  s  jeden krok 0.1 mm"));
  Serial.println(F("  a  rucni osa CART <-> PAPER"));
  Serial.println(F("  h  homing (az po zapojeni spinace)"));
  Serial.println(F("  p  pero nahoru / dolu"));
  Serial.println(F("  z  vynulovat relativni pocatek"));
  Serial.println(F("  ?  tato napoveda"));
}

// READY je podle popisu vystup pripravku, ne obousmerna linka: vystup
// RS-triggeru, H = nepripraveno, L = pripraveno. Zadostni signal tim neni, takze
// se D6 nesmi ridit - hadalo by to s vystupem pripravku. D7 se jen cte.
static uint8_t readReady() {
  return digitalRead(PIN_RDY_IN) == LOW;
}

// Grafopstrojitar potvrzuje pripravenost krokovat na kontaktu 1, ktery je
// pripojeny na D7. Kdyz READY neprijde, krokovani se vubec nespusti: pri
// rychlem krokovani bez odpovedi zustava budic v polovine kroku a motory se
// grze. Tady se misto toho jen ceka a pise, ktery kontakt na pripravku READY
// vlastne je.
static uint8_t waitReady(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while ((int32_t)(millis() - t0) < (int32_t)timeoutMs) {
    if (readReady()) return 1;
  }
  return 0;
}

// jeden chod: osa se strida kazdy chod, aby se prepnuti na D4 dalo poznat za
// behu. 100 kroku pri 5 ms na pulz = 1 s chodu, pak pauza kvuli chlazeni.
// smer musi jit parovane s osou: CART-, PAPER-, CART+, PAPER+ - tak se kurzor
// vraci na start a nedojede do dorazu.
static void runLeg() {
  static const uint8_t axisTab[4] = { XY4140::AXIS_CART,  XY4140::AXIS_PAPER,
                                      XY4140::AXIS_CART,  XY4140::AXIS_PAPER };
  static const int8_t dirTab[4] = { -1, -1, 1, 1 };
  uint8_t ax = axisTab[legIndex];
  int32_t tx = vx, ty = vy;
  if (ax == XY4140::AXIS_PAPER) tx += dirTab[legIndex] * LEG_STEPS;
  else                          ty += dirTab[legIndex] * LEG_STEPS;

  if (!homed && (tx < 0 || ty < 0 || tx > 2 * RANGE || ty > 2 * RANGE)) {
    // po vraceni z klidu je virtualni souradnice v divici. Reset na stred,
    // jinak by test zustal stahnuty v hlase a nehybal by se vubec.
    vx = RANGE;
    vy = RANGE;
    Serial.println(F("kraj dosahl, pocatek vresetovan na stred"));
    return;
  }

  if (!waitReady(READY_WAIT_MS)) {
    Serial.print(F("READY neprijde za "));
    Serial.print(READY_WAIT_MS / 1000);
    Serial.print(F(" s, krokovani zastaveno, D7="));
    Serial.println(digitalRead(PIN_RDY_IN));
    return;
  }

  Serial.print(F("chod "));
  Serial.print(legIndex);
  Serial.print(F(" osa "));
  Serial.print(ax == XY4140::AXIS_CART ? F("X/Y") : F("PAPER"));
  Serial.print(F(" smer "));
  Serial.print(dirTab[legIndex] < 0 ? F("A") : F("B"));
  Serial.print(F(" -> "));
  vx = tx;
  vy = ty;
  reportPos();
  digitalWrite(PIN_LED, HIGH);
  drv.moveTo(vx, vy);
  digitalWrite(PIN_LED, LOW);
}

static void moveBy(int32_t steps) {
  int32_t tx = vx, ty = vy;
  if (xyAxis == XY4140::AXIS_PAPER) tx += steps; else ty += steps;
  if (!homed && (tx < 0 || ty < 0 || tx > 2 * RANGE || ty > 2 * RANGE)) {
    Serial.println(F("mimo rozsah +/-3 cm, posun zrusen"));
    return;
  }
  vx = tx;
  vy = ty;
  digitalWrite(PIN_LED, HIGH);
  drv.moveTo(vx, vy);
  digitalWrite(PIN_LED, LOW);
}

static void doHome() {
  Serial.println(F("homing po CART"));
  int32_t n = drv.home();
  Serial.print(F("kroku: ")); Serial.println(n);
  if (n == XY4140::HOME_ALREADY) Serial.println(F("spinac hned na zacatku"));
  else if (n == XY4140::HOME_FAILED) Serial.println(F("spinac nereagoval"));
  else if (n > 1000 && n < 2500) Serial.println(F("~1850: osa a smer OK"));
  else if (n >= 2500) Serial.println(F(">2500: X/Y prohozene"));
  else Serial.println(F("malo kroku: spinac za startem nebo spatny smer"));
  vx = drv.positionX();
  vy = drv.positionY();
  homed = 1;
  reportPos();
}

static void handle(char c) {
  switch (c) {
    case 'm':
      autoMode = !autoMode;
      Serial.println(autoMode ? F("automaticky chod zapnut") : F("automaticky chod zastaven"));
      break;
    case 'a':
      xyAxis = (xyAxis == XY4140::AXIS_CART) ? XY4140::AXIS_PAPER : XY4140::AXIS_CART;
      Serial.println(xyAxis == XY4140::AXIS_CART
        ? F("rukcni osa CART (kaletka s perem)")
        : F("rukcni osa PAPER (posuv papiru)"));
      break;
    case 'v': reportPins(); break;
    case '?': showHelp(); break;
    case '+': moveBy( ONE_CM); break;
    case '-': moveBy(-ONE_CM); break;
    case 's': moveBy(1); break;
    case 'h': doHome(); break;
    case 'z':
      drv.setPosition(RANGE, RANGE);
      vx = RANGE;
      vy = RANGE;
      homed = 0;
      Serial.println(F("relativni pocatek na 0,0"));
      reportPos();
      break;
    case 'p':
      penDown = !penDown;
      digitalWrite(PIN_PEN, penDown ? HIGH : LOW);
      delay(PEN_SETTLE / 1000);
      Serial.println(penDown ? F("pero dolu") : F("pero nahoru"));
      break;
  }
}

// Majak pro hledani metodou tykani: ctyry signalni piny jedou soucasne, takze
// jedno mereni odpovida jednomu typu signalu a nemus se nic prepinat na Nano.
// LED ridi fazi, aby se dalo pozorovat i bez serioveho portu.
static void beacon() {
  digitalWrite(PIN_PEN, HIGH);
  digitalWrite(PIN_STEP, HIGH);
  digitalWrite(PIN_XY, HIGH);
  digitalWrite(PIN_DIR, HIGH);
  digitalWrite(PIN_LED, HIGH);
  Serial.println(F("D2 D3 D4 D5 HIGH 1 s"));
  delay(BEACON_MS);

  digitalWrite(PIN_PEN, LOW);
  digitalWrite(PIN_STEP, LOW);
  digitalWrite(PIN_XY, LOW);
  digitalWrite(PIN_DIR, LOW);
  digitalWrite(PIN_LED, LOW);
  Serial.println(F("D2 D3 D4 D5 LOW 1 s"));
  delay(BEACON_MS);

  // vyprazdnit prijimaci buffer, jinak po nekolika minutach zacne
  // vypis zdrzovat
  while (Serial.available()) Serial.read();
}

// Smer podavani papiru: osa X/Y je vybrana napevno na H (motor papiru), +/- se
// strida. 100 kroku pri STEP_US 5000 je 1 s chodu, pak pauza 1 s kvuli chlazeni.
// Cisty test 1 s vpred a 1 s vzad bez dojezdu do dorazu. Krokuje se primo na
// D3, ne preses moveTo, protoze zde zalezi na tom, kdy se nastavi smer, a ne na
// vnitrnim pocitadle polohy pripravku.
static void dirSweep() {
  static uint32_t phaseAt = 0;
  static uint8_t forward = 0;
  uint32_t now = millis();

  if ((int32_t)(now - phaseAt) < 0) return;

  forward = !forward;
  digitalWrite(PIN_XY, HIGH);   // H = motor podavani papiru
  digitalWrite(PIN_DIR, forward ? HIGH : LOW);
  delayMicroseconds(STEP_US);

  Serial.print(forward ? F("PAPER vpred  ") : F("PAPER vzad   "));
  Serial.print(F("+/-="));
  Serial.print(digitalRead(PIN_DIR));
  Serial.print(F("  D7="));
  Serial.print(digitalRead(PIN_RDY_IN));
  // H na READY znamena, ze pripravstek zapis blokuje a pero nehne
  Serial.println(readReady() ? F("  ready") : F("  BLOKOVANO"));

  phaseAt = millis() + DIR_PAUSE_MS;
  digitalWrite(PIN_LED, HIGH);
  for (int16_t i = 0; i < DIR_STEPS; i++) {
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(STEP_US);
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(STEP_US);
  }
  digitalWrite(PIN_LED, LOW);
  while (Serial.available()) Serial.read();
}

// Prohledani polarity pera. PEN je podle popisu primo spinac pera, H = dolu,
// takze se jen strida uroven na D2 a pri kazdem prehozeni se cte D7. Kdyz
// ready=0, zapis je blokovan a pripravcek pero nehne; pak je potreba
// grafopstrojatar restartovat, aby se trigger za nim vracil do L.
static void penSweep() {
  static uint8_t penDown = 0;

  // D6 se uz neridi, ready se jen cte. PEN se prepina, aby bylo videt, jestli
  // pripravcek na nej vubec reaguje: podle popisu H znamena pero dolu.
  digitalWrite(PIN_PEN, penDown ? HIGH : LOW);
  digitalWrite(PIN_LED, penDown ? HIGH : LOW);

  Serial.print(F("PEN "));
  Serial.print(penDown ? F("LOW ") : F("HIGH"));
  Serial.print(F("  D7="));
  Serial.print(digitalRead(PIN_RDY_IN));
  Serial.print(F("  ready="));
  Serial.println(readReady());

  penDown = !penDown;
  delay(PEN_TEST_MS);
  while (Serial.available()) Serial.read();
}

// Hledani leveho kraje: podle popisu je potreba najprv najit nejlevejsi
// polohu kaletky, jinak pripravstek blokuje zapis a pero nehne. Podle popisu
// je L na +/- doleva, takze se jede na L. Pri kazdem bloku kroku se cte D7;
// jakmile se uroven zmeni, je to konec. Bezpecnostni zastaveni HOME_MAX, aby
// se kaletka nemohla dobit do dorazu, kdyz signaly vbec nejdou.
static void homeSearch() {
  static int32_t n = 0;
  static uint8_t lastLevel = 2;
  uint8_t level = digitalRead(PIN_RDY_IN);

  digitalWrite(PIN_XY, LOW);    // L = motor kaletky
  digitalWrite(PIN_DIR, LOW);   // L = doleva
  delayMicroseconds(STEP_US);

  if (level != lastLevel) {
    Serial.print(F("ZMENA D7 na "));
    Serial.print(level);
    Serial.print(F(" po "));
    Serial.print(n);
    Serial.println(F(" krocich"));
    lastLevel = level;
  }

  digitalWrite(PIN_LED, HIGH);
  for (int16_t i = 0; i < HOME_STEPS; i++) {
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(STEP_US);
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(STEP_US);
    n++;
  }
  digitalWrite(PIN_LED, LOW);

  Serial.print(F("n="));
  Serial.print(n);
  Serial.print(F("  D7="));
  Serial.print(digitalRead(PIN_RDY_IN));
  Serial.println(readReady() ? F("  ready") : F("  BLOKOVANO"));

  if (n >= HOME_MAX) {
    Serial.println(F("HOME_MAX dosahl, zastavuji, smer nejspis obraceny"));
    while (1) delay(1000);
  }
  while (Serial.available()) Serial.read();
}

// Bezpecny rezim: signalni piny jako vstupy, aby pripravcek nic na grafopstrojitar
// netlacil. Pouziva se pri praci s multimetrem nebo pripravek jinudy nez ceh.
// Kdyby piny zustaly jako vystupy, pri kazdem zapnuti by do privodu lezla
// zpetna diode Nano a vysky napeti na kontaktu nebylo by 5 V, ale mensi.
static void odpojeno() {
  pinMode(PIN_PEN, INPUT);
  pinMode(PIN_STEP, INPUT);
  pinMode(PIN_XY, INPUT);
  pinMode(PIN_DIR, INPUT);
  pinMode(PIN_RDY_OUT, INPUT);
  pinMode(PIN_RDY_IN, INPUT);
}

void setup() {
  pinMode(PIN_LED, OUTPUT);

  // odreseni po resetu, aby se pripojeni k pristroji stihlo udelat
  // a ruce nebyly v miste, kam hned pojede kartezka
  digitalWrite(PIN_LED, HIGH);
  delay(START_WAIT_MS);
  digitalWrite(PIN_LED, LOW);

  drv.begin(PIN_PEN, PIN_STEP, PIN_XY, PIN_DIR, PIN_RDY_OUT, PIN_RDY_IN);
  drv.setDirInvert(DIR_INVERT);
  drv.setMagPen(1);
  drv.setStepDelay(STEP_US);
  drv.setPenSettle(PEN_SETTLE);
  drv.setAbortHook(0);

  // odpojit az po begin, ktery piny nastavuje na vystupy
  if (PROBE_MODE == 2) odpojeno();

  // D6 ridi pripravku nesmi, protoze kontakt 1 je vystup READY. Po begin by
  // tam byl vystup Nano, ktery by s vystupem pripravku soupal.
  pinMode(PIN_RDY_OUT, INPUT);

  // pri zkousce pera nesmi grafopstrojitar udelat ani krok, proto se
  // vsechny ridici piny drzi v nule a hyba se jen pero
  if (PROBE_MODE >= 3 && PROBE_MODE <= 6) {
    digitalWrite(PIN_STEP, LOW);
    digitalWrite(PIN_XY, LOW);
    if (PROBE_MODE == 4) digitalWrite(PIN_DIR, HIGH);
  }

  vx = RANGE;
  vy = RANGE;
  drv.setPosition(RANGE, RANGE);
  if (PROBE_MODE != 2 && PROBE_MODE < 3) drv.park();

  Serial.begin(115200);
  Serial.println(F("XY4140 - automaticke hledani dratu"));
  if (PROBE_MODE == 2) {
    Serial.println(F("ODPOJENO: D2..D7 jsou vstupy, grafopstrojitar nedostava nic"));
    Serial.println(F("merejte kontakt na pripravku, zadny vystup tam neni"));
  } else if (PROBE_MODE == 6) {
    Serial.println(F("HLEDEJ LEVY KRAJ: kaletka jede doleva, cte se D7"));
    Serial.println(F("kontakt 6 na D4, kontakt 3 na D5 na L, kontakt 1 na D7"));
    Serial.println(F("zastaveni je na HOME_MAX krocich"));
  } else if (PROBE_MODE == 5) {
    Serial.println(F("SMER: LD a +/- se stridaji, za 4 s projde vsechny ctery kombinace"));
    Serial.println(F("kontakt 5 na D3, kontakt 6 na D4, +/- na D5, kontakt 1 na D6 a D7"));
    Serial.println(F("zapisuj, ktera kombinace jede papir vpred a ktera vzad"));
  } else if (PROBE_MODE == 4) {
    Serial.println(F("PERO: D6 zadost strida 4 s HIGH / 4 s LOW, D2 prepina po 1 s"));
    Serial.println(F("kontakt 1 na D6 a D7,PD kontakt na D2, +/- na D5"));
    Serial.println(F("sledujte pero i lampu PEN"));
  } else if (PROBE_MODE == 3) {
    Serial.println(F("ZKOUSKA PERA: hybe se jen D2, zbytek je v nule"));
    Serial.println(F("kontakt na D2: pero se zvedne = je to PEN"));
  } else if (PROBE_MODE) {
    Serial.println(F("MAJAK: D2 D3 D4 D5 jedou soucasne 1 s HIGH / 1 s LOW"));
    Serial.println(F("kontakt pripoj postupne na kazdy z pinu, ktery hybe = STEP"));
  } else {
    Serial.println(F("chod rozjizdi sam, tlacitka nepouzivat"));
    Serial.println(F("kazdy chod tam a zpet, do dorazu nedojede"));
    Serial.println(F("lapa READY: jen D7 na kontakt, D6 nechat odpojene"));
    Serial.println(F("pri hledani READY poslat m, zastavi chod"));
  }
  reportPins();
}

void loop() {
  // v odpojenem rezimu nesmi vychodovat zalozeny stav, jen blikat LED
  if (PROBE_MODE == 2) {
    digitalWrite(PIN_LED, HIGH);
    delay(500);
    digitalWrite(PIN_LED, LOW);
    delay(500);
    return;
  }

  // prohledani polarity zadosti a spoustece pera
  if (PROBE_MODE == 3 || PROBE_MODE == 4) {
    penSweep();
    return;
  }

  // smer podavani papiru: LD a +/- se stridaji
  if (PROBE_MODE == 5) {
    dirSweep();
    return;
  }

  // hledani leveho kraje kaletky, bezpecnostni zastaveni je HOME_MAX
  if (PROBE_MODE == 6) {
    homeSearch();
    return;
  }

  if (PROBE_MODE) {
    beacon();
    return;
  }

  // dotaz bezi vzdy, i kdyz je chod zastaveny, jinak by pri hledani
  // kontaktu nebylo videt zmenu na D7
  probeReady();

  if (autoMode && (int32_t)(millis() - nextAt) >= 0) {
    runLeg();
    legIndex = (legIndex + 1) & 3;
    nextAt = millis() + LEG_PAUSE;
  }
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 13 || c == 10) return;
  handle(c);
}