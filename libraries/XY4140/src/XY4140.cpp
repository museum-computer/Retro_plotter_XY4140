// Graficka jednotka XY 4140 - ovladac rozhrani

#include "XY4140.h"

const int32_t XY4140::MAX_X;
const int32_t XY4140::MAX_Y;
const uint8_t XY4140::AXIS_CART;
const uint8_t XY4140::AXIS_PAPER;
const int32_t XY4140::HOME_ALREADY;
const int32_t XY4140::HOME_FAILED;

void XY4140::begin(uint8_t penPin, uint8_t stepPin, uint8_t xyPin, uint8_t dirPin,
                   uint8_t readyOutPin, uint8_t readyInPin) {
  _pen = penPin;
  _step = stepPin;
  _xy = xyPin;
  _dir = dirPin;
  _rdyOut = readyOutPin;
  _rdyIn = readyInPin;

  _dirInvert = 0;
  _magPen = 1;
  _stepDelay = 1000;
  _penSettle = 10000;
  _homeBackoff = 50;
  _homeMax = 4000;
  _x = 0;
  _y = 0;
  _stop = 0;
  _hook = 0;

  // uroven nastavit jeste pred pinMode, jinak by vygeneroval falsny
  // startovni impulz na STEP a jediny vystupny prikaz na PIN.
  digitalWrite(_step, HIGH);
  digitalWrite(_pen, LOW);
  digitalWrite(_xy, LOW);
  digitalWrite(_dir, LOW);
  digitalWrite(_rdyOut, HIGH);

  pinMode(_pen, OUTPUT);
  pinMode(_step, OUTPUT);
  pinMode(_xy, OUTPUT);
  pinMode(_dir, OUTPUT);
  pinMode(_rdyOut, OUTPUT);
  pinMode(_rdyIn, INPUT);
}

void XY4140::setDirInvert(uint8_t invert)    { _dirInvert = invert ? 1 : 0; }
void XY4140::setStepDelay(uint16_t us)       { _stepDelay = us; }
void XY4140::setPenSettle(uint16_t us)        { _penSettle = us; }
void XY4140::setMagPen(uint8_t useMagnet)    { _magPen = useMagnet ? 1 : 0; }
void XY4140::setHomingBackoff(int32_t steps) { _homeBackoff = steps; }
void XY4140::setHomingMaxSteps(int32_t s)    { _homeMax = s; }
void XY4140::setAbortHook(XY4140AbortHook h) { _hook = h; }

// dotaz na preruseni pri krokovani, aby tlacitko STOP fungovalo i uprostred
// dlouhe useky a nebylo potreba cekat na jeji konec
bool XY4140::_abort() {
  if (_hook == 0) return false;
  if (!_hook()) return false;
  _stop = 1;
  return true;
}

void XY4140::requestStop() { _stop = 1; }
void XY4140::clearStop()   { _stop = 0; }
bool XY4140::stopRequested() const { return _stop != 0; }

void XY4140::idle() {
  digitalWrite(_step, HIGH);
  digitalWrite(_pen, LOW);
  digitalWrite(_xy, LOW);
  digitalWrite(_dir, LOW);
  digitalWrite(_rdyOut, HIGH);
}

void XY4140::park() {
  if (_magPen) {
    digitalWrite(_pen, LOW);
    delayMicroseconds(_penSettle);
  }
}

bool XY4140::isReady() {
  // zadost -> uvolneni -> plotter stahne linku LOW = ready
  digitalWrite(_rdyOut, LOW);
  delayMicroseconds(_stepDelay);
  digitalWrite(_rdyOut, HIGH);
  delayMicroseconds(_stepDelay);
  return digitalRead(_rdyIn) == LOW;
}

void XY4140::_pulse() {
  digitalWrite(_step, LOW);
  delayMicroseconds(_stepDelay);
  digitalWrite(_step, HIGH);
  delayMicroseconds(_stepDelay);
}

void XY4140::_setAxis(uint8_t axis) {
  digitalWrite(_xy, axis == AXIS_PAPER ? HIGH : LOW);
}

void XY4140::_setDir(uint8_t forward) {
  if (_dirInvert) forward = !forward;
  digitalWrite(_dir, forward ? HIGH : LOW);
}

void XY4140::_stepAxis(uint8_t axis, uint8_t forward, int32_t count) {
  // count je znamenkovy rozdil z moveTo. Bez tohoto prevodu se zpetny
  // pohyb zahodil, protoze count <= 0 znamenal "nic nechat".
  if (count < 0) {
    count = -count;
    forward = !forward;
  }
  if (count == 0) return;
  _setAxis(axis);
  _setDir(forward);
  for (int32_t i = 0; i < count; i++) {
    if (_stop) return;
    if ((i & 63) == 63 && _abort()) return;
    if (forward) {
      if (axis == AXIS_PAPER) _x++;
      else                    _y++;
    } else {
      if (axis == AXIS_PAPER) _x--;
      else                    _y--;
    }
    _pulse();
  }
}

void XY4140::_clamp(int32_t &x, int32_t &y) {
  if (x < 0) x = 0;
  if (y < 0) y = 0;
  if (x > MAX_X) x = MAX_X;
  if (y > MAX_Y) y = MAX_Y;
}

int32_t XY4140::home() {
  clearStop();
  park();

  // koncovy spinac je na ose CART (Y)
  _setAxis(AXIS_CART);
  _setDir(0);
  delayMicroseconds(_stepDelay);

  int32_t n = 0;
  if (!isReady()) {
    while (!isReady() && n < _homeMax && !_stop) {
      if ((n & 63) == 63 && _abort()) break;
      _y--;
      _pulse();
      n++;
    }
  }
  clearStop();

  if (n < _homeMax && !isReady()) return HOME_FAILED;

  // odrazit od spinace, jinak by zustal na dorazu
  // (opetovny pridavek _y by po prevodu znamenka v _stepAxis zdvojil)
  _stepAxis(AXIS_CART, 1, _homeBackoff);
  _x = 0;
  return n;
}

void XY4140::moveTo(int32_t tx, int32_t ty) {
  _clamp(tx, ty);
  if (tx == _x) {
    _stepAxis(AXIS_CART, _y < ty, ty - _y);
    return;
  }
  if (ty == _y) {
    _stepAxis(AXIS_PAPER, _x < tx, tx - _x);
    return;
  }
  _stepAxis(AXIS_CART, _y < ty, ty - _y);
  _stepAxis(AXIS_PAPER, _x < tx, tx - _x);
}

void XY4140::lineTo(int32_t tx, int32_t ty) {
  if (_magPen) {
    digitalWrite(_pen, HIGH);
    delayMicroseconds(_penSettle);
  }
  moveTo(tx, ty);
}

void XY4140::setPosition(int32_t x, int32_t y) {
  // zamerne bez clearStop a bez clampu: jen zaznam, kde sket veri ze je
  _x = x;
  _y = y;
}
