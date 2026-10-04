// Graficka jednotka XY 4140 - ovladac rozhrani
// Konektor ma 7 mist. Overeno merenim na pripravku:
//   kontakt 2 = masa (zmereno multimetrem)
//   kontakt 5 = STEP (zmereno chodem: pri pripojeni na D3 pristroj jel)
// V klidu je na pety signalech (1, 3, 4, 5, 6) 5 V, takze rozhrani je
// 5V TTL a Nano se pripojuje primo, bez prevodniku. Kontakt 7 nezmeren.
// Poradi signalu PEN, X/Y a +/- zatim neurcene: nesoverena tabulka
// (berk76, kap. 3.2.2) je vyvracena, v obou jejich verzich byl na kontaktu 5
//   X/Y, ale ukazalo se, ze tam STEP. Zbytek se hleda sketchem test_wires.
// Smer citani ani oznaceni kontaktu 1 nejsou v zadnem dostupnem dokumentu.
//
// Arduino Nano ATmega328P 5V - pripojeni primo na 5V TTL, bez prevodniku
//   PEN   LOW = pero nahoru, HIGH = pero dolu
//   STEP  LOW = impulz (jeden krok), HIGH = pauza
//   X/Y   LOW = CART (osu Y), HIGH = PAPER (osu X)
//   +/-   LOW = smer A, HIGH = smer B
//   GND   spojena se zemnou plotteru
//
// READY: v cizi dokumentaci popisana jako handshake, ale na tomto kusu
//   nepotrebna - pristroj jede bez ni. D6 a D7 se proto nepripojuji.
//
// Rozmery: 1 krok = 0.1 mm, plocha 2600 x 1850 kroku (260 x 185 mm).

#ifndef XY4140_H
#define XY4140_H

#include <Arduino.h>

// Volitelny kratkany dotaz na preruseni, vola se pri krokovani.
// Vrati true, kdyz se ma pohyb zastavit. Slouzi pro tlacitko STOP,
// aby nebylo cekat na dokonceni dlouhe useky.
typedef bool (*XY4140AbortHook)();

class XY4140 {
public:
  static const int32_t MAX_X = 2600;
  static const int32_t MAX_Y = 1850;

  static const uint8_t AXIS_CART  = 0;
  static const uint8_t AXIS_PAPER = 1;

  // KODY VRACENE Z home(): >= 0 pocet kroku do koncoveho spinace,
  // -1 = uz na spinaci pri startu, -2 = spinac nereagoval v limitu.
  static const int32_t HOME_ALREADY = -1;
  static const int32_t HOME_FAILED  = -2;

  void begin(uint8_t penPin, uint8_t stepPin, uint8_t xyPin, uint8_t dirPin,
             uint8_t readyOutPin, uint8_t readyInPin);

  void setDirInvert(uint8_t invert);
  void setStepDelay(uint16_t us);
  void setPenSettle(uint16_t us);
  void setMagPen(uint8_t useMagnet);
  void setHomingBackoff(int32_t steps);
  void setHomingMaxSteps(int32_t steps);
  void setAbortHook(XY4140AbortHook hook);

  void idle();
  void park();
  bool isReady();
  int32_t home();

  void moveTo(int32_t x, int32_t y);
  void lineTo(int32_t x, int32_t y);

  // Nastavi souradnice bez pohybu. Slouzi pro prvnim krokem bez homingu,
  // kdy je vychozi poloha jen odhad a nesmi se dohodnout s nulou na kraji.
  void setPosition(int32_t x, int32_t y);

  void requestStop();
  void clearStop();
  bool stopRequested() const;

  int32_t positionX() const { return _x; }
  int32_t positionY() const { return _y; }

private:
  void _pulse();
  void _setAxis(uint8_t axis);
  void _setDir(uint8_t forward);
  void _stepAxis(uint8_t axis, uint8_t forward, int32_t count);
  void _clamp(int32_t &x, int32_t &y);
  bool _abort();

  uint8_t _pen, _step, _xy, _dir, _rdyOut, _rdyIn;
  uint8_t _dirInvert;
  uint8_t _magPen;
  uint16_t _stepDelay;
  uint16_t _penSettle;
  int32_t _homeBackoff;
  int32_t _homeMax;
  int32_t _x, _y;
  volatile uint8_t _stop;
  XY4140AbortHook _hook;
};

#endif
