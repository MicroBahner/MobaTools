#define MAX8BUTTONS
#include <MobaTools.h>

const uint32_t BLENDZEIT = 800;
/*const byte taster = 2;
const byte LEDstart = 5;
const byte LEDstop = 6;
const byte LEDtaster = 7;*/
const byte taster = 8;
const byte LEDstart = 9;
const byte LEDstop = 10;
const byte LEDtaster = 11;
//const byte pinArray[] = {8, 9, 10, 11, 12, 13, A0, A1, A2, A3};
const byte pinArray[] = {0,1,2,3,4,5,6,7};
const byte pinCount = sizeof(pinArray);
uint32_t jetzt;
enum {LAUF, AUSSCHALTEN, AUS};
byte schritt = LAUF;

button_t getHW( void ) {    // User-Callback-Funktion für Tasterstatus
  return (button_t) !digitalRead(taster);
}

MoToButtons Taster( getHW, 20, 500 );
MoToSoftLed meineLeds[pinCount];

void setup() {
  //Serial.begin(115200);
  //while(!Serial);
  //Serial.println("Anfang");
  pinMode(taster, INPUT_PULLUP);
  pinMode(LEDstart, OUTPUT);
  pinMode(LEDstop, OUTPUT);
  pinMode(LEDtaster, OUTPUT);

  for (byte led = 0; led < pinCount; led++) {
    meineLeds[led].attach(pinArray[led]);
    meineLeds[led].riseTime( BLENDZEIT );    // Aufblendzeit in ms
  }
}

void loop() {
  Taster.processButtons();
  jetzt = millis();
  switch (schritt) {
    case LAUF: lauflicht();
      if (Taster.pressed(0)) {
        schritt = AUSSCHALTEN;
      }
      break;
    case AUSSCHALTEN:
      for (byte led = 0; led < pinCount; led++) {
        meineLeds[led].off();
      }
      schritt = AUS;
      break;
    case AUS:
      if (Taster.pressed(0)) {
        schritt = LAUF;
      }
      break;
    default:
      schritt = LAUF;
  }
}
void lauflicht() {
  static uint32_t vorhin = jetzt;
  static byte led = 0;
  static bool ein = true;

  if (jetzt - vorhin >= BLENDZEIT/8) {
    if (ein) {
      meineLeds[led].on();
    } else {
      meineLeds[led].off();
      led = (1 + led) % pinCount;
    }
    vorhin = jetzt;
    ein = !ein;
  }
}
