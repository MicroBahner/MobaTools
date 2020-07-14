// Beispiel für die Ansteuerung eines bipolaren Steppers
// über 4 Taster und ein Poti ( für die Geschwindigkeit )
// In diesem Beispiel werden neben der Stepperklasse (MoToStepper), auch die MoToButtons
// und der MoToTicker genutzt.

#define MAX8BUTTONS // spart Speicher, da nur 4 Taster benötigt werden
#include <MobaTools.h>

const int STEPS_UMDREHUNG = 800;
//Stepper einrichten ( 800 Schritte / Umdrehung - 1/4 Microstep )
MoToStepper myStepper( STEPS_UMDREHUNG, STEPDIR );  // 400 Steps/ Umdrehung
const byte dirPin = 5;
const byte stepPin = 6;
const byte enaPin = 7;

// Taster einrichten ( die Taster müssen gegen Gnd schalten, keine Widerstände notwendig )
// Taster1 auf Pin7  Dreht rechts, solange Taster gedrückt
// Taster2 auf Pin8  Dreht links, solange Taster gedrückt
// Taster3 auf Pin9  1 Umdrehung rechts
// Taster4 auf Pin9  1 Umdrehung links

enum { Taster1, Taster2, Taster3, Taster4, refPoint } ; // Den Tasternamen die Indizes 0...3 zuordnen
const byte tasterPins[] = {A1, A2, A3, A4, A5 }; // muss als byte definiert sein, damit ein enfaches sizeof funktioniert
MoToButtons taster( tasterPins, sizeof(tasterPins), 20, 500 );

MoToTicker speedIntervall;       // Zeitinterval zum Auslesen des Speedpotentiometers

bool atRefPoint  = false;        // Stepper steht am Endschalter ( Referenzpunkt )
const byte potiPin = A0;           //Poti fuer Geschwindigkeit
int vspeed = 0;                 //Steppergeschwindigkeit in U/min*10

void toRefPoint() {
  // Stepper zum Referenzpunkt bewegen, und Position auf 0 setzen
  // Der Endschlater wird direkt gelesen
  // Im Schnellgang Richtugn Refpunkt fahren
  myStepper.setSpeedSteps( 20000, 100 );
  myStepper.rotate(-1);
  while ( !digitalRead( tasterPins[refPoint] ) );
  digitalWrite( LED_BUILTIN, digitalRead( tasterPins[refPoint] ) );
  // Refschalter erreicht, anhalten
  myStepper.rotate(0);
  // Langsam zurück zum Schaltpunkt des Refpunktes fahren
  delay(500);
  myStepper.setSpeedSteps( 1000, 0 );
  myStepper.rotate( 1 );
  while ( digitalRead( tasterPins[refPoint] ) );
  digitalWrite( LED_BUILTIN, digitalRead( tasterPins[refPoint] ) );
  myStepper.stop();
  myStepper.setZero();
  delay(500);
  myStepper.setSpeed( 200 );
  myStepper.setRampLen( 100 );                       // Rampenlänge 100 Steps bei 20U/min

}

void setup()
{ Serial.begin(115200); while (!Serial);
  myStepper.attach( stepPin, dirPin );
  myStepper.attachEnable( enaPin, 10, LOW );        // Enable Pin aktivieren ( LOW=aktiv )
  myStepper.setSpeed( 200 );
  myStepper.setRampLen( 100 );                       // Rampenlänge 100 Steps bei 20U/min
  speedIntervall.setTicker( 100 );                  // 100ms Tickerzeit
  pinMode(LED_BUILTIN, OUTPUT);
  toRefPoint();
}

void loop() {
  taster.processButtons();          // Taster einlesen und bearbeiten

  digitalWrite( LED_BUILTIN, digitalRead( tasterPins[refPoint] ) );
  // Am Referenzpunkt immer stoppen
  if ( taster.released(refPoint) ) {
    myStepper.rotate(0);
    Serial.println("RefPunkt erreicht");
    atRefPoint = true;
  }

  if ( taster.state( refPoint ) ) atRefPoint = false;

  // Speed alle 100ms neu einlesen und setzen
  if ( speedIntervall.ticker() ) {
    // wird alle 100ms aufgerufen ( Tickerzeit = 100ms im setup() )
    vspeed = map((analogRead(potiPin)), 0, 1023, 20, 1800);  //Poti mappen auf 2 ... 180 Umdr/Min
    //min speed =2 and max speed =180 rpm
    myStepper.setSpeed( vspeed );
  }

  // Referenzfahrt auslösen
  if ( taster.longPress( Taster4 ) ) toRefPoint();

  // Drehen rechtsrum
  if (taster.pressed (Taster1) ) {
    //Taster1 gedrückt
    myStepper.rotate( 1 );          // Stepper dreht vorwärts
  }
  if ( taster.released(Taster1) ) {
    //Taster1 losgelassen
    myStepper.rotate(0);             // Stepper stoppt
  }

  //Drehen linksrum
  if (taster.pressed(Taster2) ) {
    //Taster2 gedrückt
    myStepper.rotate( -1 );         // Stepper dreht rückwärts
  }
  if ( taster.released(Taster2) ) {
    //Taster2 losgelassen
    myStepper.rotate(0);            // Stepper stoppt
  }

  // 1 Umdrehung rechts {
  if (taster.shortPress(Taster3) ) {
    //Taster3 wurde gedrückt
    myStepper.doSteps(STEPS_UMDREHUNG);
  }

  // 1 Umdrehung links
  if (taster.shortPress(Taster4) ) {
    //Taster4 wurde gedrückt
    myStepper.doSteps(-STEPS_UMDREHUNG);
  }
}
