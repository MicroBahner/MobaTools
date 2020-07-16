/*  Beispiel für die Ansteuerung eines bipolaren Steppers
   über 4 Taster und ein Poti ( für die Geschwindigkeit )
   In diesem Beispiel werden neben der Stepperklasse (MoToStepper), auch die MoToButtons
   und der MoToTicker genutzt.
   Jedem Taster ist eine Position zugeordnet, die bei einem kurzen Druck auf den Taster
   angefahren wird.
   Wird Taster4 lang gedrückt, wird eine Referenzfahrt ausgelöst.
   In diesem Beispiel ist die Referenzfahrt als einfache blockierende Funktion ausgeführt.
*/

#define MAX8BUTTONS // spart Speicher, da nur 4 Taster benötigt werden
#include <MobaTools.h>

const int STEPS_UMDREHUNG = 800;
//Stepper einrichten ( 800 Schritte / Umdrehung - 1/4 Microstep )
MoToStepper myStepper( STEPS_UMDREHUNG, STEPDIR );  // 400 Steps/ Umdrehung
const byte dirPin = 5;
const byte stepPin = 6;
const byte enaPin = 7;

// Taster einrichten ( die Taster müssen gegen Gnd schalten, keine Widerstände notwendig )
enum { Taster1, Taster2, Taster3, Taster4 } ; // Den Tasternamen die Indizes 0...3 zuordnen
const byte tasterPins[] = {A1, A2, A3, A4 }; // muss als byte definiert sein, damit ein enfaches sizeof funktioniert
const byte tasterZahl = sizeof(tasterPins);
const long tasterPos[] = { 1600, 3200, 6400, 7600 };
MoToButtons taster( tasterPins, tasterZahl, 20, 500 );

// Referenzschalter
const byte refPin = A5;         // Input für Referenzpunktschalter
const byte atRefpoint = HIGH;   // Einganspegel, wenn Refpunkt erreicht

MoToTicker speedIntervall;      // Zeitinterval zum Auslesen des Speedpotentiometers

const byte potiPin = A0;        //Poti fuer Geschwindigkeit
int vspeed = 0;                 //Steppergeschwindigkeit in U/min*10
int oldSpeed = 0;               // Zur Erkennung von Geschwindigkeitsänderungen

void toRefPoint() {
  // Stepper zum Referenzpunkt bewegen, und Position auf 0 setzen
  Serial.println("Referenzpunkt anfahren");
  // Im Schnellgang Richtung Refpunkt fahren ...
  myStepper.setSpeedSteps( 20000, 100 );
  myStepper.rotate(-1);
  while ( digitalRead( refPin ) != atRefpoint );
  digitalWrite( LED_BUILTIN, digitalRead( refPin ) );
  // Refschalter erreicht, anhalten
  myStepper.rotate(0);
  while ( myStepper.moving() );     // Brensrampe abwarten;
  // Langsam zurück zum Schaltpunkt des Refpunktes fahren
  myStepper.setSpeedSteps( 1000 );
  myStepper.setRampLen(0);
  myStepper.rotate( 1 );
  while ( digitalRead( refPin ) == atRefpoint );
  digitalWrite( LED_BUILTIN, digitalRead( refPin ) );
  Serial.println("Referenzpunkt erreicht");
  myStepper.rotate(0);
  while (myStepper.moving() );
  myStepper.setZero();
  myStepper.setSpeed( 200 );
  oldSpeed = 0;
  myStepper.setRampLen( 100 );                       // Rampenlänge 100 Steps bei 20U/min
  Serial.println("Ende Reffahrt");
}

void setup()
{ Serial.begin(115200); while (!Serial);
  myStepper.attach( stepPin, dirPin );
  myStepper.attachEnable( enaPin, 10, LOW );        // Enable Pin aktivieren ( LOW=aktiv )
  myStepper.setSpeed( 200 );
  vspeed = 200;
  myStepper.setRampLen( 100 );                       // Rampenlänge 100 Steps bei 20U/min
  speedIntervall.setTicker( 100 );                  // 100ms Tickerzeit
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(refPin, INPUT_PULLUP );
  toRefPoint();
}

void loop() {
  taster.processButtons();          // Taster einlesen und bearbeiten

  digitalWrite( LED_BUILTIN, digitalRead( refPin ) );

  // Speed alle 100ms neu einlesen und setzen
  if ( speedIntervall.ticker() ) {
    // wird alle 100ms aufgerufen ( Tickerzeit = 100ms im setup() )
    vspeed = map((analogRead(potiPin)), 0, 1023, 20, 1800);  //Poti mappen auf 2 ... 180 Umdr/Min
    //min speed =2 and max speed =180 rpm
    if ( abs( oldSpeed - vspeed ) > 5 ) {
      myStepper.setSpeed( vspeed );
      oldSpeed = vspeed;
    }
  }

  // Referenzfahrt auslösen
  if ( taster.longPress( Taster4 ) ) toRefPoint();

  //
  for ( byte tastIx = 0; tastIx < tasterZahl; tastIx++ ) {
    // die 4 Taster auf kurzen Druck abfragen
    byte clickTyp = taster.clicked(tastIx);
    if ( clickTyp == SINGLECLICK ) {
      //Taste wurde gedrückt
      Serial.print("Fahre zu Pos "); Serial.println( tasterPos[tastIx] );
      myStepper.writeSteps(tasterPos[tastIx]);
    } else if ( clickTyp == DOUBLECLICK ) {
      switch ( tastIx ) {
        case Taster1:
          myStepper.rotate(1);
          break;
        case Taster2:
          myStepper.rotate(-1);
          break;
        case Taster3:
          myStepper.rotate(0);
          break;
        default:
          ;
      }
    }
  }
}
