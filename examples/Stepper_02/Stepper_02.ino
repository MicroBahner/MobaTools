/*  Demo zum Anschluß eines unipolaren Stepmotors 28BYJ-48
 *  mit Verwendung einer Beschleunigungsrampe
 *      Danke an 'agmue' vom arduino.cc Forum für dieses Beispiel
*/
#include <MobaTools.h>
Stepper4 Step1(4096);           // HALFSTEP ist default

void setup() {
  Step1.attach( 4, 5, 6, 7 ); // Anschluß an digitalen Ausgängen; Treiber IN1,IN2,IN3,IN4
  Step1.setSpeed( 240 );      // = 24 U/Min
  Step1.setRampLen(500);      // Beschleunigung
  Step1.setZero();            // Referenzpunkt für Motor 1 setzen
}

void loop() {
  uint32_t jetzt = millis();        // aktuelle Zeit
  static uint32_t vorhin = jetzt;   // gemerkte Zeit
  static byte status;               // Schrittkettenstatus

  switch (status) {
    case 0:
      Step1.write(360);               // 1 Umdrehung vorwärts
      status++;
      break;
    case 1:
      if ( !Step1.moving() ) {          // warten bis die Bewegung abgeschlossen ist
        vorhin = jetzt;                 // Zeit merken
        status++;
      }
      break;
    case 2:
      if ( jetzt - vorhin >= 1000 ) {   // 1 Sekunde warten
        status++;
      }
      break;
    case 3:
      Step1.write(0);                 // 1 Umdrehung zurück
      status++;
      break;
    case 4:
      if ( !Step1.moving() ) {          // warten bis die Bewegung abgeschlossen ist
        vorhin = jetzt;                 // Zeit merken
        status++;
      }
      break;
    case 5:
      if ( jetzt - vorhin >= 1000 ) {   // 1 Sekunde warten
        status++;
      }
      break;
    default:
      status = 0;
  }
}
