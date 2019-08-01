#include <MobaTools.h>
/*  Demo zum Anschluß eines unipolaren Stepmotors 28BYJ-48
 *  Der Schrittmotor kann wahlweise direkt über 4 frei wählbare Pins, oder über die SPI-
 *  Schnittstelle und Schieberegister angeschlossen werden. 
 *  SPI belegt beim UNO oder nano die Pins 10(SS),11(MOSI) und 13(SCK)
 *  Pin 12 (MISO) wird zwar nicht genutzt, aber von der HW belegt
*/
Stepper4 Step1(4096);           // HALFSTEP ist default
Stepper4 Step2(2048,FULLSTEP);


void setup() {
    Step1.attach( 4,5,6,7 );    // Anschluß an digitalen Ausgängen
    Step2.attach( SPI_1 );      // an die SPI-Schnittstelle muss ein Schieberegister angschlossen werden
                                // 
    Step1.setSpeed( 60 );       // = 6 U/Min
    Step1.setRampLen( 512);     // = 1/4 Umdrehung
    Step2.setSpeed( 120 );      // = 12 U/Min
    Step2.rotate( -1 );         // Motor 2 dreht dauerhaft rückwärts
    Step1.setZero();            // Referenzpunkt für Motor 1 setzen
    Step1.write(360);           // 1 Umdrehung vorwärts
    while( Step1.moving() );    // warten bis die Bewegung abgeschlossen ist
    delay(1000);                // 1 Sec stillstehen
    Step1.write(0);             // 1 Umdrehung zurück zum Referenzpunkt
    while( Step1.moving() );    // warten bis Stillstand
    delay(1000);                // Motor steht 1 sec.
    Step1.doSteps(4096);        // 4096 Schritte vorwärts ( = auch 1 Umdrehung )
    while( Step1.moving() );
    delay(1000);
    Step1.write(0);             // 1 Umdrehung zurück zum Referenzpunkt ( auch bei doSteps wird die
    while( Step1.moving() );    // Position verfolgt
    delay(1000);                // Motor steht 1 sec.
    Step1.detach();             // Motor deaktivieren
    Step1.attach(7,6,5,4);      // mit umgekehrter Reihenfolge der Pins aktivieren
    Step1.doSteps(4096);        // 'vorwärts' ist jetzt andersherum!
    while( Step1.moving() );
    delay(1000);

}

void loop() {
}
