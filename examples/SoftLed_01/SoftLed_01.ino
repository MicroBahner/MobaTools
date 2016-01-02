#include <MobaTools.h>
/* Demo: weiches Auf/Abblenden einer LED
 *  Mit der Klasse SoftLed lässt sich eine Led 'weich' ein- und ausschalten
 * Die Led muss an einem PWM-fähigen pin angeschlossen sein, d.h. an einem Pin
 * an dem auch analogWrite() funktioniert.
 * SoftLed.attach( byte Pinnr );// Anschlußpin für die Led festlegen
 * SoftLed.riseTime( int zeit );// Zeit für auf/abblenden in ms
 * SoftLed.on();                // 'weich' einschalten
 * SoftLed.off();               // 'weich' ausschalten
*/

const int ledPin =  5;  // PWM - fähiger Pin


SoftLed meinLed;

void setup() {
    meinLed.attach(ledPin); 
    meinLed.riseTime( 500 );    // Aufblendzeit in ms
}

void loop() {
	meinLed.on();
	delay(1000);
	meinLed.off();
	delay(1000);
}