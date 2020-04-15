// Blink 4 leds in random intervals by means of 
// the ticker function
// Blinking starts with first press of the correspondig button 
// and can be started and stopped by means of this button

#include <MobaTools.h>

const byte ledPins[] = {2,3,4,5};
const byte buttonPins[] = { A0,A1,A2,A3 };
MoToTicker ledTicker[4];
MoToButtons Buttons( buttonPins, 4, 20, 500 );

void setup() {
    // put your setup code here, to run once:
    //for ( byte i=0; i<4; i++ ) {
    for ( auto pin : ledPins  ) {
        pinMode( pin, OUTPUT );
    }
}

void loop() {
    // put your main code here, to run repeatedly:
    Buttons.processButtons();
    
    for ( byte i=0; i<4; i++ ) {
        if ( ledTicker[i].ticker() ) {
            digitalWrite( ledPins[i], !digitalRead( ledPins[i] ) );
        }
        if ( Buttons.pressed(i) ) {
            if ( ledTicker[i].inactive() ) {
                // first Start defines interval
                randomSeed( micros() );
                ledTicker[i].setTicker( random( 100, 2000 ) ); 
                digitalWrite( ledPins[i], HIGH );
            } else {
                if ( ledTicker[i].running() ) {
                    ledTicker[i].stop();
                    digitalWrite( ledPins[i], LOW );
                } else {
                    ledTicker[i].start();
                    digitalWrite( ledPins[i], HIGH );
                }
            }
        }
    }
}
