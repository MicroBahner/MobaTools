// Blink 4 leds in random intervals with the ticker class.
// Blinking starts with first press of the correspondig button 
// and can be stopped and restarted by pressing this button.
// The blink frequency is determined at the first start.

#include <MobaTools.h>

const byte ledPins[] = {2,3,4,5};
const byte buttonPins[] = { A0,A1,A2,A3 };
const byte ledCnt = sizeof( ledPins );      // ledPins must be type of byte
MoToTicker ledTicker[ledCnt];
MoToButtons Buttons( buttonPins, ledCnt, 20, 500 );

void setup() {
    // Set ledPins to OUTPUT. The mode of the button pins is set
    // by the MoToButtons constructor.
    for ( auto pin : ledPins  ) {
        pinMode( pin, OUTPUT );
        digitalWrite( pin, HIGH );  // initial led test 
        delay(500);
        digitalWrite( pin, LOW );
    }
}

void loop() {
    Buttons.processButtons();
    
    for ( byte i=0; i<ledCnt; i++ ) {
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
