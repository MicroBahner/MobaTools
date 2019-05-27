#include <MobaTools.h>
Stepper4  myStepper(400, A4988 );
Stepper4 myStepper1( 2000 );
int target = 50;
#include "readCommands.h"
void setup() {
  Serial.begin( 115200 );
  while( !Serial ); 
  Serial.println("Programmstart");
  // put your setup code here, to run once:
  pinMode( 2, OUTPUT );
  pinMode( 3, OUTPUT );
  pinMode( 4, OUTPUT );
  digitalWrite( 4, HIGH );
  myStepper.attach( A1, A0 );
  myStepper.setSpeedSteps( 6000, 100 );
  digitalWrite( 4, LOW );
  digitalWrite( 4, HIGH );
  //myStepper.setAcceleration( 1500 );
  digitalWrite( 4, LOW );
  //myStepper.writeSteps( target );
  delay( 1000 );
  printf( "Starting loop" );
}

void loop() {
  // put your main code here, to run repeatedly:
  getCmd();
  digitalWrite( 4, HIGH );
  //myStepper.run();
  //delay( 1 );
  digitalWrite( 4, LOW );
  /* if ( myStepper.distanceToGo() == 0 ) {
    target = -target;
    myStepper.moveTo( target );
  }*/
  //delay(1 );
}
