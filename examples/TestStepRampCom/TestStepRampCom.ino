// Testsketch für Stepper-Rampe mit automatischer Befehlsfolge
// Steuerung der Stepper-Methoden per serieller Schnittstelle und per Abfolgen im EEPROM

/* Steuerung der Methodenaufrufe per serieller Schnittstelle
 *  dst nnn     -> doSteps( +/-nnnL )
 *  wra nnn     -> write( +/-angleL )
 *  wrs nnn     -> writeSteps ( +/-stepsL )
 *  rot d       -> rotate( +/-direction );
 *  ssp nn      -> setSpeed( rpm10
 *  sss nn rr   -> setSpeedSteps( speed10, ramp ) wenn rr fehlt wird es nicht ausgegeben
 *  srl nn      -> setRampLen( ramp )
 *  stp         -> stop
 *  mov         -> print Restweg in % vom Gesamtweg
 *  std         -> print Restweg in Steps
 *  rda         -> print anglepos
 *  rds         -> print steppos
 *  szp         -> set zeropoint
 *  wrp         -> aktueller Punkt als Zielpunkt ( mit Rampe! )
 *  
 *  est nn      -> Kommandos in EEPROM ab Befehl nn abarbeiten
 *  esp         -> Abbarbeitung der EEPROM-Komandos stoppen
 *  eep ii c pp ccc nn rr Kommando im EEPROM an Stelle ii ablegen ( 0 <= ii < EEMAX )
 *              c= '-' Kommando sofort ausführen
 *              c= '<' Ausführen, wenn abs. Position pp unterschritten ( in Steps vom Ref.Punkt )
 *              c= '>' Ausführen, wenn abs, Position pp überschritten ( in Steps vom Ref. Punkt )
 *              c= 'm' Ausführen, wenn Restweg unter pp fällt ( in % )
 *              c= 't' nach pp Millisekunden ausführen
 *              c= '?' Abarbeitung stoppen
 *              ccc nn rr = eines der obigen Kommandos
 *  els         -> Kommandos aus EEPROM auflisten
 *  
*/

#include <MobaTools.h>
#include <EEPROM.h>

Stepper4  myStepper(800, A4988 );

// Definition der Pins für verschiedene Platformen
//==========================================================================
#ifdef __STM32F1__  //===================== STM32F1 ========================
const byte A4988Step=PA2, A4988Dir=PA3 ;
// SPI1 = Pins MOSI=PA7, MISO=PA6, SCK=PA5, NSS=PA4
// LA-Pins: TP1=PB12, TP2=PB13, TP3= PB14,  TP4=BP15
//............................................................................
#elif defined __AVR_ATmega328P__ // ========- 328P ( Nano, Uno Mega ) ========--
const byte A4988Step=6, A4988Dir=5;
// SPI = Pins 10,11,12,13
// LA-Pins: TP1=A1, TP2=A2, TP3= A3,  TP4=A4
//............................................................................
#elif defined __AVR_ATmega32U4__ // ====- 32U4 ( Micro, Leonardo ) ================
const byte A4988Step=6, A4988Dir=5;
// SPI = Pins 14,15,16,17
// LA-Pins: TP1=A3, TP2=A2, TP3= D1,  TP4=D0
#endif
//====================================== Ende Pin-Definitionen ======================

#define printf( x, ... ) { char txtbuf[100]; sprintf_P( txtbuf, PSTR( x ), ##__VA_ARGS__ ) ; Serial.print( txtbuf ); }

// Tokens der Befehle
enum comTok { dstT, wraT, wrsT, rotT, sspT, sssT, srlT, stpT, movT, rdaT, rdsT, szpT, wrpT, estT, espT, eepT, elsT,nopT,stdT };
const char comStr[] = "dst,wra,wrs,rot,ssp,sss,srl,stp,mov,rda,rds,szp,wrp,est,esp,eep,els,nop,std";
// Befehlsstruktur im EEPROM
#define EEMAX   64 // Zahl der einträge im EEPROM
typedef struct {
    char bedingung = '?';     // Bedingung für die Ausführung des Befehls
    long bedParam;      // Paramter für die Bedingung
    byte command;       // Auszuführendes Kommando
    long comPar1;       // Kommandoparameter
    long comPar2;       // Kommandoparamter
    uint16_t dummy;     // für spätere Erweiterung
} eeBefehl_t;
const byte eeComLen = sizeof( eeBefehl_t );

eeBefehl_t autoCom, interCom;      // aktuell abzuarbeitende Kommandos ( automatisch, interaktiv )
#ifdef __STM32F1__
eeBefehl_t cmdStorage[EEMAX]; // Auf dem STM32 werdein die Befehle im Ram gespeichert
#endif
enum  { ASTOPPED, NEXTCOM, WAITGT, WAITLT, WAITMV, WAITTM } autoZustand;
EggTimer waitTimer;
int comIx = -1;

//--------------------- Funktionen --------------------------------------
void printEeBefehl ( eeBefehl_t &comline, int eeIx = -1 ) {
    char _cmdStr[4];
    strncpy( _cmdStr, &comStr[comline.command*4], 3 );
    _cmdStr[3] = 0;
    if ( eeIx >= 0 ) {
        // im eep-Format ausgeben ( kann per Copy/paste wieder als
        // Kommando verwendet werden
        printf("eep %2d %2c %5ld  %s %5ld %5ld " ,
            eeIx,
            comline.bedingung,
            comline.bedParam,
            _cmdStr,
            comline.comPar1,
            comline.comPar2   );
        Serial.println();
    } else {
        printf("| %2c %5ld | Cmd: %s %5ld %5ld|" ,
            comline.bedingung,
            comline.bedParam,
            _cmdStr,
            comline.comPar1,
            comline.comPar2   );
        Serial.println();
    }
}


void storeCmd( byte eeIx, eeBefehl_t &cmdBuf ) {
    #ifdef __STM32F1__ // beim STM32 im Ram speichern
    if ( eeIx >= 0 && eeIx < EEMAX ) {
        memcpy( &cmdStorage[eeIx], &cmdBuf, sizeof( eeBefehl_t ) );
    }
    #else
    EEPROM.put( eeIx*eeComLen, cmdBuf );
    #endif
}

void readCmd( byte eeIx, eeBefehl_t &cmdBuf ) {
    #ifdef __STM32F1__ // hier gibt es keinen get-Befehl, eine Stelle kann aber uint16 aufnehmen
    if ( eeIx >= 0 && eeIx < EEMAX ) {
        memcpy( &cmdBuf, &cmdStorage[eeIx], eeComLen );
    }
    #else
    EEPROM.get( eeIx*eeComLen, cmdBuf );
    #endif
    
}

#include "readCommands.h"


void setup() {
  Serial.begin( 115200 );
  while( !Serial ); 
  Serial.println("Programmstart");
  if (myStepper.attach( A4988Step, A4988Dir )  ) Serial.println("Attach A4988 OK"); else Serial.println("Attach A4988 NOK");

  myStepper.setSpeedSteps( 6000, 100 );
  delay( 500 );
  Serial.println( "Starting loop" );
}

void loop() {
    if ( getCmd( interCom ) ) {
        //printEeBefehl( interCom );
        Serial.print("Manu:");
        execCmd( interCom ) ;
    }

    // Automatische Befehlsabarbeitung aus EEPROM
    switch( autoZustand ) {
      case ASTOPPED:
        comIx = -1;
        break;
      case NEXTCOM:
         if ( comIx == -1 ){
            // Ablauf stoppen
            Serial.println("Auto: Angehalten");
            autoZustand = ASTOPPED;
         } else {
             readCmd( comIx++, autoCom );
             printf("Auto: (%2d) ", comIx-1 );
             if ( autoCom.bedingung == '-' ) {
                // Befehl sofort ausführen
                execCmd( autoCom );
             } else if ( autoCom.bedingung == '>' ) {
                // verzögerte Ausführung
                printf(" warte bis Steppos > %d ->", autoCom.bedParam );
                autoZustand = WAITGT;
             } else if ( autoCom.bedingung == '<' ) {
                printf(" warte bis Steppos < %d ->", autoCom.bedParam );
                autoZustand = WAITLT;
             } else if ( autoCom.bedingung == 'm' ) {
                printf( " warte bis Restweg <= %d%% ->", autoCom.bedParam );
                autoZustand = WAITMV;
             } else if ( autoCom.bedingung == 't' ) {
                printf(" warte %d ms ->", autoCom.bedParam );
                autoZustand = WAITTM;
                waitTimer.setTime( autoCom.bedParam );
             } else {
                //kein gültiger Eintrag, Ablaufende
                Serial.println(" Ende");
                autoZustand = ASTOPPED;
             }
        }
        break;
      case WAITGT: // warten auf Erfüllung der aktuellen Bedingung
        if ( myStepper.readSteps() > autoCom.bedParam ) {
            execCmd( autoCom );
            autoZustand = NEXTCOM;
        }
        break;
      case WAITLT: // warten auf Erfüllung der aktuellen Bedingung
        if ( myStepper.readSteps() < autoCom.bedParam ) {
            execCmd( autoCom );
            autoZustand = NEXTCOM;
        }
        break;
      case WAITMV: // warten auf Erfüllung der aktuellen Bedingung
        if ( myStepper.moving() <= autoCom.bedParam ) {
            execCmd( autoCom );
            autoZustand = NEXTCOM;
        }
        break;
      case WAITTM: // warten auf Erfüllung der aktuellen Bedingung
        if ( !waitTimer.running() ) {
            execCmd( autoCom );
            autoZustand = NEXTCOM;
        }
        break;
    } //Ende Kommandoautomat
}
