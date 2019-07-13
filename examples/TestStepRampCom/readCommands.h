

bool getCmd ( eeBefehl_t &cmdBuf ) {
    // Wenn Daten verfügbar, diese in den receive-Buffer lesen. Endezeichen ist LF oder CR
    // Funktionswert ist true, wenn ein kompletter Befehl empfangen, und im cmdBuf eingetragen wurde
    bool fulCmd = false;
    byte eeIx = -1;         // default: kein EEPROM Eintrag
    static char rcvBuf[50];
    static byte rcvIx=0;       // Index im Empfangspuffer
    char *token, *tkPtr;
    const char trenner[] = " ,\n\r";
    byte dataAnz = Serial.available();
    if ( dataAnz > 0 ) {
        Serial.readBytes( &rcvBuf[rcvIx], dataAnz );
        rcvIx += dataAnz;
        if ( rcvBuf[rcvIx-1] == 10 || rcvBuf[rcvIx-1] == 13 ) {
            rcvBuf[rcvIx-1] = 0;
            // komplette Zeile empfangen -> auswerten und in cmdBuf eintragen
            fulCmd = true;
            cmdBuf.bedingung = '-';
            token = strtok( rcvBuf, "\n\r ,");
            tkPtr = strstr( comStr, token );
            if ( tkPtr == NULL ) {
                // unbekanntes Kommando
                Serial.println("Kommando unbekannt");
                fulCmd = false;
            } else { // === gültiges Kommando empfangen ===
                // token berechnen
                cmdBuf.command = (comTok)( (int)(tkPtr-comStr) / 4 );
                // Prüfen, ob der Befehl ins EEPROM soll
               if ( cmdBuf.command == eepT ) { 
                    // EEprom-Index lesen
                    eeIx = atoi( strtok( NULL, trenner ) );
                    // Ausführungsbedingung einlesen
                    token = strtok( NULL, trenner );
                    cmdBuf.bedingung = *token;
                    cmdBuf.bedParam  = atol( strtok( NULL, trenner) );
                    token = strtok( NULL, trenner ); // ab hier Befehlsauswertung
                    tkPtr = strstr( comStr, token );
                    if ( tkPtr == NULL ) {
                        // unbekanntes Kommando, wie nop behandeln
                        cmdBuf.command = nopT;
                    } else {
                         cmdBuf.command = (comTok)( (int)(tkPtr-comStr) / 4 ); 
                    }
                }
                // Parameter einlesen
                cmdBuf.comPar1 = atol( strtok( NULL, trenner ) );
                token = strtok( NULL, trenner );
                if ( strlen( token ) == 0 ) {
                    cmdBuf.comPar2 = -1;
                } else {
                    cmdBuf.comPar2 = atoi( token );
                }
                 
                switch ( cmdBuf.command )  {     
                  case estT: // ===============================  est nnn     -> Automat an Index nn starten ======
                    comIx = cmdBuf.comPar1;
                    if ( comIx >= 0 && comIx <32 ) autoZustand = NEXTCOM;
                    printf( "Starte autom. Ablauf ab Index %d\n\r", comIx );
                    fulCmd = false;
                    break;
                  case espT: // ===============================  esp     -> Automat stoppen ======
                    autoZustand = ASTOPPED;
                    fulCmd = false;
                    break;
                  case elsT: // ===============================  els         -> list Commands ======================
                    // Kommandos aus EEPROM ausgeben ( ab 0 bis ungültiger Eintrag )
                    Serial.println();
                    eeIx = 0;
                    readCmd( eeIx, cmdBuf  );
                    while (eeIx < 32 ) {
                        if ( (strchr( "-<>mt!", cmdBuf.bedingung ) != NULL) && cmdBuf.bedingung != 0  ) {
                            // nur gültioge Einträge anzeigen
                            printf( "%02d: ", eeIx );
                            printEeBefehl( cmdBuf );
                        }
                        readCmd( ++eeIx, cmdBuf );
                    }
                    fulCmd = false; // Kommando nicht weiterleiten
                    break;
                  default: // alle regulären Kommandos brauchen keine weitere Verarbeitung
                    ;
                }
                // ------------- war eep-Befehl -> Kommando im Speicher eintragen --------------
                if ( fulCmd && eeIx >=0 && eeIx < 32 ) {
                    // Kommando in EEPROM eintragen
                    printf( "EEPROM-Ix = %d, ", eeIx );
                    storeCmd( eeIx, cmdBuf );
                    readCmd( eeIx, cmdBuf );
                    printEeBefehl( cmdBuf );
                    fulCmd = false; // Kommando nicht direkt ausführen
                }
            } // Ende Kommandoauswertung
            // Empfangspuffer rücksetzen
            rcvIx = 0;
        } // Ende komplette Zeile empfagen und auswerten
    }
    return fulCmd;
}

void execCmd( eeBefehl_t &cmdBuf ) {
    long steps;
    uint16_t ramp;
    switch ( cmdBuf.command ) {
      case szpT:
        myStepper.setZero();
        break;
      case dstT: // ===============================  dst nnn     -> doSteps( +/-nnnL ) ==========
        printf(" Move: %ld\n\r",cmdBuf.comPar1 );
        myStepper.doSteps( cmdBuf.comPar1 );
        break;
      case wraT: // ===============================  wra nnn     -> write( +/-angleL ) ===========
        printf(" MoveTo: %ld\n\r",cmdBuf.comPar1 );
        myStepper.write( cmdBuf.comPar1 );
        break;
      case wrsT: // ===============================  wrs nnn     -> writeSteps ( +/-stepsL ) ======
        printf(" MoveTo: %ld\n\r",cmdBuf.comPar1 );
        myStepper.writeSteps( cmdBuf.comPar1 );
        break;
      case wrpT: // ===============================  wrp        -> writeSteps ( readStreps() ) ======
        steps = myStepper.readSteps();
        printf(" MoveTo: %ld\n\r",steps );
        myStepper.writeSteps( steps );
        break;
      case rotT: // ===============================  rot d       -> rotate( +/-direction ); =======
        printf(" rotate: %d\n\r",cmdBuf.comPar1  );
        myStepper.rotate( cmdBuf.comPar1  );
        break;
      case sspT: // ===============================  ssp nn      -> setSpeed( rpm10 ) ===============================
        printf(" Rpm: %u,%u ", cmdBuf.comPar1 /10, cmdBuf.comPar1 %10 );
        myStepper.setSpeed( cmdBuf.comPar1  );
        break;
      case sssT: // ===============================  sss nn rr   -> setSpeedSteps( speed10, ramp ) wenn rr 0 ist oder fehlt wird es nicht ausgegeben
        if ( cmdBuf.comPar2 < 0 ) {
            printf(" sss: %u,%u",cmdBuf.comPar1 /10, cmdBuf.comPar1 %10);
            ramp = myStepper.setSpeedSteps( cmdBuf.comPar1  );
        } else {
            printf(" sss: %u,%u, ramp: %u",cmdBuf.comPar1 /10, cmdBuf.comPar1 %10, cmdBuf.comPar2 );
            ramp = myStepper.setSpeedSteps( cmdBuf.comPar1 , cmdBuf.comPar2  );
        }
        printf( "->akt.Rampe=%u\n\r", ramp );
        break;
     case stpT: // ===============================  stp         -> stop ===============================
        Serial.println( " Stop!" );
        myStepper.stop();
        break;
     case srlT: // ===============================  srl nn      -> setRampLen( ramp ) ================
        ramp = myStepper.setRampLen( cmdBuf.comPar1  );
        printf( " akt.Rampe=%u\n\r", ramp );
        break;
     case movT: // ===============================  mov         -> print restweg ======================
        printf( " Remaining: %d\n\r", myStepper.moving() );
        break;
     case rdaT: // ===============================  rda         -> print anglepos =====================
       printf( " Winkelposition: %ld\n\r", myStepper.read() );
       break;
     case rdsT: // ===============================  rds         -> print steppos ======================
       printf( " Stepposition: %ld\n\r", myStepper.readSteps() );
       break;
     case nopT:
       Serial.println( " NOP" );
       break;
     default:
       Serial.println("Kommando unbekannt");
    }
    
}
