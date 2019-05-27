char txtbuf[80];
#define printf( x, ... ) { sprintf_P( txtbuf, (const char*) PSTR( x ), ##__VA_ARGS__ ) ; Serial.print( txtbuf ); }

/* Steuerung der Methodenaufrufe per serieller Schnittstelle
 *  dst nnn     -> doSteps( +/-nnnL )
 *  wra nnn     -> write( +/-angleL )
 *  wrs nnn     -> writeSteps ( +/-stepsL )
 *  rot d       -> rotate( +/-direction );
 *  ssp nn      -> setSpeed( rpm10
 *  sss nn rr   -> setSpeedSteps( speed10, ramp ) wenn rr 0 ist oder fehlt wird es nicht ausgegeben
 *  srl nn      -> setRampLen( ramp )
 *  stp         -> stop
 *  mov         -> print restweg
 *  rda         -> print anglepos
 *  rds         -> print steppos
 *  szp         -> set zeropoint
 *  
*/
void getCmd ( void ) {
    // Wenn Daten verfügbar, diese in den receive-Buffer lesen. Endezeichen ist LF oder CR
    static char rcvBuf[20];
    static byte rcvIx=0;       // Index im Empfangspuffer
    char *token;
    long steps =0, angle = 0; 
    uint16_t wert;
    int16_t dwert;
    uint16_t  faktor;
    byte dataAnz = Serial.available();
    if ( dataAnz > 0 ) {
        //printf( "Daten verfügbar" );
        Serial.readBytes( &rcvBuf[rcvIx], dataAnz );
        rcvIx += dataAnz;
        if ( rcvBuf[rcvIx-1] == 10 || rcvBuf[rcvIx-1] == 13 ) {
            rcvBuf[rcvIx-1] = 0;
            // komplette Zeile empfangen -> auswerten
            token = strtok( rcvBuf, "\n\r ,");
            printf("Kommande: -%s- / ", token );
            // ===============================  szp         -> set zeropoint ======================
            if ( strcmp( token, "szp" ) == 0 ) { 
                Serial.print("Nullpunkt \n\r");
                myStepper.setZero();
            // ===============================  dst nnn     -> doSteps( +/-nnnL ) ==========
            } else if ( strcmp( token, "dst" ) == 0 ) {
                steps = atol( strtok( NULL, " ," ) );
                printf(" Move: %ld\n\r",steps ); delay( 500 );
                myStepper.doSteps( steps );
            // ===============================  wra nnn     -> write( +/-angleL ) ===========
            } else if ( strcmp( token, "wra" ) == 0 ) {
                steps = atol( strtok( NULL, " ," ) );
                printf(" MoveTo: %ld\n\r",steps );
                myStepper.write( steps );
            // ===============================  wrs nnn     -> writeSteps ( +/-stepsL ) ======
            } else if ( strcmp( token, "wrs" ) == 0 ) {
                steps = atol( strtok( NULL, " ," ) );
                printf(" MoveTo: %ld\n\r",steps );
                myStepper.writeSteps( steps );
            // ===============================  rot d       -> rotate( +/-direction ); =======
            } else if ( strcmp( token, "rot" ) == 0 ) {
                dwert = atoi( strtok( NULL, " ," ) );
                printf(" rotate: %d\n\r",dwert );
                myStepper.rotate( dwert );
            // ===============================  ssp nn      -> setSpeed( rpm10 ) ===============================
            } else if ( strcmp( token, "ssp" ) == 0 ) { 
                wert = atoi( strtok( NULL, " ," ) );
                printf("Rpm: %u,%u ", wert/10, wert%10 );
                myStepper.setRampLen( wert );
            // ===============================  sss nn rr   -> setSpeedSteps( speed10, ramp ) wenn rr 0 ist oder fehlt wird es nicht ausgegeben
            } else if ( strcmp( token, "sss" ) == 0 ) { 
                wert = atoi( strtok( NULL, " ," ) );
                faktor = atoi( strtok( NULL, " ," ) );
                if ( faktor == 0 ) {;
                    printf("sps: %d,%d\n\r",wert/10, wert%10);
                    myStepper.setSpeedSteps( wert );
                } else {
                    printf("sps: %d,%d, ramp: %d\n\r",wert/10, wert%10, faktor);
                    myStepper.setSpeedSteps( wert, faktor );
                }
            // ===============================  stp         -> stop ===============================
            } else if ( strcmp( token, "stp" ) == 0 ) { 
                wert = atoi( strtok( NULL, " ," ) );
                printf( "Stop!\n\r" );
                myStepper.stop();
            // ===============================  srl nn      -> setRampLen( ramp ) ================
            } else if ( strcmp( token, "srl" ) == 0 ) { 
                wert = atoi( strtok( NULL, " ," ) );
                Serial.print("Ramp: ");Serial.println( wert );
                myStepper.setRampLen( wert );
            // ===============================  mov         -> print restweg ======================
            } else if ( strcmp( token, "mov" ) == 0 ) { 
                printf( "Remaining: %d\n\r", myStepper.moving() );
            // ===============================  rda         -> print anglepos =====================
            } else if ( strcmp( token, "rda" ) == 0 ) { 
                printf( "Winkelposition: %ld\n\r", myStepper.read() );
            // ===============================  rds         -> print steppos ======================
            } else if ( strcmp( token, "rds" ) == 0 ) { 
                printf( "Stepposition: %ld\n\r", myStepper.readSteps() );
            } else {
                printf("Kommando unbekannt\n\r",0);
            }
        // Empfangspuffer rücksetzen
        rcvIx = 0;
        }
    }
}
