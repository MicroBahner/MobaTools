//#define debugTP
//#define debugPrint

#ifdef debugTP 
    #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
        #define MODE_TP1 DDRF |= (1<<2) //pinA2
        #define SET_TP1 PORTF |= (1<<2)
        #define CLR_TP1 PORTF &= ~(1<<2)
        #define MODE_TP2 DDRF |= (1<<3) //pinA3
        #define SET_TP2 PORTF |= (1<<3)
        #define CLR_TP2 PORTF &= ~(1<<3)
        #define MODE_TP3 DDRF |= (1<<4) //pinA4 
        #define SET_TP3 PORTF |= (1<<4) 
        #define CLR_TP3 PORTF &= ~(1<<4) 
        #define MODE_TP4 DDRF |= (1<<5) //pinA5 
        #define SET_TP4 PORTF |= (1<<5) 
        #define CLR_TP4 PORTF &= ~(1<<5) 
    #elif defined(__AVR_ATmega32U4__)
        #define MODE_TP1 DDRF |= (1<<4) //A3
        #define SET_TP1 PORTF |= (1<<4)
        #define CLR_TP1 PORTF &= ~(1<<4)
        #define MODE_TP2 DDRF |= (1<<5) //A2
        #define SET_TP2 PORTF |= (1<<5)
        #define CLR_TP2 PORTF &= ~(1<<5)
        #define MODE_TP3 DDRD |= (1<<3) //D1
        #define SET_TP3 PORTD |= (1<<3)
        #define CLR_TP3 PORTD &= ~(1<<3)
        #define MODE_TP4 DDRD |= (1<<2) //D0
        #define SET_TP4 PORTD |= (1<<2)
        #define CLR_TP4 PORTD &= ~(1<<2)
        /*#define MODE_TP3 
        #define SET_TP3 
        #define CLR_TP3 
        #define MODE_TP4 
        #define SET_TP4 
        #define CLR_TP4 */
    #elif defined(__AVR_ATmega328P__) 
        #define MODE_TP1 DDRC |= (1<<1) //A1
        #define SET_TP1 PORTC |= (1<<1)
        #define CLR_TP1 PORTC &= ~(1<<1)
        #define MODE_TP2 DDRC |= (1<<2) // A2
        #define SET_TP2 PORTC |= (1<<2)
        #define CLR_TP2 PORTC &= ~(1<<2)
        #define MODE_TP3 DDRC |= (1<<3) //A3
        #define SET_TP3 PORTC |= (1<<3) 
        #define CLR_TP3 PORTC &= ~(1<<3) 
        #define MODE_TP4 DDRC |= (1<<4) //A4 
        #define SET_TP4 PORTC |= (1<<4) 
        #define CLR_TP4 PORTC &= ~(1<<4) 
    #elif defined(__AVR_ATmega128__) ||defined(__AVR_ATmega1281__)||defined(__AVR_ATmega2561__)
    #elif defined (__SAM3X8E__)
        // Arduino Due
        #define MODE_TP1 pinMode( A1,OUTPUT )   // A1= PA24
        #define SET_TP1  REG_PIOA_SODR = (1<<24)
        #define CLR_TP1  REG_PIOA_CODR = (1<<24)
        #define MODE_TP2 pinMode( A2,OUTPUT )   // A2= PA23
        #define SET_TP2  REG_PIOA_SODR = (1<<23)
        #define CLR_TP2  REG_PIOA_CODR = (1<<23)
        #define MODE_TP3 pinMode( A3,OUTPUT )   // A3 = PA22
        #define SET_TP3  REG_PIOA_SODR = (1<<22)
        #define CLR_TP3  REG_PIOA_CODR = (1<<22)
        #define MODE_TP4 pinMode( A4,OUTPUT )   // A4 = PA6
        #define SET_TP4  REG_PIOA_SODR = (1<<6)
        #define CLR_TP4  REG_PIOA_CODR = (1<<6)
    #elif defined (__STM32F1__)
        // STM32F103... ( SPI2-Pins! pin 31-28 maple mini )
        #define MODE_TP1 pinMode( PB12,OUTPUT )   // TP1= PB12
        #define SET_TP1  gpio_write_bit( GPIOB,12, HIGH );
        #define CLR_TP1  gpio_write_bit( GPIOB,12, LOW );
        #define MODE_TP2 pinMode( PB13,OUTPUT )   // TP2= PB13
        #define SET_TP2  gpio_write_bit( GPIOB,13, HIGH );
        #define CLR_TP2  gpio_write_bit( GPIOB,13, LOW );
        #define MODE_TP3 pinMode( PB14,OUTPUT )   // TP3 = PB14
        #define SET_TP3  gpio_write_bit( GPIOB,14, HIGH );
        #define CLR_TP3  gpio_write_bit( GPIOB,14, LOW );
        #define MODE_TP4 pinMode( PB15,OUTPUT )   // TP4 = PB15
        #define SET_TP4  gpio_write_bit( GPIOB,15, HIGH );
        #define CLR_TP4  gpio_write_bit( GPIOB,15, LOW );
    #else
        #define MODE_TP1 DDRC |= (1<<3) //A3
        #define SET_TP1 PORTC |= (1<<3)
        #define CLR_TP1 PORTC &= ~(1<<3)
        #define MODE_TP2 DDRC |= (1<<2) // A2
        #define SET_TP2 PORTC |= (1<<2)
        #define CLR_TP2 PORTC &= ~(1<<2)
        #define MODE_TP3 
        #define SET_TP3 
        #define CLR_TP3 
        #define MODE_TP4 
        #define SET_TP4 
        #define CLR_TP4 
    #endif 
// special Testpoint for Servo-testing
    //#define SET_SV3     SET_TP3
    //#define CLR_SV3     CLR_TP3
    /*    #define SET_SV3 PORTC |= (1<<3)  //A3
        #define CLR_SV3 PORTC &= ~(1<<3) 
    #define SET_SV4     SET_TP4
    #define CLR_SV4     CLR_TP4*/
    #define SET_SV3
    #define CLR_SV3
    #define SET_SV4     
    #define CLR_SV4     

// switch off Testpoints temporarily
 /*   #undef  MODE_TP4
    #undef  SET_TP4 
    #undef  CLR_TP4 
    #define MODE_TP4 
    #define SET_TP4 
    #define CLR_TP4 
    //#undef  MODE_TP3
    //#define MODE_TP3 
    #undef  SET_TP3 
    #undef  CLR_TP3 
    #define SET_TP3 
    #define CLR_TP3 
    #undef  MODE_TP2 
    #undef  SET_TP2 
    #undef  CLR_TP2 
    #define MODE_TP2 
    #define SET_TP2 
    #define CLR_TP2 
    #undef  MODE_TP1 
    #undef  SET_TP1 
    #undef  CLR_TP1 
    #define MODE_TP1 
    #define SET_TP1 
    #define CLR_TP1 */
    
#else
    #define MODE_TP1 
    #define SET_TP1 
    #define CLR_TP1 
    #define MODE_TP2 
    #define SET_TP2 
    #define CLR_TP2 
    #define MODE_TP3 
    #define SET_TP3 
    #define CLR_TP3
    #define MODE_TP4 
    #define SET_TP4 
    #define CLR_TP4 
// special Testpoint for Servo-testing
    #define SET_SV3 
    #define CLR_SV3 
    #define SET_SV4 
    #define CLR_SV4 

#endif
        //#define MODE_TP3 pinMode( PB14,OUTPUT )   // TP3 = PB14
        //#define SET_SV3  gpio_write_bit( GPIOB,14, HIGH )
        //#define CLR_SV3  gpio_write_bit( GPIOB,14, LOW )
        //#define MODE_TP4 pinMode( PB15,OUTPUT )   // TP4 = PB15
        //#define TOG_TP4  gpio_toggle_bit( GPIOB,15)
        //#define TOG_TP2  gpio_toggle_bit( GPIOB,13)
        #define TOG_TP4
        #define TOG_TP2


#ifdef debugPrint
    #define DB_PRINT( x, ... ) { sprintf_P( dbgBuf, PSTR( x ), ##__VA_ARGS__ ) ; Serial.println( dbgBuf ); }
    static char dbgBuf[80];
    

     const char *rsC[] = { "INACTIVE", "STOPPED", "RAMPSTART", "RAMPACCEL", "CRUISING", "STARTDECEL", "RAMPDECEL", "SPEEDDECEL" };    
#else
    #define DB_PRINT( x, ... ) ;
#endif

