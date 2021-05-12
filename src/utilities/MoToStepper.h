#ifndef MOTOSTEPPER_H
#define MOTOSTEPPER_H
/*
  MobaTools.h - a library for model railroaders
  Author: fpm, fpm@mnet-mail.de
  Copyright (c) 2020 All right reserved.

  Definitions and declarations for the stepper part of MobaTools
*/

// defines for the stepper motor
#define NOSTEP      0   // invalid-flag
#define HALFSTEP    1
#define FULLSTEP    2
#define A4988       3   // using motordriver A4988
#define STEPDIR		3	// all motordrivers with a step und dir input ( same as A4988 )


// Output modes ( outarg in attach method )
#define NO_OUTPUT   0

#if defined PORTD && defined PORTB
#define PIN8_11     1
#define PIN4_7      2
#endif

#ifndef ESP8266
#define SPI_1           3
#define SPI_2           4
#define SPI_3           5
#define SPI_4           6
#define SINGLE_PINS     7
#endif
#define A4988_PINS      8


#define CYCLETICS       (CYCLETIME*TICS_PER_MICROSECOND)
#define MIN_START_CYCLES 4000/CYCLETIME  // 5ms min until first step if stepper is in stop
#define MIN_STEPTIME    (CYCLETIME * MIN_STEP_CYCLE) 
#ifdef IS_32BIT
#define MAXRAMPLEN      160000 
#else
#define MAXRAMPLEN      16000       // Do not change!
#endif

#ifdef __STM32Fx__
    void ISR_Stepper(void);
#endif

//some AccelStepper compatible method names ( maybe sligtly different in functionality
#define moveTo              writeSteps
#define move                doSteps
#define setMaxSpeed( speed) setSpeedSteps( speed*10 )
#define distanceToGo        stepsToDo
#define currentPosition     readSteps

/////////////////////////////////////////////////////////////////////////////////
// global stepper data ( used in ISR )
enum class rampStat:byte { INACTIVE, STOPPED, STOPPING, STARTING, CRUISING, LASTSTEP, RAMPACCEL, RAMPDECEL, SPEEDDECEL  };
// states from CRUISING and above mean that the motor is moving
typedef struct stepperData_t {
  struct stepperData_t *nextStepperDataP;    // chain pointer
  volatile uint32_t stepCnt;        // nmbr of steps to take
  uint32_t stepCnt2;                // nmbr of steps to take after automatic reverse
  volatile int8_t patternIx;    // Pattern-Index of actual Step (0-7)
  int8_t   patternIxInc;        // halfstep: +/-1, fullstep: +/-2, A4988 +1/-1  the sign defines direction
  #ifdef ESP8266
	// on 32Bit-platforms all time values are in µs
	uint32_t tCycSteps;           // µseconds per step ( target value of motorspeed  )
	volatile uint32_t aCycSteps;  // nµseconds per step ( actual motorspeed  )
	uint32_t cyctXramplen;        // precompiled  tCycSteps*(rampLen+RAMPOFFSET)
    uint16_t cycDelay;            // delay time: enable -> stepping
    boolean  dirChange;          // Flag: Dir has to be changed ( at falling edge )
  #else
	// on the other platforms the time values count in cycles.
    // On 32-bit processors cyclelength is 1 µsec, and there is no remainder
	uintxx_t tCycSteps;           // nbr of IRQ cycles per step ( target value of motorspeed  )
	volatile uintxx_t aCycSteps;           // nbr of IRQ cycles per step ( actual motorspeed  )
    #ifndef IS_32BIT
    // Remainder needed only on 8-Bit processors
	uint16_t tCycRemain;          // Remainder of division when computing tCycSteps
	uint16_t aCycRemain;          // accumulate tCycRemain when cruising
    #endif
	uintxx_t cyctXramplen;        // precompiled  tCycSteps*(rampLen+RAMPOFFSET)
    volatile uintxx_t cycCnt;     // counting cycles until cycStep
	uintxx_t cycDelay;            // delay time enable -> stepping
  #endif
  uintxx_t  stepRampLen;        // Length of ramp in steps
  uintxx_t  stepsInRamp;        // stepcounter within ramp ( counting from stop ( = 0 ): incrementing in startramp, decrementing in stopramp
  uintxx_t  deltaSteps;         // number of computed steps per real step in SPEEDDECEL
                                // max value is stepRampLen
  rampStat rampState;        // State of acceleration/deceleration
  volatile long stepsFromZero;  // distance from last reference point ( always as steps in HALFSTEP mode )
                                // in FULLSTEP mode this is twice the real step number
  uint8_t output  :6 ;             // PORTB(pin8-11), PORTD (pin4-7), SPI0,SPI1,SPI2,SPI3, SINGLE_PINS, A4988_PINS
  uint8_t delayActiv :1;        // enable delaytime is running
  uint8_t enable:1;             // true: enablePin=HIGH is active, false: enablePin=LOW is active
  uint8_t enablePin;            // define an enablePin, which is active while the stepper is moving (255: no pin defined)
  #ifdef FAST_PORTWRT
  portBits_t portPins[4];       // Outputpins as Portaddress and Bitmask for faster writing
  #else
  uint8_t pins[4];                 // Outputpins as Arduino numbers
  #endif
  uint8_t lastPattern;             // only changed pins are updated ( is faster )
} stepperData_t ;

typedef union { // used output channels as bit and uint8_t
      struct {
        uint8_t pin8_11 :1;
        uint8_t pin4_7  :1;
        uint8_t spi1    :1;
        uint8_t spi2    :1;
        uint8_t spi3    :1;
        uint8_t spi4    :1;
      };
      uint8_t outputs;
    
} outUsed_t;

//////////////////////////////////////////////////////////////////////////////
class MoToStepper
{
  private:
    static outUsed_t outputsUsed;
    static byte     _stepperCount;  // number of objects ( objectcounter )
    stepperData_t _stepperData;      // Variables that are used in IRQ
    uint8_t _stepperIx;              // Objectnumber ( 0 ... MAX_STEPPER )
    long stepsRev;                   // steps per full rotation
    uintxx_t _stepSpeed10;      	// speed in steps/10sec
    uintxx_t _lastRampLen ;         // last manually set ramplen
    uintxx_t _lastRampSpeed;        // speed when ramp was set manually
    long stepsToMove;               // from last point
    uint8_t stepMode;               // FULLSTEP or HALFSTEP
    
    long getSFZ();                  // get step-distance from last reference point
    long lastSFZ;                   // last read value ( for corrections in doSteps ) 
    void _doSteps(long count, bool absPos ); // rotate count steps. abs=true means it was called from write methods

    bool _chkRunning();             // check if stepper is running
    void initialize(long,uint8_t);
    uint16_t  _setRampValues();
    uint8_t attach(uint8_t outArg, uint8_t*  ); // internal attach function ( called by one of the public attach
  public:
    // don't allow copying and moving of Stepper objects
    MoToStepper &operator= (const MoToStepper & )    =delete;
    MoToStepper &operator= (MoToStepper && )       =delete;
    MoToStepper (const MoToStepper & )             =delete;
    MoToStepper (MoToStepper && )            =delete;

    //Constuctor:
    MoToStepper(long steps);            // steps per 360 degree in HALFSTEP mode or A4988 Mode on ESP
    MoToStepper(long steps, uint8_t mode ); // with ESP8266 only STEPDIR is allowed
	#ifndef ESP8266 				// there are no different modes with ESP8266
                                        // mode means A4988 ( Step/Dir), HALFSTEP or FULLSTEP
        //Methods                                
        uint8_t attach( uint8_t,uint8_t,uint8_t,uint8_t); //single pins definition for output
        uint8_t attach(uint8_t outArg);    // stepMode defaults to halfstep
    #endif
    uint8_t attach( uint8_t stepP, uint8_t dirP); // Port for step and direction in A4988 mode
                                    // returns 0 on failure
    void    attachEnable( uint8_t enableP, uint16_t delay, bool active ); // define an enable pin and the delay (ms) between enable and starting/stopping the motor. 
                                                                          // 'active' defines if the output is HIGH or LOW to activate the motirdriver.
    void detach();                  // detach from output, motor will not move anymore
    void write(long angle);         // specify the angle in degrees, mybe pos or neg. angle is
                                    // measured from last 'setZero' point
    void write(long angle, uint8_t factor);        // factor specifies resolution of parameter angle
                                    // e.g. 10 means, 'angle' is angle in .1 degrees
	void writeSteps( long stepPos );// Go to position stepPos steps from zeropoint
    void setZero();                 // actual position is set as 0 angle (zeropoint)
    void setZero( long zeroPoint);  // new zeropoint ist zeroPoint steps apart from actual position
    void setZero( long zeroPoint, long stepsPerRotation);  // beside zero point change steps per Rotation too
    int setSpeed(int rpm10 );       // Set movement speed, rpm*10
    uintxx_t setSpeedSteps( uintxx_t speed10 ); // set speed withput changing ramp, returns ramp length
    uintxx_t setSpeedSteps( uintxx_t speed10, int rampLen ); // set speed and ramp, returns ramp length
    uintxx_t setRampLen( uintxx_t rampLen ); // set new ramplen in steps without changing speed
    uintxx_t getSpeedSteps();		// returns actual speed in steps/10sec ( even in ramp )
    void doSteps(long count);       // rotate count steps. May be positive or negative
                                    // angle is updated internally, so the next call to 'write'
                                    // will move to the correct angle
    void rotate(int8_t direction ); // rotate endless until 'stop',
    void stop();                    // stops moving immediately
    long stepsToDo();               // remaining steps until target position
    uint8_t moving();               // returns the remaining way to the position last set with write() in
                                    // in percentage. '0' means, that the target positio is reached
                                    // 255 means the motor is rotating endlessly
    long read();                    // actual angle from zeropoint (setZero)
    long readSteps();               // actual distance to zeropoint in steps
    uint8_t attached();
    void prDynData();             // print actual Stepperdata
};

#endif