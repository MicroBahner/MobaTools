/* MoToButtons - a Arduino library to manage up to 8, 16 or 32 buttons/Switches 
  with debouncing, edge detection and long/short press detection.
  Author: fpm, fpm-gh@mnet-mail.de
  Copyright (c) 2020 All right reserved.
  This Library is part of MoBaTools

 Default is managing up to 16 buttons/switches.
 The default can be changed to save RAM (up to 8 buttons) or to manage up to 32 buttons (with additional RAM consumption). 
 This can be achieved by inserting '#define MAX32BUTTONS' or '#define MAX8BUTTONS'  before the #include <MoToButtons.h>.
  
 Reading the hardware state of the buttons is done by a usercallback function. 
 This enables designs where the buttons/switches are arranged in a matrix and/or read via a port extender.
 The return value of this function has to be a 8-Bit, 16-Bit or 32-Bit value according to the maximum manageable 
 number of buttons. Every button/switch is represented by one bit, where '1' means the button is pressed.

 'button_t' is automtically set to the correct type and can be used to define the type of the callback function.
 

  Constructor parameters:
    button_t (*getHWbuttons)()  Adress of the userfuction that reads the state of the buttons
    debTime                     Debouncetime in ms
    pressTime                   (in ms ) If the button is pressed longer, it is a 'long press'
                                max presstime = debTime*255
    doubleClick                 max time between two presses to be recognized as dounle click
                                ( optional, default: 2*pressTime )
    MoToButtons( button_t (*getHWbuttons)(), uint8_t debTime, uint16_t pressTime );
    example in sketch: 
        MoToButtons Buttons( readFunction, 20, 500 );
    The first parameter can alternativley by a reference to an array with pin numbers. In this case the state of the pins 
    is read by the class itself. 'LOW' at the pins means 'pressed':
    MoToButtons( uint8_t &pinNumbers[], uint8_t debTime, uint16_t pressTime );
    
  Methods to be called in loop:
    void    processButtons();                   // must be called in the loop frequently
                                                // if it is called less frequently than debTime, pressTime will be inaccurate
    Reading the debounced state of the Buttons/Switches:                                          
      bool state( uint8_t buttonNbr );       // get static state of button (debounced)
      button_t allStates();                     // bit field of all buttons (debounced)
      button_t changed();                       // all bits are set where state has changed since last call
  
    Reading events:
      bool shortPress( uint8_t buttonNbr );  // true if button was pressed short ( set when button is released, reset after call )  
      bool longPress( uint8_t buttonNbr );   // true if button was pressed long ( set when button is released, reset after call )  
      bool pressed( uint8_t buttonNbr );     // true if button is pressed ( reset after call )
      bool released( uint8_t buttonNbr );    // true if button is released ( reset after call )
      bool doubleClick( uint8_t buttonNbr ); // true if a double click was detected ( reset after call )
  
    void forceChanged(){                        // force all changed with call of next 'pressed', 'released' ore 'changed'
    void resetChanged(){                        // clear alle events of 'pressed', 'released' or 'changed'
                                                 // ( longPress() and shortPress() are unaffected )
   
    Event bits are set at the corresponding edge and they are cleared 
    when read or at the next inverted edge ( pressed-> released or vice versa )

    buttonNbr is a value from 0 to max buttons-1.
 */
 /*
   This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef MOTOBUTTONS_H
#define MOTOBUTTONS_H

#include <Arduino.h>

#ifdef BUTTON_CNT 
    #if BUTTON_CNT>32
        #error "too much buttons"
    #elif BUTTON_CNT>16
        #define MAX32BUTTONS
        #warning "Please use MAX32BUTTONS instead of BUTTON_CNT"
    #elif BUTTON_CNT>8
        #define MAX16BUTTONS
        #warning "Please use MAX16BUTTONS instead of BUTTON_CNT or leave it out completely"
    #else
        #define MAX8BUTTONS
        #warning "Please use MAX8BUTTONS instead of BUTTON_CNT"
    #endif
#endif

#ifdef MAX32BUTTONS
typedef uint32_t button_t;
#elif defined MAX8BUTTONS
typedef uint8_t button_t;
#else
typedef uint16_t button_t;
#endif


class MoToButtons {
  public:
    MoToButtons( uint8_t (&pinNumbers)[], uint8_t debTime, uint16_t pressTime, uint16_t doubleClick = (300 ) );
      _pinCnt = sizeof( pinMumbers );
      _pinArray = &pinNumbers;
      _getHWbuttons = getPinStates;
      _debTime = debTime;
      _pressTime = pressTime / debTime;   // in debTime tics
      _dClickTime = doubleClick / debTime;
      _lastReadTime = 0;     // Last time HW state was read
      // Bit fields to hold various button states
      _lastState = 0;
      _lastChanged = 0;
      _actState = 0;
      _longPress = 0;
      _shortPress = 0;
      _leadingEdge = 0;
      _trailingEdge = 0;
      for ( byte i = 0; i < _buttonCnt; i++ ) {
        _buttonTime[ i ] = 0; // Time in debounce tics
      }
    }

    
    MoToButtons( button_t (*getHWbuttons)(), uint8_t debTime, uint16_t pressTime, uint16_t doubleClick = (300 ) ) {
      _getHWbuttons = getHWbuttons;
      _debTime = debTime;
      _pressTime = pressTime / debTime;   // in debTime tics
      _dClickTime = doubleClick / debTime;
      _lastReadTime = 0;     // Last time HW state was read
      // Bit fields to hold various button states
      _lastState = 0;
      _lastChanged = 0;
      _actState = 0;
      _longPress = 0;
      _shortPress = 0;
      _leadingEdge = 0;
      _trailingEdge = 0;
      for ( byte i = 0; i < _buttonCnt; i++ ) {
        _buttonTime[ i ] = 0; // Time in debounce tics
      }
    }
    
    button_t getPinStates() {
      // read pins to get the HW state of the buttons
      
    }
    void processButtons() {
      // must be called in loop frequently
      if ( millis() - _lastReadTime > (uint32_t) _debTime ) {
        _lastReadTime = millis();
        _actState = _getHWbuttons();    // read button states
        // edge detection - new detected edges are added to the corresponding bit field
        // edge bits are cleard when read or at the next inverted edge ( pressed-> released or vice versa )
        // leading edge
        _leadingEdge &= _actState;  // clear bits if button is no longer pressed
        button_t pressEvent  = ~_lastState & _actState;     // new press event 
        _leadingEdge = pressEvent | _leadingEdge;
        // trailing edge
        _trailingEdge &= ~_actState;  // clear bits if button is pressed again
        _trailingEdge = (_lastState & ~_actState) | _trailingEdge ;

        _lastState = _actState;
        // process pressing time
        for ( byte i = 0; i < _buttonCnt; i++ ) {
          if ( _buttonTime[i] < 255 ) _buttonTime[i]++;
          if ( bitRead( pressEvent, i ) ) {
            // button is pressed, check doubleClick and reset time counter
            if ( _buttonTime[i] < _dClickTime ) {
              // Time since last pressed  is short -> it's a double click
              bitSet( _doubleClick, i );
              } else {
                // time too long, no double click
                bitClear( _doubleClick, i );
              }
            bitClear( _longPress, i );
            bitClear( _shortPress, i );
            _buttonTime[i] = 0;
          } else if ( bitRead( _trailingEdge, i ) ) {
            // button was released, check if it was presssd long or short
            if ( _buttonTime[i] > 0 ) { // check only once after releasing
              if (_buttonTime[i] < _pressTime) bitSet( _shortPress, i );
              else bitSet( _longPress, i );
              _buttonTime[i] = 0;
            }
          }
        }
      }
    }

    bool state( uint8_t buttonNbr ) {            // get static state of button (debounced)
      if ( buttonNbr >= _buttonCnt ) return 0;
      return bitRead( _actState, buttonNbr );
    }

    button_t allStates() {                          // bit field of all buttons (debounced)
       return ( _actState );
    }
    
    button_t changed(){                             // all bits are set where state is different from last call
      button_t temp = _actState ^ _lastChanged;
      _lastChanged = _actState;
      return temp;
    }
    
    void forceChanged(){                             // force all changed with call of next 'pressed', 'released' ore 'changed'
      _lastChanged = ~_actState;
      _leadingEdge = _actState;
      _trailingEdge = ~_actState;
      return;
    }
    
    void resetChanged(){                             // clear alle events of 'pressed', 'released' or 'changed'
      _lastChanged = _actState;
      _leadingEdge = 0;
      _trailingEdge = 0;
      return;
    }
    
    bool shortPress( uint8_t buttonNbr ) {       // if button was pressed short
     if ( buttonNbr >= _buttonCnt ) return 0;
      // get short pressed state of button (debounced)
      bool temp = bitRead( _shortPress, buttonNbr );
      bitClear( _shortPress, buttonNbr );
      return temp;
    }
    bool longPress( uint8_t buttonNbr ) {        // if button was pressed long
      if ( buttonNbr >= _buttonCnt ) return 0;
      // get long pressed state of button (debounced)
      bool temp = bitRead( _longPress, buttonNbr );
      bitClear( _longPress, buttonNbr );
      return temp;
    }

    bool pressed( uint8_t buttonNbr ) {          // leading edge of button press
      if ( buttonNbr >= _buttonCnt ) return 0;
      // get momentarily pressed state of button (debounced)
      bool temp = bitRead( _leadingEdge, buttonNbr );
      bitClear( _leadingEdge, buttonNbr );
      return temp;
    }
    bool released( uint8_t buttonNbr ) {         // trailing edge of button press
      if ( buttonNbr >= _buttonCnt ) return 0;
      // get momentarily released state of button (debounced)
      bool temp = bitRead( _trailingEdge, buttonNbr );
      bitClear( _trailingEdge, buttonNbr );
      return temp;
    }

    bool doubleClick( uint8_t buttonNbr ) {         // double click of button press
      if ( buttonNbr >= _buttonCnt ) return 0;
      // get double click state of button (debounced)
      bool temp = bitRead( _doubleClick, buttonNbr );
      bitClear( _doubleClick, buttonNbr );
      return temp;
    }

    private:
    uint8_t _pinCnt;
    uint8_t *_pinArray;
    uint8_t _debTime;            // Debounce time im ms
    uint8_t _pressTime;          // pressTime measured in debounce tics
    uint8_t _dClickTime;        // double click time measured in debouce tics
    uint32_t _lastReadTime;     // Last time HW state was read
    static const uint8_t   _buttonCnt = sizeof(button_t)*8;        // Number of buttons
    button_t  (*_getHWbuttons)();  // Ptr to user function to read raw state of buttons
    // Bit fields to hold various button states
    button_t  _lastState;
    button_t  _lastChanged;
    button_t  _actState;
    button_t  _longPress;
    button_t  _shortPress;
    button_t  _leadingEdge;
    button_t  _trailingEdge;
    button_t  _doubleClick;
    uint8_t   _buttonTime[ _buttonCnt ]; // Time in debounce tics

  

};


#endif
