/* MoTobutton - a Arduino library to manage up to 8, 16 or 32 Buttons/Switches 
  with debouncing, edge detection and long/short press detection.
  Author: fpm, fpm-gh@mnet-mail.de
  Copyright (c) 2020 All right reserved.

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
    MoToButton( button_t (*getHWbuttons)(), uint8_t debTime, uint16_t pressTime );
    example in sketch: 
        MoToButton Buttons( readFunction, 20, 500 );
  
  Methods to be called in loop:
    void    processButtons();                   // must be called in the loop frequently
                                                // if it is called less frequently than debTime, pressTime will be inaccurate
    Reading the debounced state of the Buttons/Switches:                                          
      boolean state( uint8_t buttonNbr );       // get static state of button (debounced)
      button_t allStates();                     // bit field of all buttons (debounced)
      button_t changed();                       // all bits are set where state has changed since last call
  
    Reading events:
      boolean shortPress( uint8_t buttonNbr );  // true if button was pressed short ( set when button is released, reset after call )  
      boolean longPress( uint8_t buttonNbr );   // true if button was pressed long ( set when button is released, reset after call )  
      boolean pressed( uint8_t buttonNbr );     // true if button is pressed ( reset after call )
      boolean released( uint8_t buttonNbr );    // true if button is released ( reset after call )
  
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

#ifndef MOTOBUTTON_H
#define MOTOBUTTON_H

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


class MoToButton {
  public:
    MoToButton( button_t (*getHWbuttons)(), uint8_t debTime, uint16_t pressTime ) {
      _getHWbuttons = getHWbuttons;
      _debTime = debTime;
      _pressTime = pressTime / debTime;   // in debTime tics
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

    void processButtons() {
      // must be called in loop frequently
      if ( millis() - _lastReadTime > (uint32_t) _debTime ) {
        _lastReadTime = millis();
        _actState = _getHWbuttons();    // read button states
        // edge detection - new detected edges are added to the corresponding bit field
        // edge bits are cleard when read or at the next inverted edge ( pressed-> released or vice versa )
        // leading edge
        _leadingEdge &= _actState;  // clear bits if button is no longer pressed
        _leadingEdge = (~_lastState & _actState) | _leadingEdge;
        // trailing edge
        _trailingEdge &= ~_actState;  // clear bits if button is pressed again
        _trailingEdge = (_lastState & ~_actState) | _trailingEdge ;

        _lastState = _actState;

        // process pressing time
        for ( byte i = 0; i < _buttonCnt; i++ ) {
          if ( bitRead( _actState, i ) ) {
            // button is still pressed, update time counter
            bitClear( _longPress, i );
            bitClear( _shortPress, i );
            if ( _buttonTime[i] < _pressTime ) _buttonTime[i]++;
          } else {
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

    boolean state( uint8_t buttonNbr ) {            // get static state of button (debounced)
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
    
    boolean shortPress( uint8_t buttonNbr ) {       // if button was pressed short
     if ( buttonNbr >= _buttonCnt ) return 0;
      // get short pressed state of button (debounced)
      boolean temp = bitRead( _shortPress, buttonNbr );
      bitClear( _shortPress, buttonNbr );
      return temp;
    }
    boolean longPress( uint8_t buttonNbr ) {        // if button was pressed long
      if ( buttonNbr >= _buttonCnt ) return 0;
      // get short pressed state of button (debounced)
      boolean temp = bitRead( _longPress, buttonNbr );
      bitClear( _longPress, buttonNbr );
      return temp;
    }

    boolean pressed( uint8_t buttonNbr ) {          // leading edge of button press
      if ( buttonNbr >= _buttonCnt ) return 0;
      // get momentarily pressed state of button (debounced)
      boolean temp = bitRead( _leadingEdge, buttonNbr );
      bitClear( _leadingEdge, buttonNbr );
      return temp;
    }
    boolean released( uint8_t buttonNbr ) {         // trailing edge of button press
      if ( buttonNbr >= _buttonCnt ) return 0;
      // get momentarily released state of button (debounced)
      boolean temp = bitRead( _trailingEdge, buttonNbr );
      bitClear( _trailingEdge, buttonNbr );
      return temp;
    }

    private:
    uint8_t _debTime;            // Debounce time im ms
    uint8_t _pressTime;          // pressTime measured in debounce tics
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
    uint8_t   _buttonTime[ _buttonCnt ]; // Time in debounce tics

  

};


#endif
