SDI12
=========

<h3>Teensy 3.x SDI12 Library V1</h3>

<h4>This library implements the SDI12 v1.3 protocol, nativaly using Teensy's hardware serial one-wire protocol.</h4>

<b>[SDI12 Specification]</b>
> SDI12 is a single wire serial protocol that uses inverted 5V logic levels, specifically (1200 baud, 7E1) for bi-directional data flow with one Master and many Slaves. This library sets up the Freescale Cortex one-wire protocol for each of its 3 Hardware Serial ports TX pin along with critical timing - Break and Mark signals to wake the sensor bus. This makes effectivaly 3 seperate SDI12 buses that can be used or not. Since SDI12 is Master-Slave, many different types of sensor can share the same bus through the use of unique address for each senor.

<br>
Since Teensy are 3.3V micrcontrollers it is actually out of SDI12 specification:<br>
1. Spacing (3.5V to 5V)<br>
2. Marking (-0.5V to 1V)<br>

Though all the sensors listed below will still work with 3.3V signals. I'm working on proper level shifting to be in spec.

<b>Sensor Tested:</b>
>1. [Decagon 5TE]
>2. [Decagon CTD]
>3. [Keller DigiLevel]
>4. [Vaisala WXT520]

<b>Hookup</b>
>1. Teensy 3.0 are not 5V tolerant, put a resistor inlined with the data line.<br>
>2. Teensy 3.1 is 5V tolerant so direct connection can be done.<br>
>3. While SDI12 specification states 12V is used for power, many sensors use a range of values (5-17V).<br>

Connect the sensor to teensy using Teensy's Vin (5V) for sensor power.<br>
```
  Teensy 3.0                                                   Sensor
 ----------------                                           -------------  
| Serial Port TX |<------------[10K Resistor]------------->|    Data     |
|       GND      |<----------------------------------------|    GND      |
|       Vin      |---------------------------------------->|    Power    |
 ----------------                                           -------------
 
   Teensy 3.1                                                  Sensor
 ----------------                                           -------------  
| Serial Port TX |<--------------------------------------->|    Data     |
|       GND      |<----------------------------------------|    GND      |
|       Vin      |---------------------------------------->|    Power    |
 ----------------                                           -------------
```
Connect the sensor to teensy using external Vin for sensor power.<br>
```
  Teensy 3.0                                                   Sensor
 ----------------                                           -------------  
| Serial Port TX |<------------[10K Resistor]------------->|    Data     |
|       GND      |--------|                    |-----------|    GND      |
|       Vin      |X       |                    | |-------->|    Power    |
 ----------------         |    External Power  | |          -------------
                          |      -------       | |
                          |---->| GND   |<-----| |
                                | Power |--------|
                                 -------
  Teensy 3.1                                                   Sensor
 ----------------                                           -------------  
| Serial Port TX |<--------------------------------------->|    Data     |
|       GND      |--------|                    |-----------|    GND      |
|       Vin      |X       |                    | |-------->|    Power    |
 ----------------         |    External Power  | |          -------------
                          |      -------       | |
                          |---->| GND   |<-----| |
                                | Power |--------|
                                 -------
```

<h2>Usage</h2><br>
<b>Constructor:</b>
---
```c
SDI12( Stream *port, char address, bool crc = false ) ;
```
>1. ```Stream *port``` = One of the Hardware Serial Ports.
>2. ```char address``` = Sensor Address, must be preprogramed.
>3. ```bool crc```(optional) = Set 'true' to append CRC to sensor data.

Example:
```c
// Define a constructor for each sensor you plan to use.
// Serial Port can either be Serial1, Serial2, Serial3.
// Address have to be ascii values. (0-10),(a-z),(A-Z).
// CRC will be appended to returned data packet for each sensor.
SDI12 DECAGON_5TE_10CM( &Serial3, '1', true );
SDI12 DECAGON_5TE_20CM( &Serial3, '2', true );
SDI12 DECAGON_5TE_30CM( &Serial3, '3', true );
SDI12 DECAGON_5TE_40CM( &Serial3, '4', true );
```
<br>
<h3>--------------------------Functions-------------------------------</h3>

<b>isActive:</b>
---
```c
//SDI12 (Acknowledge Active) "a!" command.
bool isActive( int address = -1 );
```
>1. ```int address```(optional) = can use other address also.

Example:
```c
int error;

// Check if sensor is active using address defined in constructor. 
error = DECAGON_5TE_10CM.isActive( );
if( error ) Serial.println( "Sensor at address 1 Not Active" );

// Check if sensor is active using address defined in constructor. 
error = DECAGON_5TE_20CM.isActive( );
if( error ) Serial.println( "Sensor at address 2 Not Active" );

// Check if sensor is active using address defined in constructor. 
error = DECAGON_5TE_30CM.isActive( );
if( error ) Serial.println( "Sensor at address 3 Not Active" );

// Check if sensor is active using address defined in constructor. 
error = DECAGON_5TE_40CM.isActive( );
if( error ) Serial.println( "Sensor at address 4 Not Active" );

// Check if sensor is active using different address than defined 
// in constructor. This allows us to see if it is actually a 
// different address.
error = DECAGON_5TE_40CM.isActive( '5' );
if( error ) Serial.println( "Sensor at address 5 Not Active" );
```

<br><b>identification:</b>
---
```c
// SDI12 (Send Indentification) "aI!" command.
bool identification( const char *src ) { identification( (const uint8_t *)src ); }
bool identification( const uint8_t *src );
```
>1. ```const (char or uint8_t) *src``` = buffer array you supply to hold returned string.

Example:
```c
int error;
// Max size of return string is 35 character.
char buf[35]

// If no error then print id string. 
error = DECAGON_5TE_10CM.identification( buf );
if( !error ) Serial.println( buf );

// If no error then print id string. 
error = DECAGON_5TE_20CM.identification( buf );
if( !error ) Serial.println( buf );

// If no error then print id string.
error = DECAGON_5TE_30CM.identification( buf );
if( !error ) Serial.println( buf );

// If no error then print id string.
error = DECAGON_5TE_40CM.identification( buf );
if( !error ) Serial.println( buf );
```
<br><b>queryAddress:</b>
---
```c
// SDI12 (Address Query) "?!" command.
/*** Only ONE sensor can on the bus when using this command! ***/
int queryAddress( void );
```
Example:
```c
int address;

// Used to see what your sensor address actually is.
// Only one sensor can be connected to bus at a time. 
address = DECAGON_5TE_10CM.queryAddress( );
if(address != -1) {
    Serial.print( "Sensor Address is " );
    Serial.println( (char)address )
}
```

<br><b>changeAddress:</b>
---
```c
// SDI12 (Change Address) "aAb!" command.
int changeAddress( uint8_t new_address );
```
>1. ```uint8_t new_address``` = address you want to change to.

Example:
```c
int address;

// Change address defined in the constructor '1' to '5'.
// This address will be updated for any future use of this function.
address = DECAGON_5TE_10CM.changeAddress( '5' );
if( address != -1 ) {
    Serial.print( "New Address is" );
    Serial.println( (char)address );
} else {
    Serial.println( "Address out of range or command failed" );
}
```
<br><b>verification:</b>
---
```c
// SDI12 (Start Verification) "aV!" command.
bool verification( const char *src ) { verification( (const uint8_t *)src ); }
bool verification( const uint8_t *src );
```
>1. ```const (char or uint8_t) *src``` = buffer array you supply to hold returned string.

Example:
```c
int error;
// Buffer to hold returned string.
char buf[35];
// Opionally can get debug info on command return
char debug[10];

error = DECAGON_5TE_10CM.verification( debug );
if ( !error ) Serial.print( debug );

// Verification needs a return measurement command to get data
// 'returnMeasurement' function is explained below.
error = DECAGON_5TE_10CM.returnMeasurement( buf, 0 );
if ( !error ) Serial.print( buf );
```

<br><b>measurement:</b>
---
```c
// SDI12 (Start Measurement) command.
// "aM!", "aMC!" or "aM0...aM9" or "aMC0...aMC9"
bool measurement( int num = -1 ) { uint8_t s[75]; measurement( s, num ); }
bool measurement( const char *src, int num = -1 ) { measurement( (const uint8_t *)src, num ); }
bool measurement( const uint8_t *src, int num = -1 );
```
>1. ```const (char or uint8_t) *src``` = buffer array you supply to hold returned string.
>2. ```int num = -1```(optional) = Additional measuremnts.

Example:
```c
int error;
// buffer to hold sensor measurement acknowldegement string.
char debug[10];
// Max return sensor string size is 81 characters.
char data[81];

memset( data, 0, 81 );
memset( debug, 0, 10 );
// Function to tell sensor to make a measurement.
// optional additional measurements command.
/***error = DECAGON_5TE_10CM.measurement( debug, 0 );***/
error = DECAGON_5TE_10CM.measurement( debug );
if ( !error ) Serial.print( debug );
// 'measurement' needs a return measurement command to get data.
// 'returnMeasurement' function is explained below.
error = DECAGON_5TE_10CM.returnMeasurement( data, 0 );
if ( !error ) Serial.print( data );

 memset( data, 0, 81 );
 memset( debug, 0, 10 );
// Function to tell sensor to make a measurement.
// optional additional measurements command.
/***error = DECAGON_5TE_20CM.measurement( debug, 0 );***/
 error = DECAGON_5TE_20CM.measurement( debug );
 if ( !error ) Serial.print( debug );
 error = DECAGON_5TE_20CM.returnMeasurement( data, 0 );
 if (!error) Serial.print( data );
   
memset( data, 0, 81 );
memset( debug, 0, 10 );
// Function to tell sensor to make a measurement.
// optional additional measurements command.
/***error = DECAGON_5TE_30CM.measurement( debug, 0 );***/
error = DECAGON_5TE_30CM.measurement( debug );
if ( !error ) Serial.print( debug );
error = DECAGON_5TE_30CM.returnMeasurement( data, 0 );
if ( !error ) Serial.print( data );
   
memset( data, 0, 81 );
memset( debug, 0, 10 );
// Function to tell sensor to make a measurement.
// optional additional measurements command.
/***error = DECAGON_5TE_40CM.measurement( debug, 0 );***/
error = DECAGON_5TE_40CM.measurement( debug );
if ( !error ) Serial.print( debug );
error = DECAGON_5TE_40CM.returnMeasurement( data, 0 );
if ( !error ) Serial.print( data );
```

<br><b>concurrent:</b>
---
```c
// SDI12 (Start Concurrent Measurement) command.
// "aC!","aCC!" or "aC0...aC9" or "aCC0...aCC9"
bool concurrent( int num = -1 ) { uint8_t s[75]; concurrent( s, num ); }
bool concurrent( const char *src, int num = -1 ) { concurrent( (const uint8_t *)src, num ); }
bool concurrent(  const uint8_t *src, int num  );
```
>1. ```const (char or uint8_t) *src``` = buffer array you supply to hold returned string.
>2. ```int num = -1```(optional) = Additional measuremnts.

Example:
```c
// Not implemented yet... 
/* 
 * This function will provide a non blocking way to read sensors.
 * This will be the function used in "Background Mode" where 
 * sensors can be logged autmatically.
 */
```

<br><b>continuous:</b>
---
```c
// SDI12 (Start Continuous Measurement) command.
// "aR0!...aR9!" or "aRC0!...aRC9!"
bool continuous( const char *src, int num = -1 ) { continuous( (const uint8_t *)src, num ); }
bool continuous( const uint8_t *src, int num = -1 );
```
>1. ```const (char or uint8_t) *src``` = buffer array you supply to hold returned string.
>2. ```int num = -1```(optional) = Additional measuremnts.

Example:
```c
int error;
// buffer to hold sensor measurement string. 
// Max return sensor string size is 81 characters.
char data[81];

memset( data, 0, 81 );
// Function to tell sensor to get a continuous measurement 
// if the sensor supports it.
// optional additional measurements command.
/***error = DECAGON_5TE_10CM.continuous( data, 0 );***/
error = DECAGON_5TE_10CM.continuous( data );
if ( !error ) Serial.print( data );

```

<br><b>returnMeasurement:</b>
---
```c
// SDI12 (Return Measurement) command.
// "aD!" or "aD0!...aD9!"
bool returnMeasurement( const char *src, int num = -1 ) { returnMeasurement( (const uint8_t *)src, num ); }
bool returnMeasurement( const uint8_t *src, int num = -1 );
```
>1. ```const (char or uint8_t) *src``` = buffer array you supply to hold returned string.
>2. ```int num = -1```(optional) = Additional measuremnts.

Example:
```c
// This function will send the send data command to get the data string.
// Example is provided with 'measurement' function above.
```


<br>transparent:
---
```c
// SDI12 (Transparent) command. Allows extended SDI12 commands.
bool transparent( const char *command, const uint8_t *src ) { transparent( (uint8_t*)command, src ); }
bool transparent( const uint8_t *command, const char *src ) { transparent( command, (uint8_t*)src ); }
bool transparent( const char *command, const char *src ) { transparent( (uint8_t*)command, (uint8_t*)src ); }
bool transparent( const uint8_t *command, const uint8_t *src );
```
>1. ```const (char or uint8_t) *command``` = SDI12 command to send.
>2. ```const (char or uint8_t) *src``` = buffer array you supply to hold returned string.

Example:
```c
int error;
// Command to send.
char cmd[3] = "2I!";
// buffer to hold sensor measurement string. 
// Max return sensor string size is 81 characters.
char data[81];

memset( data, 0, 81 );
// Function to tell sensor to send a tranparent command.
// If using 'M' command it will handle sensor acknowledgement also.
error = DECAGON_5TE_10CM.transparent( cmd, data );
if ( !error ) Serial.print( data );
```


[SDI12 Specification]:http://www.sdi-12.org/current%20specification/SDI-12_version1_3%20January%2026,%202013.pdf
[Vaisala WXT520]:http://www.vaisala.com/en/products/multiweathersensors/Pages/WXT520.aspx
[Decagon 5TE]:http://www.decagon.com/products/soils/volumetric-water-content-sensors/5te-vwc-ec-temp/
[Decagon CTD]:http://www.decagon.com/products/hydrology/water-level-temperature-electrical-conductivity/ctd-5-10-sensor-electrical-conductivity-temperature-depth/
[Keller DigiLevel]:http://www.kelleramerica.com/blog/?tag=digilevel
