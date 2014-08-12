SDI12
=========

<h3>Teensy 3.x SDI12 Library V1</h3>

<h4>This library implements the SDI12 v1.3 hardware serial protocol nativaly using the special one-wire serial protocol.</h4>

<b>[SDI12 Specification]</b>
> SDI12 is a single wire serial protocol that uses inverted 5V logic levels, specifically (1200 baud, 7E1) for bi-directional data flow with one Master and many Slaves. This library sets up the Freescale Cortex one-wire protocol for each of its 3 Hardware Serial ports TX line along with timing critical Break and Mark signals to wake the sensor bus. This makes effectivaly 3 seperate SDI12 buses that can be used or not. Since SDI12 is Master-Slave, many different types of sensor can share the same bus through the use of unique address for each senor.


<b>Hookup</b>
>1. Teensy 3.0 are not 5V tolerant, put a resistor inlined with the data line.<br>
>2. Teensy 3.1 is 5V tolerant so direct connection can be done.<br>
>3. While SDI12 specification states 12V is used for power many sensors use a range of values (5-17V).<br>

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
```c
SDI12( Stream *port, char address, bool crc = false ) ;
```
1. ```Stream *port``` = One of the Hardware Serial Ports.
2. ```char address``` = Sensor Address, must be preprogramed.
3. ```bool crc```(optional) = Set 'true' to append CRC to sensor data.

Example:
```c
// Define a constructor for each sensor you plan to use.
// Serial Port can either be Serial1, Serial2, Serial3.
// Address have to be ascii values. (0-10),(a-z),(A-Z).
// CRC will be appended to returned data packet for each sensor.
SDI12 DECAGON_5TE_10CM(&Serial3, '1', true);
SDI12 DECAGON_5TE_20CM(&Serial3, '2', true);
SDI12 DECAGON_5TE_30CM(&Serial3, '3', true);
SDI12 DECAGON_5TE_40CM(&Serial3, '4', true);
```
<br>
<h3>-------------------------------------Functions----------------------------------</h3>

<b>isActive:</b>
```c
//SDI12 (Acknowledge Active) "a!" command.
bool isActive( int address = -1 );
```
1. ```int address```(optional) = can use other address also.

Example:
```c
// Check if sensor is active using address defined in constructor. 
DECAGON_5TE_10CM.isActive( );
// Check if sensor is active using address defined in constructor. 
DECAGON_5TE_20CM.isActive( );
// Check if sensor is active using address defined in constructor. 
DECAGON_5TE_30CM.isActive( );
// Check if sensor is active using address defined in constructor. 
DECAGON_5TE_40CM.isActive( );
// Check if sensor is active using different address than defined 
// in constructor. This allows us to see if it is actually a 
// different address.
DECAGON_5TE_40CM.isActive('5');
```

<br><b>identification:</b>
```c
// SDI12 (Send Indentification) "aI!" command.
bool identification( const char *src ) { identification( (const uint8_t *)src ); }
bool identification( const uint8_t *src );

```
<br><b>queryAddress:</b>
```c
int queryAddress( void );

```
<br><b>changeAddress:</b>
```c
int changeAddress( uint8_t new_address );

```
<br><b>verification:</b>
```c
bool verification( const char *src ) { verification( (const uint8_t *)src ); }
bool verification( const uint8_t *src );

```
<br><b>measurement:</b>
```c
bool measurement( int num = -1 ) { uint8_t s[75]; measurement( s, num ); }
bool measurement( const char *src, int num = -1 ) { measurement( (const uint8_t *)src, num ); }
bool measurement( const uint8_t *src, int num = -1 );

```
<br><b>concurrent:</b>
```c
bool concurrent( int num = -1 ) { uint8_t s[75]; concurrent( s, num ); }
bool concurrent( const char *src, int num = -1 ) { concurrent( (const uint8_t *)src, num ); }
bool concurrent(  const uint8_t *src, int num  );

```
<br><b>continuous:</b>
```c
bool continuous( const char *src, int num = -1 ) { continuous( (const uint8_t *)src, num ); }
bool continuous( const uint8_t *src, int num = -1 );

```
<br><b>returnMeasurement:</b>
```c
bool returnMeasurement( const char *src, int num = -1 ) { returnMeasurement( (const uint8_t *)src, num ); }
bool returnMeasurement( const uint8_t *src, int num = -1 );

```
<br><b>transparent:</b>
```c
bool transparent( const char *command, const uint8_t *src ) { transparent( (uint8_t*)command, src ); }
bool transparent( const uint8_t *command, const char *src ) { transparent( command, (uint8_t*)src ); }
bool transparent( const char *command, const char *src ) { transparent( (uint8_t*)command, (uint8_t*)src ); }
bool transparent( const uint8_t *command, const uint8_t *src );

```
[SDI12 Specification]:http://www.sdi-12.org/current%20specification/SDI-12_version1_3%20January%2026,%202013.pdf
