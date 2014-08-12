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

<h4>Usage:</h4><br>
<b>Functions:</b>
```c
bool isActive( int address = -1 );

```
```c
bool identification( const char *src ) { identification( (const uint8_t *)src ); }
bool identification( const uint8_t *src );

```
```c
int queryAddress( void );

```
```c
int changeAddress( uint8_t new_address );

```
```c
bool verification( const char *src ) { verification( (const uint8_t *)src ); }
bool verification( const uint8_t *src );

```
```c
bool measurement( int num = -1 ) { uint8_t s[75]; measurement( s, num ); }
bool measurement( const char *src, int num = -1 ) { measurement( (const uint8_t *)src, num ); }
bool measurement( const uint8_t *src, int num = -1 );

```
```c
bool concurrent( int num = -1 ) { uint8_t s[75]; concurrent( s, num ); }
bool concurrent( const char *src, int num = -1 ) { concurrent( (const uint8_t *)src, num ); }
bool concurrent(  const uint8_t *src, int num  );

```
```c
bool continuous( const char *src, int num = -1 ) { continuous( (const uint8_t *)src, num ); }
bool continuous( const uint8_t *src, int num = -1 );

```
```c
bool returnMeasurement( const char *src, int num = -1 ) { returnMeasurement( (const uint8_t *)src, num ); }
bool returnMeasurement( const uint8_t *src, int num = -1 );

```
```c
bool transparent( const char *command, const uint8_t *src ) { transparent( (uint8_t*)command, src ); }
bool transparent( const uint8_t *command, const char *src ) { transparent( command, (uint8_t*)src ); }
bool transparent( const char *command, const char *src ) { transparent( (uint8_t*)command, (uint8_t*)src ); }
bool transparent( const uint8_t *command, const uint8_t *src );

```
[SDI12 Specification]:http://www.sdi-12.org/current%20specification/SDI-12_version1_3%20January%2026,%202013.pdf
