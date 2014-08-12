SDI12
=========

<h3>Teensy 3.x SDI12 Library V1</h3>

<h4>This library implements the SDI12 v1.3 hardware serial protocol nativaly using the special one-wire serial protocol.</h4>

<b>SDI12 Specification</b>
> SDI12 is a single wire serial protocol that uses inverted 5V logic levels, specifically (1200 baud, 7E1) for bi-directional data flow with one Master and many Slaves. This library sets up the Freescale Cortex one-wire protocol for each of its 3 Hardware Serial ports TX line along with timing critical Break and Mark signals to wake the sensor bus. This makes effectivaly 3 seperate SDI12 buses that can be used or not. Since SDI12 is Master-Slave, many different types of sensor can share the same bus through the use of unique address for each senor.


<b>Hookup</b>
>1. Teensy 3.0 are not 5V tolerant, put a resistor inlined with the data line.<br>
>2. Teensy 3.1 is 5V tolerant so direct connection can be done.<br>
>3. While SDI12 specification states 12V is used for power many sensors use a range of values (5-17V).<br>

Below is typically how you would connect the sensor to teensy.<br>



<b>[Teensy 3.0 Serial 1,2,3]</b><----------==<b>[10K Resistor]</b>==----------><b>[Sensor Data Line]</b><br>
