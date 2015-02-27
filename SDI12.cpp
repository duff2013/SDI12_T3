/*
 ||
 || @file 	SDI12.cpp
 || @version 	2
 || @author 	Colin Duffy
 || @contact 	cmduffy@engr.psu.edu
 || @license
 || | Copyright (c) 2015 Colin Duffy
 || | This library is free software; you can redistribute it and/or
 || | modify it under the terms of the GNU Lesser General Public
 || | License as published by the Free Software Foundation; version
 || | 2.1 of the License.
 || |
 || | This library is distributed in the hope that it will be useful,
 || | but WITHOUT ANY WARRANTY; without even the implied warranty of
 || | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 || | Lesser General Public License for more details.
 || |
 || | You should have received a copy of the GNU Lesser General Public
 || | License along with this library; if not, write to the Free Software
 || | Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 || #
 ||
 */

#include "SDI12.h"
#include "Arduino.h"

enum { IDLE, BREAK, BREAK_DELAY, MARK, MARK_DELAY, TRANSMITTING, RECIEVING, MEASURE_RESPONSE, DATA_RESPONSE };

volatile bool     serviceRequest  = false;
volatile bool     commandResponse = false;
volatile bool     startBit        = false;
volatile bool     ioActive        = false;
volatile bool     transmitting    = false;

volatile uint16_t txHead = 0;
volatile uint16_t txTail = 0;
volatile uint16_t rxHead = 0;
volatile uint16_t rxTail = 0;

uint8_t serial1_vector_allocated_mask = 0;
uint8_t serial2_vector_allocated_mask = 0;
uint8_t serial3_vector_allocated_mask = 0;

volatile uint8_t tx_buffer[TX_BUFFER_SIZE];
volatile uint8_t rx_buffer[RX_BUFFER_SIZE];

void uart0_isr( void );
void uart1_isr( void );
void uart2_isr( void );
/****************************************************************************/
/*                                  UART                                    */
/****************************************************************************/
int UART::read( void ) {
    uint32_t head, tail;
	int c;
    
	head = rxHead;
	tail = rxTail;
	if (head == tail) return -1;
	if (++tail >= RX_BUFFER_SIZE) tail = 0;
	c = rx_buffer[tail];
	rxTail = tail;
	return c;
}

int UART::available( void ) {
    uint32_t head, tail;
	head = rxHead;
	tail = rxTail;
	if (head >= tail) return head - tail;
	return RX_BUFFER_SIZE + head - tail;
}

int UART::end( void ) {
    return 0;
}

void UART::flush( void ) {
    while ( transmitting ) yield( );
}

void UART::clear( void ) {
    rxHead = rxTail;
}

int UART::peek( void ) {
    return 0;
}
/****************************************************************************/
/*                                 SDI12                                    */
/****************************************************************************/
void SDI12::init( void ) {
    //int ascii_numbers    = (registeredAddress >= 0x30) && (registeredAddress <= 0x39);
    //int ascii_upper_case = (registeredAddress >= 0x41) && (registeredAddress <= 0x5A);
    //int ascii_lower_case = (registeredAddress >= 0x61) && (registeredAddress <= 0x7A);
    
    if ( sensor.uart == &Serial1 ) {
        RUN_ONCE_BEGIN;
        ENABLE_UART0;
        attachInterruptVector( IRQ_UART0_STATUS, uart0_isr );
        NVIC_ENABLE_IRQ( IRQ_UART0_STATUS );
        RUN_ONCE_END;
        REG        = &UART0;
        RX_PIN     = &PORTB_PCR16;
        TX_PIN     = &PORTB_PCR17;
        SET        = &GPIOB_PSOR;
        CLEAR      = &GPIOB_PCOR;
        BITMASK    = CORE_PIN1_BITMASK;
        PIN_NUM_RX = 0;
        PIN_NUM_TX = 1;
    }
    else if ( sensor.uart == &Serial2 ) {
        RUN_ONCE_BEGIN;
        ENABLE_UART1;
        attachInterruptVector( IRQ_UART1_STATUS, uart1_isr );
        NVIC_ENABLE_IRQ( IRQ_UART1_STATUS );
        RUN_ONCE_END;
        REG        = &UART1;
        RX_PIN     = &PORTC_PCR3;
        TX_PIN     = &PORTC_PCR4;
        SET        = &GPIOC_PSOR;
        CLEAR      = &GPIOC_PCOR;
        BITMASK    = CORE_PIN10_BITMASK;
        PIN_NUM_RX = 9;
        PIN_NUM_TX = 10;
    }
    else if ( sensor.uart == &Serial3 ) {
        RUN_ONCE_BEGIN;
        ENABLE_UART2;
        attachInterruptVector( IRQ_UART2_STATUS, uart2_isr );
        NVIC_ENABLE_IRQ( IRQ_UART2_STATUS );
        RUN_ONCE_END;
        REG         = &UART2;
        RX_PIN      = &PORTD_PCR2;
        TX_PIN      = &PORTD_PCR3;
        SET         = &GPIOD_PSOR;
        CLEAR       = &GPIOD_PCOR;
        BITMASK     = CORE_PIN8_BITMASK;
        PIN_NUM_RX  = 7;
        PIN_NUM_TX  = 8;
    }
}

void SDI12::allocateVector( void ) {
    uint32_t vect_channel = 0;
    
    if ( sensor.uart == &Serial1 ) {
        __disable_irq( );
        while (1) {
            if (!(serial1_vector_allocated_mask & ( 1 << vect_channel ) ) ) {
                serial1_vector_allocated_mask |= ( 1 << vect_channel );
                __enable_irq( );
                break;
            }
            if ( ++vect_channel >= 8 ) {
                __enable_irq( );
                return; // no more vector channels available
            }
        }
    }
    else if ( sensor.uart == &Serial2 ) {
        __disable_irq( );
        while ( 1 ) {
            if (!(serial2_vector_allocated_mask & ( 1 << vect_channel ) ) ) {
                serial2_vector_allocated_mask |= ( 1 << vect_channel );
                __enable_irq( );
                break;
            }
            if ( ++vect_channel >= 8 ) {
                __enable_irq( );
                return; // no more vector channels available
            }
        }
    }
    else if ( sensor.uart == &Serial3 ) {
        __disable_irq( );
        while ( 1 ) {
            if (!(serial3_vector_allocated_mask & ( 1 << vect_channel ) ) ) {
                serial3_vector_allocated_mask |= ( 1 << vect_channel );
                __enable_irq( );
                break;
            }
            if ( ++vect_channel >= 8 ) {
                __enable_irq( );
                return; // no more vector channels available
            }
        }
    }
    /*Serial.printf("a serial1_vector_allocated_mask: %04X | ", serial1_vector_allocated_mask);
    Serial.println(serial1_vector_allocated_mask, BIN);
    Serial.printf("a serial2_vector_allocated_mask: %04X | ", serial2_vector_allocated_mask);
    Serial.println(serial2_vector_allocated_mask, BIN);
    Serial.printf("a serial3_vector_allocated_mask: %04X | ", serial3_vector_allocated_mask);
    Serial.println(serial3_vector_allocated_mask, BIN);
    Serial.println();*/
}

void SDI12::releaseVector( void ) {
    uint32_t vect_channel = 0xFF;
    
    if ( sensor.uart == &Serial1 ) {
        __disable_irq( );
        while ( 1 ) {
            if ( ( serial1_vector_allocated_mask & ( 1 << vect_channel ) ) ) {
                serial1_vector_allocated_mask &= ~( 1 << vect_channel );
                __enable_irq( );
                break;
            }
            vect_channel--;
        }
        __disable_irq( );
        if ( serial1_vector_allocated_mask == 0 ) {
            //detachInterruptVector( IRQ_UART0_STATUS );
            __enable_irq( );
        }
    }
    else if ( sensor.uart == &Serial2 ) {
        __disable_irq( );
        while ( 1 ) {
            if ( ( serial2_vector_allocated_mask & ( 1 << vect_channel ) ) ) {
                serial2_vector_allocated_mask &= ~( 1 << vect_channel );
                __enable_irq( );
                break;
            }
            vect_channel--;
        }
        __disable_irq( );
        if ( serial2_vector_allocated_mask == 0 ) {
            //detachInterruptVector( IRQ_UART1_STATUS );
            __enable_irq( );
        }
    }
    else if ( sensor.uart == &Serial3 ) {
        __disable_irq( );
        while ( 1 ) {
            if ( ( serial3_vector_allocated_mask & ( 1 << vect_channel ) ) ) {
                serial3_vector_allocated_mask &= ~( 1 << vect_channel );
                __enable_irq( );
                break;
            }
            vect_channel--;
        }
        __disable_irq( );
        if ( serial3_vector_allocated_mask == 0 ) {
            //detachInterruptVector( IRQ_UART2_STATUS );
            __enable_irq( );
        }
    }
    /*Serial.printf("r serial1_vector_allocated_mask: %04X | ", serial1_vector_allocated_mask);
    Serial.println(serial1_vector_allocated_mask, BIN);
    Serial.printf("r serial2_vector_allocated_mask: %04X | ", serial2_vector_allocated_mask);
    Serial.println(serial2_vector_allocated_mask, BIN);
    Serial.printf("r serial3_vector_allocated_mask: %04X | ", serial3_vector_allocated_mask);
    Serial.println(serial3_vector_allocated_mask, BIN);
    Serial.println();*/
}

void SDI12::begin( void ) {

}

int SDI12::isActive( int address ) {
    if ( ioActive ) return -5;
    else ioActive = true;
    
    int error;
    
    uint8_t command[3];
    if ( address == -1 ) command[0] = sensor.address;
    else command[0] = address;
    command[1] = '!';
    
    error = send_command( command, 2, BLOCKING );
    if ( error ) {
        ioActive = false;
        return -1;
    }
    
    uint8_t p[128];
    int i = 0;
    while ( available( ) ) p[i++] = read( );
    
    if ( p[0] == sensor.address ) {
        ioActive = false;
        return 0;
    }
    else {
        ioActive = false;
        return -2;
    }
}

int SDI12::identification( const uint8_t *src ) {
    if ( ioActive ) return -5;
    else ioActive = true;
    
    int error;
    
    uint8_t command[3];
    command[0] = sensor.address;
    command[1] = 'I';
    command[2] = '!';
    
    error = send_command( command, 3, BLOCKING );
    if ( error ) {
        ioActive = false;
        return -1;
    }
    
    uint8_t *s = ( uint8_t * )src;
    while ( available( ) ) *s++ = read( );
    ioActive = false;
    return 0;
}

int SDI12::changeAddress( const uint8_t new_address ) {
    if ( ioActive ) return -5;
    else ioActive = true;
    
    int error;
    int ascii_numbers    = ( new_address < 0x30 ) || ( new_address > 0x39 );
    int ascii_upper_case = ( new_address < 0x41 ) || ( new_address > 0x5A );
    int ascii_lower_case = ( new_address < 0x61 ) || ( new_address > 0x7A );

    if ( ascii_numbers & ascii_lower_case & ascii_upper_case ) {
        ioActive = false;
        return -3;
    }

    error = isActive( );

    if ( error ) {
        ioActive = false;
        return -4;
    }
    
    uint8_t command[4];
    command[0] = sensor.address;
    command[1] = 'A';
    command[2] = new_address;
    command[3] = '!';
    
    error = send_command( command, 4, BLOCKING );
    if ( error ) {
        ioActive = false;
        return -1;
    }
    
    int i = 0;
    char p[128];
    while ( available( ) ) p[i++] = read( );
    
    if ( p[0] == new_address ) {
        sensor.address = p[0];
        ioActive = false;
        return p[0];
    }
    else {
        ioActive = false;
        return -4;
    }
}

int SDI12::queryAddress( void ) {
    if ( ioActive ) return -5;
    else ioActive = true;
    
    bool error;
    
    uint8_t command[2];
    command[0] = '?';
    command[1] = '!';
    
    error = send_command( command, 2, BLOCKING );
    if ( error ) {
        ioActive = false;
        return -1;
    }
    
    int i = 0;
    char p[5];
    while ( available( ) ) p[i++] = read( );
    ioActive = false;
    return p[0];
}

int SDI12::verification( const uint8_t *src ) {
    if ( ioActive ) return -5;
    else ioActive = true;
    
    uint8_t *s = ( uint8_t * )src;
    int error;
    uint8_t command[6];
    command[0] = sensor.address;
    command[1] = 'V';
    command[2] = '!';
    error = send_command( command, 3, BLOCKING );
    if ( error ) {
        ioActive = false;
        return -1;
    }
    
    uint32_t timeout = 0;
    int i = 0;
    
    char v_reponse[10];
    serviceRequest = false;
    while ( available( ) ) *s++ = v_reponse[i++] = read( );
    
    if ( v_reponse[0] != sensor.address ) { ioActive = false; return -2; }
    if ( (v_reponse[1] < 0x30) || (v_reponse[1] > 0x39) ) { ioActive = false; return -3; }
    if ( (v_reponse[2] < 0x30) || (v_reponse[2] > 0x39) ) { ioActive = false; return -3; }
    if ( (v_reponse[3] < 0x30) || (v_reponse[3] > 0x39) ) { ioActive = false; return -3; }
    timeout  = ( v_reponse[1] - 0x30 ) * 100;
    timeout += ( v_reponse[2] - 0x30 ) * 10;
    timeout += ( v_reponse[3] - 0x30 );
    timeout *= 1000;
    timeout += 10;
    
    elapsedMillis time;
    time = 0;
    while ( !serviceRequest ) {
        if ( time >= timeout ) { ioActive = false; return -4; }
        yield( );
    }
    
    do {
        yield( );
    } while ( !(UART2_S1 & UART_S1_IDLE) );
    
    char service_request[10];
    i = 0;
    while ( available( ) ) *s++ = service_request[i++] = read( );
    if ( service_request[0] != sensor.address) { ioActive = false; return -5; }
    ioActive = false;
    return 0;
}

int SDI12::measurement( const uint8_t *src, int num ) {
    if ( ioActive ) return -5;
    else ioActive = true;
    int error;
    uint8_t command[5];
    command[0] = sensor.address;
    command[1] = 'M';
    if (num < 0) {
        if (sensor.crc) {
            command[2]  = 'C';
            command[3]  = '!';
            error = send_command( command, 4, BLOCKING );
            if ( error ) {
                ioActive = false;
                return -1;
            }
        }
        else {
            command[2]  = '!';
            error = send_command( command, 3, BLOCKING );
            if ( error ) {
                ioActive = false;
                return -1;
            }
        }
    }
    else {
        if (sensor.crc) {
            command[2]  = num + 0x30;
            command[3]  = 'C';
            command[4]  = '!';
            error = send_command( command, 5, BLOCKING );
            if ( error ) {
                ioActive = false;
                return -1;
            }
        }
        else {
            command[2]  = num + 0x30;
            command[3]  = '!';
            error = send_command( command, 4, BLOCKING );
            if ( error ) {
                ioActive = false;
                return -1;
            }
        }
    }
    serviceRequest = false;
    char m_reponse[10];
    uint32_t timeout = 0;
    int i = 0;
    
    uint8_t *s = ( uint8_t * )src;
    while ( available( ) ) *s++ = m_reponse[i++] = read( );

    if ( m_reponse[0] != sensor.address) { ioActive = false; return -2; }
    if ( ( m_reponse[1] < 0x30 ) || ( m_reponse[1] > 0x39 ) ) { ioActive = false; return -3; }
    if ( ( m_reponse[2] < 0x30 ) || ( m_reponse[2] > 0x39 ) ) { ioActive = false; return -3; }
    if ( ( m_reponse[3] < 0x30 ) || ( m_reponse[3] > 0x39 ) ) { ioActive = false; return -3; }
    timeout  = ( m_reponse[1] - 0x30 ) * 100;
    timeout += ( m_reponse[2] - 0x30 ) * 10;
    timeout += ( m_reponse[3] - 0x30 );
    timeout *= 1000;
    timeout += 10;
    
    elapsedMillis time;
    time = 0;
    while ( !serviceRequest ) {
        if ( time >= timeout ) { ioActive = false; return -4; }
        yield( );
    }
    
    do {
        yield( );
    } while ( !( UART2_S1 & UART_S1_IDLE ) );
    
    char service_request[10];
    i = 0;
    while ( available( ) ) *s++ = service_request[i++] = read( );
    if ( service_request[0] != sensor.address ) { ioActive = false; return -5; }
    ioActive = false;
    return 0;
}

int SDI12::concurrent( const uint8_t *src, int num ) {
    if ( ioActive ) return -5;
    else ioActive = true;
    int error;
    uint8_t command[5];
    command[0] = sensor.address;
    command[1] = 'C';
    if (num < 0) {
        if (sensor.crc) {
            command[2]  = 'C';
            command[3]  = '!';
            error = send_command( command, 4, BLOCKING );
            if ( error ) {
                ioActive = false;
                return -1;
            }
        }
        else {
            command[2]  = '!';
            error = send_command( command, 3, BLOCKING );
            if ( error ) {
                ioActive = false;
                return -1;
            }
        }
    }
    else {
        if (sensor.crc) {
            command[2]  = num + 0x30;
            command[3]  = 'C';
            command[4]  = '!';
            error = send_command( command, 5, BLOCKING );
            if ( error ) {
                ioActive = false;
                return -1;
            }
        }
        else {
            command[2]  = num + 0x30;
            command[3]  = '!';
            error = send_command( command, 4, BLOCKING );
            if ( error ) {
                ioActive = false;
                return -1;
            }
        }
    }
    char c_reponse[128];
    uint32_t timeout = 0;
    int i = 0;
    uint8_t *s = ( uint8_t * )src;
    while ( available( ) ) *s++ = c_reponse[i++] = read( );
    if ( c_reponse[0] != sensor.address) { ioActive = false; return -2; }
    if ( ( c_reponse[1] < 0x30 ) || ( c_reponse[1] > 0x39 ) ) { ioActive = false; return -3; }
    if ( ( c_reponse[2] < 0x30 ) || ( c_reponse[2] > 0x39 ) ) { ioActive = false; return -3; }
    if ( ( c_reponse[3] < 0x30 ) || ( c_reponse[3] > 0x39 ) ) { ioActive = false; return -3; }
    timeout  = ( c_reponse[1] - 0x30 ) * 100;
    timeout += ( c_reponse[2] - 0x30 ) * 10;
    timeout += ( c_reponse[3] - 0x30 );
    timeout *= 1000;
    ioActive = false;
    return timeout;
}

int SDI12::continuous( const uint8_t *src, int num ) {
    if ( ioActive ) return -5;
    else ioActive = true;
    
    int error;
    
    uint8_t command[6];
    
    command[0] = sensor.address;
    command[1] = 'R';
    
    if (sensor.crc) {
        command[2]  = num + 0x30;
        command[3]  = 'C';
        command[4]  = '!';
        error = send_command( command, 5, BLOCKING );
        if ( error ) {
            ioActive = false;
            return -1;
        }
    }
    else {
        command[2]  = num + 0x30;
        command[3]  = '!';
        error = send_command( command, 4, BLOCKING );
        if ( error ) {
            ioActive = false;
            return -1;
        }
    }
    
    uint8_t *s = ( uint8_t * )src;
    while ( available( ) ) *s++ = read( );
    ioActive = false;
    return 0;
}

int SDI12::transparent( const uint8_t *command, const uint8_t *src ) {
    if ( ioActive ) return -5;
    else ioActive = true;
    int error;
    const char *p = ( const char * )command;
    int len = strlen(p);
    error = send_command(command, len, BLOCKING );
    if ( error ) {
        ioActive = false;
        return -1;
    }
    uint8_t *s = ( uint8_t * )src;
    while ( available( ) ) *s++ = read( );
    if ( command[1] == 'M' ) {
        serviceRequest = false;
        uint32_t timeout = 0;
        if ( src[0] != command[0]) { ioActive = false; return -2; }
        if ( (src[1] < 0x30) || (src[1] > 0x39) ) { ioActive = false; return -3; }
        if ( (src[2] < 0x30) || (src[2] > 0x39) ) { ioActive = false; return -3; }
        if ( (src[3] < 0x30) || (src[3] > 0x39) ) { ioActive = false; return -3; }
        timeout  = ( src[1] - 0x30 ) * 100;
        timeout += ( src[2] - 0x30 ) * 10;
        timeout += ( src[3] - 0x30 );
        timeout *= 1000;
        elapsedMillis time;
        time = 0;
        while ( !serviceRequest ) {
            if ( time >= timeout ) { serviceRequest = true; ioActive = false; return -4; }
            yield( );
        }
        do { yield( ); } while ( !(UART2_S1 & UART_S1_IDLE) );
        while ( available( ) ) *s++ = read( );
    }
    ioActive = false;
    return 0;
}

int SDI12::returnMeasurement( const uint8_t *src, int num ) {
    if ( ioActive ) return -5;
    else ioActive = true;
    int error;
    uint8_t command[6];         // var to hold send command
    command[0] = sensor.address;// append sensor address
    command[1] = 'D';           // append command syntax
    if (num < 0) {
        command[2]  = '!';
        error = send_command( command, 3, BLOCKING );
        if ( error ) {
            ioActive = false;
            return -1;
        }
    }
    else {
        command[2]  = num + 0x30;
        command[3]  = '!';
        error = send_command( command, 4, BLOCKING );
        if ( error ) {
            ioActive = false;
            return -1;
        }
    }
    uint8_t *s = ( uint8_t * )src;
    while ( available( ) ) *s++ = read( );  // copy returned data to user array
    if ( sensor.crc ) {
        CRC crc;
        error = crc.check( s );             // check the sensor returned CRC
        if ( error ) {
            ioActive = false;
            return -2;
        }
    }
    ioActive = false;
    return 0;
}
//---------------------------------------------private-----------------------------------------
int SDI12::send_command( const void *cmd, uint8_t count, uint8_t type ) {
    if ( count > TX_BUFFER_SIZE ) return -1;    // command bigger than buffer size
    const uint8_t *p = ( const uint8_t * )cmd;  // pointer to command
    commandResponse = false;                    // flag for return param from sent command
    elapsedMillis time;                         // no response timer
    int retry = 3;                              // num of retries for no response from command
    int cnt = count;                            // copy total num of bytes to be sent
    
    //-------------Serial Copy Buffer-------------
    uint32_t head = txHead;
    while ( count-- > 0 ) {
        tx_buffer[head] = *p++;
        if (++head >= TX_BUFFER_SIZE) head = 0;
    }
    txHead = head;
    //--------------------------------------------
    
    switch ( type ) {
        case BLOCKING:
            flush( );               // wait for any other commands to finish sending. "BLOCKING"
            wake_io( );             // wake the sensors
            REG->C2 = C2_TX_ACTIVE; // send command
            flush( );               // wait for any other commands to finish sending. "BLOCKING"
            startBit = false;       // set flag 'false' for recieving first byte
            time = 0;               // reset timeout
            while ( !startBit ) {
                if ( time >= 60 ) {
                    if (--retry == 0) return -2;
                    int tail = txTail;
                    if ( tail - cnt < 0 ) tail = ( tail - cnt ) + TX_BUFFER_SIZE;
                    else tail -= cnt;
                    txTail = tail;
                    transmitting = true;        // update transmitting flag
                    REG->C3 |= UART_TX_DIR_OUT; // change TX pin direction
                    REG->C2 = C2_TX_ACTIVE;     // resend command
                    flush( );                   // wait for send to complete
                    startBit = false;           // set flag
                    time = 0;                   // restart timeout
                }
                yield( );
            }
            time = 0;                                               // reset timeout
            while ( !commandResponse && ( time < 1000 ) ) yield( ); // wait for response from sensor
            do { yield( ); } while ( !(UART2_S1 & UART_S1_IDLE) );  // wait for i/o to become idle
            
            if ( time >= 1000 ) { return -3; }                      // signal any errors
            break;
            
        case NON_BLOCKING:
            transmitting = true;// update transmitting flag
            REG->C2 = C2_TX_ACTIVE;
            break;
        default:
            break;
    }
    return 0;
}

void SDI12::wake_io( void ) {
    transmitting = true;
    volatile uint32_t *config;
    __disable_irq();
    config = portConfigRegister( PIN_NUM_TX );
#ifdef KINETISK
    *portModeRegister( PIN_NUM_TX ) = 1;
#else
    *portModeRegister( PIN_NUM_TX ) |= digitalPinToBitMask( PIN_NUM_TX ); // TODO: atomic
#endif
    *config = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX( 1 );
    *SET = BITMASK;
    elapsedMicros breakDelay = 0;
    while ( breakDelay <= 12000 ) yield( );
    *TX_PIN = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX( 3 );
    REG->C3 |= UART_TX_DIR_OUT;
    elapsedMicros markDelay = 0;
    while ( markDelay <= 8333 ) yield( );
    __enable_irq();
}
//---------------------------------------------uart0_isr------------------------------------------
void uart0_isr( void ) {
    uint32_t head, tail, n;
    uint8_t c;
    
    if ( UART0_S1 & ( UART_S1_RDRF ) ) {
        if ( !startBit ) startBit = true;
        head = rxHead;
        tail = rxTail;
        n = UART0_D;
        if ( n != 0 ) {
            n &= ~0x80;
            head += 1;
            if ( head >= RX_BUFFER_SIZE ) head = 0;
            rx_buffer[head] = n;
            if ( !commandResponse && n == 0x0A ) commandResponse = true;
            else if ( !serviceRequest && n == 0x0A ) serviceRequest = true;
            rxHead = head;
        }
    }
    
    c = UART0_C2;
    if ( (c & UART_C2_TIE) && (UART0_S1 & UART_S1_TDRE) ) {
        head = txHead;
        tail = txTail;
        if ( head == tail ) UART0_C2 = C2_TX_COMPLETING;
        else {
            UART0_D = tx_buffer[tail];
            if ( ++tail >= TX_BUFFER_SIZE ) tail = 0;
        }
        txTail = tail;
    }
    
    if ( (c & UART_C2_TCIE) && (UART0_S1 & UART_S1_TC) ) {
        UART0_C2 = C2_TX_INACTIVE;
        UART0_C3 &= UART_TX_DIR_IN;
        transmitting = false;
    }
}
//---------------------------------------------uart1_isr------------------------------------------
void uart1_isr( void ) {
    uint32_t head, tail, n;
    uint8_t c;
    
    if ( UART1_S1 & (UART_S1_RDRF) ) {
        if ( !startBit ) startBit = true;
        head = rxHead;
        tail = rxTail;
        n = UART1_D;
        if ( n != 0 ) {
            n &= ~0x80;
            head += 1;
            if ( head >= RX_BUFFER_SIZE ) head = 0;
            rx_buffer[head] = n;
            if ( !commandResponse && n == 0x0A ) commandResponse = true;
            else if ( !serviceRequest && n == 0x0A ) serviceRequest = true;
            rxHead = head;
        }
    }
    
    c = UART1_C2;
    if ( (c & UART_C2_TIE) && (UART1_S1 & UART_S1_TDRE) ) {
        head = txHead;
        tail = txTail;
        if ( head == tail ) UART1_C2 = C2_TX_COMPLETING;
        else {
            UART1_D = tx_buffer[tail];
            if ( ++tail >= TX_BUFFER_SIZE ) tail = 0;
        }
        txTail = tail;
    }
    
    if ( (c & UART_C2_TCIE) && (UART1_S1 & UART_S1_TC) ) {
        UART1_C2 = C2_TX_INACTIVE;
        UART1_C3 &= UART_TX_DIR_IN;
        transmitting = false;
    }
}
//---------------------------------------------uart2_isr------------------------------------------
void uart2_isr( void ) {
    uint32_t head, tail, n;
    uint8_t c;

    if ( UART2_S1 & (UART_S1_RDRF) ) {
        if ( !startBit ) startBit = true;
        head = rxHead;
        n = UART2_D;
        if ( n != 0 ) {
            n &= ~0x80;
            head += 1;
            if ( head >= RX_BUFFER_SIZE ) head = 0;
            rx_buffer[head] = n;
            if ( !commandResponse && n == 0x0A ) commandResponse = true;
            else if ( !serviceRequest && n == 0x0A ) serviceRequest = true;
            rxHead = head;
        }
    }
    
    c = UART2_C2;
    if ( (c & UART_C2_TIE) && (UART2_S1 & UART_S1_TDRE) ) {
        head = txHead;
        tail = txTail;
        if (head == tail) UART2_C2 = C2_TX_COMPLETING;
        else {
            n = tx_buffer[tail];
            UART2_D = n;
            if ( ++tail >= TX_BUFFER_SIZE ) tail = 0;
        }
        txTail = tail;
    }
    
    if ( (c & UART_C2_TCIE) && (UART2_S1 & UART_S1_TC) ) {
        UART2_C2 = C2_TX_INACTIVE;
        UART2_C3 &= UART_TX_DIR_IN;
        transmitting = false;
    }
}
/****************************************************************************/
