/*
 ||
 || @file 	SDI12.cpp
 || @version 	1
 || @author 	Colin Duffy
 || @contact 	cmduffy@engr.psu.edu
 || @license
 || | Copyright (c) 2014 Colin Duffy
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

IntervalTimer     ioTimer;

volatile bool     serviceRequest;
volatile bool     commandNotDone;
//volatile bool     nonBlockSend;
//volatile bool     nonBlockRecieve;
//volatile bool     nonBlockGetData;
volatile bool     activeAddress[60];

volatile int spot = 0;

volatile uint16_t txHead;
volatile uint16_t txTail;
volatile uint16_t rxHead;
volatile uint16_t rxTail;

volatile bool     sending;
volatile bool     ioActive;
volatile bool     transmitting;

static volatile uint8_t tx_buffer[TX_BUFFER_SIZE];
static volatile uint8_t rx_buffer[RX_BUFFER_SIZE];

static void uart0_isr( void );
static void uart1_isr( void );
static void uart2_isr( void );
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
    
}

void UART::flush( void ) {
    while ( transmitting ) ;
}

void UART::clear( void ) {
    
}

int UART::peek( void ) {
    
}
/****************************************************************************/
/*                                 SDI12                                    */
/****************************************************************************/
SDI12  *SDI12::STATIC;
SDI12  *SDI12::TMP[];

SDI12::SDI12( Stream *port, char address, bool crc ) {
    pinMode(13, OUTPUT);
    pinMode(17, OUTPUT);
    pinMode(18, OUTPUT);
    pinMode(19, OUTPUT);
    pinMode(20, OUTPUT);
    
    sensor.uart = port;
    sensor.address = address;
    sensor.crc = crc;
    
    for (int i = 0; i < 60; i++) {
        activeAddress[i] = false;
    }
    dd
    int ascii_numbers    = (address < 0x30) || (address > 0x39);
    int ascii_upper_case = (address < 0x41) || (address > 0x5A);
    int ascii_lower_case = (address < 0x61) || (address > 0x7A);
    
    if ( ascii_numbers ) {
        TMP[address-0x30] = this;
    }
    else if (ascii_upper_case) {
        TMP[address-0x41] = this;
    }
    else if (ascii_lower_case) {
        TMP[address-0x61] = this;
    }
    else {
        return;
    }
    
    RUN_ONCE_BEGIN;
    spot           = 0;
    txHead         = 0;
    txTail         = 0;
    rxHead         = 0;
    rxTail         = 0;
    sending        = false;
    ioActive       = false;
    transmitting   = false;
    serviceRequest = false;
    commandNotDone = false;
    RUN_ONCE_END;
    
    if ( port == &Serial1 ) {
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
    else if ( port == &Serial2 ) {
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
    else if ( port == &Serial3 ) {
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

void SDI12::begin( void ) {
    Serial.printf("Address: %c\n",TMP[sensor.address]->sensor.address);
}

bool SDI12::isActive( int address ) {
    bool error;
    
    uint8_t command[3];
    if ( address == -1 ) command[0] = sensor.address;
    else command[0] = address;
    command[1] = '!';
    
    error = send_command( command, 2, BLOCKING );
    if ( error ) return error;
    
    uint8_t p[5];
    int i = 0;
    while ( available( ) ) p[i++] = read( );
    if ( p[0] == sensor.address ) return false;
    else return true;
}

bool SDI12::identification( const uint8_t *src ) {
    bool error;
    
    uint8_t command[3];
    command[0] = sensor.address;
    command[1] = 'I';
    command[2] = '!';
    
    error = send_command(command, 3, BLOCKING );
    if ( error ) return error;
    
    uint8_t *s = ( uint8_t * )src;
    while ( available( ) ) *s++ = read( );
    
    return false;
}

int SDI12::changeAddress( uint8_t new_address ) {
    if ( new_address == sensor.address ) return -1;
    bool error;
    
    int ascii_numbers    = (new_address < 0x30) || (new_address > 0x39);
    int ascii_upper_case = (new_address < 0x41) || (new_address > 0x5A);
    int ascii_lower_case = (new_address < 0x61) || (new_address > 0x7A);

    if (ascii_numbers & ascii_lower_case & ascii_upper_case) {
        //Serial.println("Address out of range");
        return -1;
    }
    
    int address;
    address = isActive( );
    
    if ( address == -1 ) {
        Serial.println("Bad Address");
        return -1;
    }
    
    uint8_t command[4];
    command[0] = sensor.address;
    command[1] = 'A';
    command[2] = new_address;
    command[3] = '!';
    
    error = send_command(command, 4, BLOCKING );
    if ( error ) return -1;
    
    int i = 0;
    char p[5];
    while ( available( ) ) p[i++] = read( );
    
    if ( p[0] == new_address ) {
        sensor.address = p[0];
        return p[0];
    }
    else sensor.address;
}

int SDI12::queryAddress( void ) {
    bool error;
    
    uint8_t command[2];
    command[0] = '?';
    command[1] = '!';
    
    error = send_command( command, 2, BLOCKING );
    if ( error ) return -1;
    
    int i = 0;
    char p[5];
    while ( available( ) ) p[i++] = read( );
    
    return p[0];
}

bool SDI12::verification( const uint8_t *src ) {
    uint8_t *s = ( uint8_t * )src;
    bool error;
    
    uint8_t command[6];
    command[0] = sensor.address;
    command[1] = 'V';
    command[2] = '!';
    error = send_command( command, 3, BLOCKING );
    if ( error ) return error;
    
    int timeout = 0;
    int i = 0;
    
    char v_reponse[10];
    serviceRequest = true;
    while ( available( ) ) *s++ = v_reponse[i++] = read( );
    
    if ( v_reponse[0] != sensor.address) return true;
    if ( 0x30 > v_reponse[1] > 0x39 ) return true;
    if ( 0x30 > v_reponse[2] > 0x39 ) return true;
    if ( 0x30 > v_reponse[3] > 0x39 ) return true;
    timeout  = (v_reponse[1] - 0x30) * 100;
    timeout += (v_reponse[2] - 0x30) * 10;
    timeout += (v_reponse[3] - 0x30);
    timeout *= 1000;
    timeout += 10;
    
    elapsedMillis time;
    time = 0;
    while ( serviceRequest ) {
        if ( time >= timeout ) { serviceRequest = false; return true; }
    }
    char service_request[10];
    i = 0;
    while ( available( ) ) *s++ = service_request[i++] = read( );
    if ( service_request[0] != sensor.address) return true;
    return false;
}

bool SDI12::measurement( const uint8_t *src, int num ) {
    bool error;
    uint8_t *s = ( uint8_t * )src;
    uint8_t command[5];
    command[0] = sensor.address;
    command[1] = 'M';
    if (num < 0) {
        if (sensor.crc) {
            command[2]  = 'C';
            command[3]  = '!';
            error = send_command( command, 4, BLOCKING );
            if ( error ) return error;
        }
        else {
            command[2]  = '!';
            error = send_command( command, 3, BLOCKING );
            if ( error ) return error;
        }
    }
    else {
        if (sensor.crc) {
            command[2]  = num + 0x30;
            command[3]  = 'C';
            command[4]  = '!';
            error = send_command( command, 5, BLOCKING );
            if ( error ) return error;
        }
        else {
            command[2]  = num + 0x30;
            command[3]  = '!';
            error = send_command( command, 4, BLOCKING );
            if ( error ) return error;
        }
    }
    serviceRequest = true;
    char m_reponse[10];
    int timeout = 0;
    int i = 0;
    
    while ( available( ) ) *s++ = m_reponse[i++] = read( );
    
    //Serial.printf("m_responce: %s", m_reponse);
    if ( m_reponse[0] != sensor.address) return true;
    if ( 0x30 > m_reponse[1] > 0x39 ) return true;
    if ( 0x30 > m_reponse[2] > 0x39 ) return true;
    if ( 0x30 > m_reponse[3] > 0x39 ) return true;
    timeout  = (m_reponse[1] - 0x30) * 100;
    timeout += (m_reponse[2] - 0x30) * 10;
    timeout += (m_reponse[3] - 0x30);
    timeout *= 1000;
    timeout += 10;
    
    elapsedMillis time;
    time = 0;
    while ( serviceRequest ) {
        if ( time >= timeout ) { serviceRequest = false; return true; }
    }
    char service_request[10];
    i = 0;
    while ( available( ) ) *s++ = service_request[i++] = read( );
    if ( service_request[0] != sensor.address) return true;
    //Serial.printf("Service Request: %s\n", service_request);
    return false;
}

bool SDI12::concurrent( const uint8_t *src, int num ) {
    bool error;
    uint8_t *s = ( uint8_t * )src;
    activeAddresses[sensor.address] = true;
    uint8_t command[5];
    command[0] = sensor.address;
    command[1] = 'C';
    if (num < 0) {
        if (sensor.crc) {
            command[2]  = 'C';
            command[3]  = '!';
            error = send_command( command, 4, NON_BLOCKING );
            if ( error ) return error;
        }
        else {
            command[2]  = '!';
            error = send_command( command, 3, NON_BLOCKING );
            if ( error ) return error;
        }
    }
    else {
        if (sensor.crc) {
            command[2]  = num + 0x30;
            command[3]  = 'C';
            command[4]  = '!';
            error = send_command( command, 5, NON_BLOCKING );
            if ( error ) return error;
        }
        else {
            command[2]  = num + 0x30;
            command[3]  = '!';
            error = send_command( command, 4, NON_BLOCKING );
            if ( error ) return error;
        }
    }
}

bool SDI12::continuous( const uint8_t *src, int num ) {
    bool error;
    
    uint8_t command[6];
    command[0] = sensor.address;
    command[1] = 'R';
    
    if (sensor.crc) {
        command[2]  = num + 0x30;
        command[3]  = 'C';
        command[4]  = '!';
        error = send_command( command, 5, BLOCKING );
        if ( error ) return error;
    }
    else {
        command[2]  = num + 0x30;
        command[3]  = '!';
        error = send_command( command, 4, BLOCKING );
        if ( error ) return error;
    }
    
    uint8_t *s = ( uint8_t * )src;
    while ( available( ) ) *s++ = read( );
    return false;
}

bool SDI12::transparent( const uint8_t *command, const uint8_t *src ) {
    bool error;
    const char *p = ( const char * )command;
    int len = strlen(p);
    
    error = send_command(command, len, BLOCKING );
    if ( error ) return error;
    
    uint8_t *s = ( uint8_t * )src;
    while ( available( ) ) *s++ = read( );
    
    if ( command[1] == 'M' ) {
        serviceRequest = true;
        int timeout = 0;
        if ( src[0] != command[0])  return true;
        if ( 0x30 > src[1] > 0x39 ) return true;
        if ( 0x30 > src[2] > 0x39 ) return true;
        if ( 0x30 > src[3] > 0x39 ) return true;
        timeout  = (src[1] - 0x30) * 100;
        timeout += (src[2] - 0x30) * 10;
        timeout += (src[3] - 0x30);
        timeout *= 1000;
        int t = timeout;
        elapsedMillis time;
        time = 0;
        while ( serviceRequest ) {
            if ( time >= timeout ) { serviceRequest = false; return true; }
        }
        while ( available( ) ) *s++ = read( );
    }
    return false;
}

bool SDI12::returnMeasurement( const uint8_t *src, int num ) {
    bool error;
    uint8_t command[6];
    command[0] = sensor.address;
    command[1] = 'D';
    
    if (num < 0) {
        command[2]  = '!';
        error = send_command( command, 3, BLOCKING );
        if ( error ) return error;
    }
    else {
        command[2]  = num + 0x30;
        command[3]  = '!';
        error = send_command( command, 4, BLOCKING );
        if ( error ) return error;
    }
    
    uint8_t *s = ( uint8_t * )src;
    while ( available( ) ) *s++ = read( );
    if ( sensor.crc ) {
        CRC crc;
        error = crc.check( s );
        if ( !error ) {
            //Serial.println("CRC Good");
            return error;
        }
        else {
            //Serial.println("CRC Bad");
            return error;
        }
    }
    error = false;
    return error;
}
//---------------------------------------------private---------------------------------------------
bool SDI12::send_command( const void *cmd, uint8_t count, uint8_t type ) {
    const uint8_t *p = ( const uint8_t * )cmd;
    commandNotDone = true;
    elapsedMillis time;
    int head = txHead;
    
    while ( count-- > 0 ) {
        tx_buffer[head] = *p++;
        if (++head >= TX_BUFFER_SIZE) head = 0;
    }
    txHead = head;
    
    switch ( type ) {
        case BLOCKING:
            wake_io_blocking( );
            REG->C2 = C2_TX_ACTIVE;
            flush( );
            time = 0;
            while ( commandNotDone && (time < 1000) ) ;
            if ( time >= 1000 ) { commandNotDone = false; return true; }
            break;
        case NON_BLOCKING:
            STATIC = this;
            wake_io_non_blocking( );
            break;
        default:
            break;
    }
    return false;
}

void SDI12::wake_io_blocking( void ) {
    transmitting = true;
    volatile uint32_t *config;
    config = portConfigRegister( PIN_NUM_TX );
    *portModeRegister( PIN_NUM_TX ) = 1;
    *config = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX( 1 );
    *SET = BITMASK;
    delayMicroseconds( 12000 );
    *TX_PIN = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX( 3 );
    REG->C3 |= UART_TX_DIR_OUT;
    delayMicroseconds( 8333 );
}

bool SDI12::wake_io_non_blocking( void ) {
    //STATIC = this;
    transmitting = true;
    volatile uint32_t *config;
    config = portConfigRegister( PIN_NUM_TX );
    *portModeRegister( PIN_NUM_TX ) = 1;
    *config = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX( 1 );
    *SET = BITMASK;
    bool error = ioTimer.begin( io_break, 12000 );
    return !error;
}

void SDI12::io_break( void ) {
    *STATIC->TX_PIN = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX( 3 );
    STATIC->REG->C3 |= UART_TX_DIR_OUT;
    ioTimer.begin( io_mark, 8333 );
}

void SDI12::io_mark( void ) {
    //ioTimer.end( );
    ioTimer.begin( concurrentHandle, 8333 );
    if ( !ioActive ) STATIC->REG->C2 = C2_TX_ACTIVE;
    
    digitalWrite(13, HIGH);
}

void SDI12::concurrentHandle( void ) {
    UART uart;
    int i = 0;
    char tmp[75];
    int timeout;
    uint8_t measureCmd[4];
    
    switch (spot) {
        case 0:
            if ( !transmitting ) {
                Serial.println("case 0");
                spot = 1;
            }
            break;
        case 1:
            if ( !commandNotDone  ) {
                Serial.println("case 1");
                spot = 2;
            }
            break;
        case 2:
            ioTimer.end( );
            i=0;
            while ( uart.available( ) ) {
                tmp[i++] = uart.read( );
            }
            //if ( src[0] != command[0])  return true;
            //if ( 0x30 > src[1] > 0x39 ) return true;
            //if ( 0x30 > src[2] > 0x39 ) return true;
            //if ( 0x30 > src[3] > 0x39 ) return true;
            timeout  = (tmp[1] - 0x30) * 100;
            timeout += (tmp[2] - 0x30) * 10;
            timeout += (tmp[3] - 0x30);
            timeout *= 1000000;
            Serial.printf("case 2 | timeout: %i\n", timeout);
            spot = 3;
            ioTimer.begin( concurrentHandle, timeout );
            break;
        case 3:
            ioTimer.end( );
            measureCmd[0] = STATIC->sensor.address;
            measureCmd[1] = 'D';
            measureCmd[2] = '0';
            measureCmd[3] = '!';
            Serial.printf("case 3 | cmd: %s\n", measureCmd);
            STATIC->send_command(measureCmd, 4, NON_BLOCKING);
            spot = 4;
            break;
        case 4:
            if ( !transmitting ) {
                Serial.println("case 4");
                spot = 5;
            }
            break;
        case 5:
            if ( !commandNotDone  ) {
                Serial.println("case 5");
                spot = 6;
            }
            break;
        case 6:
            ioTimer.end( );
            i=0;
            while ( uart.available( ) ) {
                tmp[i++] = uart.read( );
            }
            Serial.print("case 6 | ");
            Serial.write(tmp,i);
            ioActive = false;
            spot = 0;
            break;
        default:
            break;
    }
    //__enable_irq();
}
//---------------------------------------------uart0_isr------------------------------------------
void uart0_isr( void ) {
    uint32_t head, tail, n;
    uint8_t c;
    
    if ( UART0_S1 & ( UART_S1_RDRF ) ) {
        head = rxHead;
        tail = rxTail;
        n = UART0_D;
        if ( n != 0 ) {
            n &= ~0x80;
            head += 1;
            if ( head >= RX_BUFFER_SIZE ) head = 0;
            rx_buffer[head] = n;
            if ( commandNotDone && n == 0x0A ) commandNotDone = false;
            else if ( serviceRequest && n == 0x0A ) serviceRequest = false;
            rxHead = head;
        }
    }
    
    c = UART0_C2;
    if ( (c & UART_C2_TIE) && (UART0_S1 & UART_S1_TDRE) ) {
        head = txHead;
        tail = txTail;
        if (head == tail) UART0_C2 = C2_TX_COMPLETING;
        else {
            UART0_D = tx_buffer[tail];
            if ( ++tail >= TX_BUFFER_SIZE ) tail = 0;
        }
        txTail = tail;
    }
    
    if ((c & UART_C2_TCIE) && (UART0_S1 & UART_S1_TC)) {
        UART0_C2 = C2_TX_INACTIVE;
        UART0_C3 &= ~UART_C3_TXDIR;
        transmitting = false;
    }
}
//---------------------------------------------uart1_isr------------------------------------------
void uart1_isr( void ) {
    uint32_t head, tail, n;
    uint8_t c;
    
    if ( UART1_S1 & ( UART_S1_RDRF ) ) {
        head = rxHead;
        tail = rxTail;
        n = UART1_D;
        if ( n != 0 ) {
            n &= ~0x80;
            head += 1;
            if ( head >= RX_BUFFER_SIZE ) head = 0;
            rx_buffer[head] = n;
            if ( commandNotDone && n == 0x0A ) commandNotDone = false;
            else if ( serviceRequest && n == 0x0A ) serviceRequest = false;
            rxHead = head;
        }
    }
    
    c = UART1_C2;
    if ( (c & UART_C2_TIE) && (UART1_S1 & UART_S1_TDRE) ) {
        head = txHead;
        tail = txTail;
        if (head == tail) UART1_C2 = C2_TX_COMPLETING;
        else {
            UART1_D = tx_buffer[tail];
            if ( ++tail >= TX_BUFFER_SIZE ) tail = 0;
        }
        txTail = tail;
    }
    
    if ((c & UART_C2_TCIE) && (UART1_S1 & UART_S1_TC)) {
        UART1_C2 = C2_TX_INACTIVE;
        UART1_C3 &= ~UART_C3_TXDIR;
        transmitting = false;
    }
}
//---------------------------------------------uart2_isr------------------------------------------
void uart2_isr( void ) {
    //digitalWriteFast(13, HIGH);
    uint32_t head, tail, n;
    uint8_t c;

    if ( UART2_S1 & ( UART_S1_RDRF ) ) {
        head = rxHead;
        n = UART2_D;
        if ( n != 0 ) {
            n &= ~0x80;
            head += 1;
            if ( head >= RX_BUFFER_SIZE ) head = 0;
            rx_buffer[head] = n;
            if ( commandNotDone && n == 0x0A ) {
                commandNotDone = false;
            }
            else if ( serviceRequest && n == 0x0A ) serviceRequest = false;
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
    
    if ((c & UART_C2_TCIE) && (UART2_S1 & UART_S1_TC)) {
        UART2_C2 = C2_TX_INACTIVE;
        UART2_C3 &= UART_TX_DIR_IN;
        transmitting = false;
    }
    //digitalWriteFast(13, LOW);
}
/****************************************************************************/
