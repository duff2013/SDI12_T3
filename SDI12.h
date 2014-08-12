/*
 ||
 || @file 	SDI12.h
 || @version 	1
 || @author 	Colin Duffy
 || @contact 	cmduffy@engr.psu.edu
 ||
 || @description
 || |
 || #
 ||
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

#ifndef SDI12_h
#define SDI12_h

#ifdef __cplusplus

#include "Arduino.h"
#include "utility/utils.h"
#include "utility/CRC.h"

//-----------------------------------------------------------------------------------------------
class UART {
public:
    UART( void ) :
        REG(nullptr),
        TX_PIN(nullptr),
        RX_PIN(nullptr),
        SET(nullptr),
        CLEAR(nullptr),
        PIN_NUM_RX(NULL),
        PIN_NUM_TX(NULL)
    {

    }
private:
    int  peek      ( void );
    int  read      ( void );
    int  available ( void );
    int  end       ( void );
    void clear     ( void );
    void flush     ( void );
    
    friend class      SDI12;
    KINETISK_UART_t   *REG;
    volatile uint32_t *TX_PIN;
    volatile uint32_t *RX_PIN;
    volatile uint8_t  SCGC4;
    volatile uint32_t *SET;
    volatile uint32_t *CLEAR;
    volatile uint32_t BITMASK;
    uint8_t           PIN_NUM_RX;
    uint8_t           PIN_NUM_TX;
};
//-----------------------------------------------------------------------------------------------
class SDI12 : public UART {
    typedef struct __attribute__((packed)) {
        Stream *uart;
        uint8_t address;
        bool crc;
    } sdi12_t;
public:
    SDI12( void );
    SDI12( Stream *port, char address, bool crc = false ) ;
    ~SDI12( void ) {  }
    //SDI12 & operator = (const SDI12 &rhs) { SDI12 = rhs.SDI12; return *this; }
    void begin             ( uint32_t sample_rate, char command );
    void begin             ( void );
    
    bool  isActive          ( int address = -1 );
    
    bool identification    ( const char *src ) { identification( (const uint8_t *)src ); }
    bool identification    ( const uint8_t *src );
    
    int  queryAddress      ( void );
    
    int  changeAddress     ( uint8_t new_address );
    
    bool verification      ( const char *src ) { verification( (const uint8_t *)src ); }
    bool verification      ( const uint8_t *src );
    
    bool measurement       ( int num = -1 ) { uint8_t s[75]; measurement( s, num ); }
    bool measurement       ( const char *src, int num = -1 ) { measurement( (const uint8_t *)src, num ); }
    bool measurement       ( const uint8_t *src, int num = -1 );
    
    bool concurrent        ( int num = -1 ) { uint8_t s[75]; concurrent( s, num ); }
    bool concurrent        ( const char *src, int num = -1 ) { concurrent( (const uint8_t *)src, num ); }
    bool concurrent        (  const uint8_t *src, int num  );
    
    bool continuous        ( const char *src, int num = -1 ) { continuous( (const uint8_t *)src, num ); }
    bool continuous        ( const uint8_t *src, int num = -1 );
    
    bool returnMeasurement ( const char *src, int num = -1 ) { returnMeasurement( (const uint8_t *)src, num ); }
    bool returnMeasurement ( const uint8_t *src, int num = -1 );
    
    bool transparent       ( const char *command, const uint8_t *src ) { transparent( (uint8_t*)command, src ); }
    bool transparent       ( const uint8_t *command, const char *src ) { transparent( command, (uint8_t*)src ); }
    bool transparent       ( const char *command, const char *src ) { transparent( (uint8_t*)command, (uint8_t*)src ); }
    bool transparent       ( const uint8_t *command, const uint8_t *src );
private:
    static SDI12* STATIC;
    static SDI12* TMP[60];
    sdi12_t       sensor;
    bool send_command            ( const void *cmd, uint8_t count, uint8_t type );
    void wake_io_blocking        ( void );
    bool wake_io_non_blocking    ( void );
    static void io_break         ( void );
    static void io_mark          ( void );
    static void concurrentHandle ( void );
};
//-----------------------------------------------------------------------------------------------
#endif
#endif
