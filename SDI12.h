/*
 ||
 || @file 	SDI12.h
 || @version 	2
 || @author 	Colin Duffy
 || @contact 	cmduffy@engr.psu.edu
 ||
 || @description
 || | SDI12 library for Teensy 3.0/3.1.
 || #
 ||
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

#ifndef SDI12_h
#define SDI12_h

#ifdef __cplusplus

#include "Arduino.h"
#include "utility/utils.h"
#include "utility/CRC.h"

class SDI12;

//-------------------------------------------UART----------------------------------------------
class UART {
public:
    UART( void ) :
        REG( nullptr ),
        TX_PIN( nullptr ),
        RX_PIN( nullptr ),
        SET( nullptr ),
        CLEAR( nullptr ),
        PIN_NUM_RX( 0 ),
        PIN_NUM_TX( 0 )
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
//------------------------------------------SDI12-----------------------------------------------
class SDI12 : public UART {
private:
    typedef struct {
        Stream *uart;
        uint8_t address;
        bool crc;
    } sensor_block_t;
public:
    SDI12( Stream *port, const uint8_t address, bool crc = false ) {
        sensor.uart = port;
        sensor.address = address;
        sensor.crc = crc;
        init( );
        allocateVector( );
    }
    ~SDI12                 ( void ) { releaseVector( ); }
    void begin             ( uint32_t sample_rate, char command );
    void begin             ( void );
    int  isActive          ( int address = -1 );
    int  identification    ( const char *src ) { return identification( (const uint8_t *)src ); }
    int  identification    ( const uint8_t *src );
    int  queryAddress      ( void );
    int  changeAddress     ( const uint8_t new_address );
    int  verification      ( const char *src ) { return verification( (const uint8_t *)src ); }
    int  verification      ( const uint8_t *src );
    int  measurement       ( int num = -1 ) { uint8_t s[75]; return measurement( s, num ); }
    int  measurement       ( const char *src, int num = -1 ) { return measurement( ( const uint8_t * )src, num ); }
    int  measurement       ( const uint8_t *src, int num = -1 );
    int  continuous        ( const char *src, int num = -1 ) { return continuous( ( const uint8_t * )src, num ); }
    int  continuous        ( const uint8_t *src, int num = -1 );
    int  returnMeasurement ( const char *src, int num = -1 ) { return returnMeasurement( ( const uint8_t * )src, num ); }
    int  returnMeasurement ( const uint8_t *src, int num = -1 );
    int  transparent       ( const char *command, const uint8_t *src ) { return transparent( ( uint8_t * )command, src ); }
    int  transparent       ( const uint8_t *command, const char *src ) { return transparent( command, ( uint8_t * )src ); }
    int  transparent       ( const char *command, const char *src ) { return transparent( ( uint8_t * )command, ( uint8_t * )src ); }
    int  transparent       ( const uint8_t *command, const uint8_t *src );
    int  concurrent        ( int num = -1 ) { uint8_t s[75]; return concurrent( s, num ); }
    int  concurrent        ( const char *src, int num = -1 ) { return concurrent( (const uint8_t * )src, num ); }
    int  concurrent        ( const uint8_t *src, int num = -1 ) ;
private:
    void init                    ( void );
    void allocateVector          ( void );
    void releaseVector           ( void );
    void conncurrent_handler     ( void );
    void wake_io                 ( void );
    int  send_command            ( const void *cmd, uint8_t count, uint8_t type );
    sensor_block_t sensor;
    volatile uint8_t retry;
};
//-----------------------------------------------------------------------------------------------
#endif
#endif
