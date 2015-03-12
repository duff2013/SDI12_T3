/*
 ||
 || @file 	SDI12.h
 || @version 	3
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
    
    SDI12( Stream *port ) {
        sensor.uart = port;
        init( );
        allocateVector( );
    }
    
    SDI12 & operator = ( const SDI12 &rhs ) {
        if ( this != &rhs ) {
            sensor.uart = rhs.sensor.uart;
            sensor.address = rhs.sensor.address;
            sensor.crc = rhs.sensor.crc;
        }
        return *this;
    };
    
    ~SDI12                 ( void ) { releaseVector( ); }
    int  isActive          ( int address = -1 );
    int  identification    ( volatile void *src );
    int  queryAddress      ( void );
    int  changeAddress     ( const uint8_t new_address );
    int  verification      ( volatile void *src );
    int  measurement       ( int num = -1 ) { uint8_t src[10]; return measurement( src, num ); }
    int  measurement       ( volatile void *src, int num = -1 );
    int  continuous        ( volatile void *src, int num = -1 );
    int  returnMeasurement ( volatile void *src, int num = -1 );
    int  concurrent        ( volatile void *src, int num = -1 );
    int  transparent       ( const void *command, volatile void *src );
private:
    void init               ( void );
    void allocateVector     ( void );
    void releaseVector      ( void );
    void conncurrent_handler( void );
    void wake_io            ( void );
    int  send_command       ( const void *cmd, uint8_t count, uint8_t type );
    sensor_block_t   sensor;
    volatile uint8_t retry;
};
//-----------------------------------------------------------------------------------------------
#endif// end __cplusplus
#endif
