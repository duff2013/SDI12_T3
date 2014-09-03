/*
 ||
 || @file 	SDI12.h
 || @version 	1
 || @author 	Colin Duffy
 || @contact 	cmduffy@engr.psu.edu
 ||
 || @description
 || | SDI12 library for Teensy 3.0/3.1.
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

class SDI12;

//---------------------------------------------------------------------------------------------
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
//---------------------------------------------------------------------------------------------
class SDI12 : public UART {
    typedef struct __attribute__((packed)) {
        Stream *uart;
        uint32_t timeout;
        uint8_t address;
        bool crc;
    } sdi12_t;
    
    typedef struct {
        uint8_t conncurrentCommand[5];
        uint8_t conncurentData[82];
        volatile uint8_t  cmdSize;
        volatile uint32_t timeout;
        volatile uint32_t timepos;
        SDI12 *Class;
    } sensor_block_t;
    
public:
    SDI12( void ) { }
    
    SDI12( Stream *port, char address, bool crc = false ) {
        sensor.uart = port;
        sensor.address = address;
        sensor.crc = crc;
        init( );
        allocateVector( );
    }
    
    ~SDI12( void ) {
        releaseVector( );
    }
    
    SDI12 & operator () ( SDI12 &ref1 ) {
        sensor_block[0].Class = &ref1;
        sensor_block[1].Class = nullptr;
        sensor_block[2].Class = nullptr;
        sensor_block[3].Class = nullptr;
        
        sensor_block[0].conncurrentCommand[0] = ref1.sensor.address;
        sensor_block[0].conncurrentCommand[1] = 'C';
        if (ref1.sensor.crc) sensor_block[0].conncurrentCommand[2] = 'C';
        
        number_of_registered_sensors = 1;
    }
    
    SDI12 & operator () ( SDI12 &ref1, SDI12 &ref2 ) {
        sensor_block[0].Class = &ref1;
        sensor_block[1].Class = &ref2;
        sensor_block[2].Class = nullptr;
        sensor_block[3].Class = nullptr;
        
        sensor_block[0].conncurrentCommand[0] = ref1.sensor.address;
        sensor_block[0].conncurrentCommand[1] = 'C';
        if (ref1.sensor.crc) sensor_block[0].conncurrentCommand[2] = 'C';
        
        sensor_block[1].conncurrentCommand[0] = ref2.sensor.address;
        sensor_block[1].conncurrentCommand[1] = 'C';
        if (ref2.sensor.crc) sensor_block[1].conncurrentCommand[2] = 'C';
        
        number_of_registered_sensors = 2;
    }
    
    SDI12 & operator () ( SDI12 &ref1, SDI12 &ref2, SDI12 &ref3 ) {
        sensor_block[0].Class = &ref1;
        sensor_block[1].Class = &ref2;
        sensor_block[2].Class = &ref3;
        sensor_block[3].Class = nullptr;
        
        sensor_block[0].conncurrentCommand[0] = ref1.sensor.address;
        sensor_block[0].conncurrentCommand[1] = 'C';
        if (ref1.sensor.crc) sensor_block[0].conncurrentCommand[2] = 'C';
        
        sensor_block[1].conncurrentCommand[0] = ref2.sensor.address;
        sensor_block[1].conncurrentCommand[1] = 'C';
        if (ref1.sensor.crc) sensor_block[1].conncurrentCommand[2] = 'C';
        
        sensor_block[2].conncurrentCommand[0] = ref3.sensor.address;
        sensor_block[2].conncurrentCommand[1] = 'C';
        if (ref1.sensor.crc) sensor_block[2].conncurrentCommand[2] = 'C';
        
        number_of_registered_sensors = 3;
    }
    
    SDI12 & operator () ( SDI12 &ref1, SDI12 &ref2, SDI12 &ref3, SDI12 &ref4 ) {
        sensor_block[0].Class = &ref1;
        sensor_block[1].Class = &ref2;
        sensor_block[2].Class = &ref3;
        sensor_block[3].Class = &ref4;
        
        sensor_block[0].conncurrentCommand[0] = ref1.sensor.address;
        sensor_block[0].conncurrentCommand[1] = 'C';
        if (ref1.sensor.crc) sensor_block[0].conncurrentCommand[2] = 'C';
        
        sensor_block[1].conncurrentCommand[0] = ref2.sensor.address;
        sensor_block[1].conncurrentCommand[1] = 'C';
        if (ref1.sensor.crc) sensor_block[1].conncurrentCommand[2] = 'C';
        
        sensor_block[2].conncurrentCommand[0] = ref3.sensor.address;
        sensor_block[2].conncurrentCommand[1] = 'C';
        if (ref1.sensor.crc) sensor_block[2].conncurrentCommand[2] = 'C';
        
        sensor_block[3].conncurrentCommand[0] = ref4.sensor.address;
        sensor_block[3].conncurrentCommand[1] = 'C';
        if (ref1.sensor.crc) sensor_block[3].conncurrentCommand[2] = 'C';
        
        number_of_registered_sensors = 4;
        //Serial.println("ref4");
    }
    
    /*const SDI12 & operator = ( const SDI12 &rhs ) {
        this->sensor.uart    = rhs.sensor.uart;
        this->sensor.timeout = rhs.sensor.timeout;
        this->sensor.address = rhs.sensor.address;
        this->sensor.crc     = rhs.sensor.crc;
        this->REG            = rhs.REG;
        this->RX_PIN         = rhs.RX_PIN;
        this->TX_PIN         = rhs.TX_PIN;
        this->SET            = rhs.SET;
        this->CLEAR          = rhs.CLEAR;
        this->BITMASK        = rhs.BITMASK;
        this->PIN_NUM_RX     = rhs.PIN_NUM_RX;
        this->PIN_NUM_TX     = rhs.PIN_NUM_TX;
        //sensor_block[class_count++].Class = this;
        return *this;
    }*/
    
    void begin             ( uint32_t sample_rate, char command );
    void begin             ( void );
    int  isActive          ( int address = -1 );
    int  identification    ( const char *src ) { return identification( (const uint8_t *)src ); }
    int  identification    ( const uint8_t *src );
    int  queryAddress      ( void );
    int  changeAddress     ( uint8_t new_address );
    int  verification      ( const char *src ) { return verification( (const uint8_t *)src ); }
    int  verification      ( const uint8_t *src );
    int  measurement       ( int num = -1 ) { uint8_t s[75]; return measurement( s, num ); }
    int  measurement       ( const char *src, int num = -1 ) { return measurement( (const uint8_t *)src, num ); }
    int  measurement       ( const uint8_t *src, int num = -1 );
    
    //bool concurrent        ( int num = -1 ) { uint8_t s[75]; return concurrent( s, num ); }
    //bool concurrent        ( const char *src, int num = -1 ) { return concurrent( (const uint8_t *)src, num ); }
    //bool concurrent        ( const uint8_t *src, int num  );
    
    int  continuous        ( const char *src, int num = -1 ) { return continuous( (const uint8_t *)src, num ); }
    int  continuous        ( const uint8_t *src, int num = -1 );
    int  returnMeasurement ( const char *src, int num = -1 ) { return returnMeasurement( (const uint8_t *)src, num ); }
    int  returnMeasurement ( const uint8_t *src, int num = -1 );
    int  transparent       ( const char *command, const uint8_t *src ) { return transparent( (uint8_t*)command, src ); }
    int  transparent       ( const uint8_t *command, const char *src ) { return transparent( command, (uint8_t*)src ); }
    int  transparent       ( const char *command, const char *src ) { return transparent( (uint8_t*)command, (uint8_t*)src ); }
    int  transparent       ( const uint8_t *command, const uint8_t *src );
    
    static int concurrent ( int sen_arg1 = 0, int sen_arg2 = 0, int sen_arg3 = 0, int sen_arg4 = 0 );
private:
    static SDI12 *CURRENT_CLASS;
    static int number_of_registered_sensors;
    static sensor_block_t sensor_block[4];
    sdi12_t sensor;
    
    void init                    ( void );
    void allocateVector          ( void );
    void releaseVector           ( void );
    int  send_command            ( const void *cmd, uint8_t count, uint8_t type );
    void wake_io_blocking        ( void );
    int  wake_io_non_blocking    ( void );
    
    static void io_break         ( void );
    static void io_mark          ( void );
    static void concurrentHandle ( void );
    static void concurrentData   ( void );
};
//-----------------------------------------------------------------------------------------------
#endif
#endif
