/*
 ||
 || @file 		CRC.h
 || @version 	2
 || @author 	Colin Duffy, Mike Jablonski
 || @contact 	cmduffy@engr.psu.edu
 ||
 || Mike Jablonski
 || NR Systems, Inc.
 || 165 East 500 South
 || River Heights, Utah  84321
 || 435-752-4200
 || nrsys@comcast.net
 ||
 || @description
 || | SDI12 CRC library, appends or checks ascii CRC using the algorithm
 || | supplied from http://www.sdi-12.org/
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

#ifndef CRC_h
#define CRC_h

#ifdef __cplusplus

#define END_STRING          0
#define ASCII_CRC_SIZE      4
#define LINE_FEED           10
#define CARRIAGE_RETURN     13

#include "Arduino.h"

class CRC {
public:
    CRC( void );
    ~CRC( void );
    void append ( uint8_t *s );
    int  check  ( volatile void  *s );
    uint8_t asciiCRC[4];
private:
    // The maximum characters allowed in a response to D command is:
    // 75 chars in the <values> part of the the command + addr + crc1 + crc2 + crc3 +
    // carriage return + line feed = 81, + null terminator = 82
    static const int MAX_D_COMMAND_RESPONSE_SIZE = 82;
    void appendAscii    ( uint8_t *s );
    void appendCRLF     ( uint8_t *s );
    void asciiToShort   ( void );
    void compute        ( uint8_t *s );
    void toAscii        ( void );
    void extractAscii   ( uint8_t *s );
    int  hasCRLF        ( uint8_t *s );
    void removeAscii    ( uint8_t *s );
    void removeCRLF     ( uint8_t *s );
    // This is a 16 bit unsigned integer. Depending on
    // your compiler the type for an unsigned 16 bit integer
    // may be different. For example, on some compilers a
    // 16 bit unsigned integer is "unsigned int"
    // This variable, and other other instances of
    // "unsigned short" in this class, must be 16 bit
    // unsigned integers.
    unsigned short sdi12_crc;
};

#endif
#endif
