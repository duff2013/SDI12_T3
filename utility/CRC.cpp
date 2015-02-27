/*
 ||
 || @file 		CRC.h
 || @version 	1
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

#include "CRC.h"


CRC::CRC( void ) {
    sdi12_crc = 0;
    strcpy( ( char * )asciiCRC, "" );
}

CRC::~CRC( void ) {
    
}

void CRC::append( uint8_t *s ) {
    // This public function computes and appends a CRC to the end of a response to
    // an SDI-12 D Command.
    removeCRLF ( s );
    compute    ( s );
    
    toAscii    (   );
    appendAscii( s );
    appendCRLF ( s );
}

//----------------------------------------------public------------------------------------------------

int CRC::check( uint8_t *s ) {
    // This public function checks the CRC found at the end of the response to an
    // SDI-12 D command.
    unsigned short receivedCRC;
    unsigned short expectedCRC;
    
    // get the CRC that was received
    removeCRLF  ( s );
    extractAscii( s );
    asciiToShort(   );
    
    receivedCRC = sdi12_crc;
    // compute the expected CRC
    
    removeAscii( s );
    compute    ( s );
    
    expectedCRC = sdi12_crc;
    if ( receivedCRC == expectedCRC ) return( 0 );
    else return(1);
}

//---------------------------------------------private----------------------------------------------

void CRC::appendAscii( uint8_t *s ) {
    strcat( ( char * )s , ( char * )asciiCRC );
}

void CRC::appendCRLF( uint8_t *s ) {
    int i  = strlen( ( char * )s );
    s[i++] = CARRIAGE_RETURN;
    s[i++] = LINE_FEED;
    s[i]   = END_STRING;
}

void CRC::asciiToShort( ) {
    // This function converts a 3 character ascii crc into a 16 bit unsigned
    // integer. This logic is the inverse of CRCToAscii (see below).
    unsigned short tempWord;
    if ( strlen(( char * )asciiCRC ) == 3 ) {
        tempWord  = ( ( asciiCRC[0] & 0x00bf ) << 12 );
        tempWord  = tempWord | ( ( asciiCRC[1] & 0x00bf ) << 6 );
        tempWord  = tempWord | ( asciiCRC[2] & 0x00bf );
        sdi12_crc = tempWord;
    }
    else sdi12_crc = 0;
}

void CRC::compute( uint8_t *s ) {
    // This function computes the CRC using the algorithm found in paragraph
    // 4.4.12.1 of the SDI-12 specification.
    int i;
    int j;
    int count;
    sdi12_crc = 0;
    count = strlen( ( char * )s );
    if ( count > 0 ) {
        count--;
        for ( i=0; i <= count; i++ ) {
            sdi12_crc = sdi12_crc ^ s[i];
            for ( j=1; j<=8; j++ ) {
                if ( ( sdi12_crc & 0x0001 ) == 0x0001 ) {
                    sdi12_crc = sdi12_crc >> 1;
                    sdi12_crc = sdi12_crc ^ 0xA001;
                }
                else sdi12_crc = sdi12_crc >> 1;
            }
        }
    }
}

void CRC::toAscii( ) {
    // This function encodes the CRC as three ascii characters, using the
    // algorithm found in paragraph 4.4.12.2 of the SDI-12 specification.
    strcpy( (char*)asciiCRC, "" );
    asciiCRC[0] = 0x40 | ( sdi12_crc >> 12 );
    asciiCRC[1] = 0x40 | ( ( sdi12_crc >> 6 ) & 0x3f );
    asciiCRC[2] = 0x40 | ( sdi12_crc & 0x3f );
    asciiCRC[3] = END_STRING;
}

void CRC::extractAscii( uint8_t *s ) {
    int i;
    i = strlen( ( char * )s );
    if (i >= 3) {
        asciiCRC[0] = s[i-3];
        asciiCRC[1] = s[i-2];
        asciiCRC[2] = s[i-1];
        asciiCRC[3] = END_STRING;
    }
    else strcpy( ( char * )asciiCRC, "" );
}

int  CRC::hasCRLF( uint8_t *s ) {
    int count;
    count = strlen( ( char * )s );
    if (count >= 2) {
        if ( ( s[count-2] == CARRIAGE_RETURN ) && ( s[count-1] == LINE_FEED ) ) return( 1 );
        else return( 0 );
    }
    else return( 0 );
}

void CRC::removeAscii( uint8_t * s ) {
    int count;
    count = strlen( ( char * )s );
    if ( count >= 3 ) s[count-3] = END_STRING;
}

void CRC::removeCRLF( uint8_t *s ) {
    int count;
    if ( hasCRLF( s ) ) {
        count = strlen( ( char * )s );
        if ( count >= 2 ) s[count-2] = END_STRING;
    }
}