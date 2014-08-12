#include <SDI12.h>

SDI12 DECAGON_5TE_10CM(&Serial3, '1', true);

void setup() {
  while(!Serial);
  delay(100);
}

void loop() {
  char data[75];
  char debug[10];
  bool error;
  //------------------------------transparent--------------------------------------
  memset( data, 0 ,75 );
  DECAGON_5TE_10CM.transparent( "1M!", data );
  Serial.print(data);

  memset( data, 0 ,75 );
  DECAGON_5TE_10CM.transparent( "1D0!", data );
  Serial.print(data);
  //-----------------------------verification--------------------------------------
  memset( data, 0, 75 );
  memset( debug, 0, 10 );

  error = DECAGON_5TE_10CM.verification( debug );
  if ( !error ) Serial.print( debug );

  error = DECAGON_5TE_10CM.returnMeasurement( data, 0 );
  if ( !error ) Serial.print( data );
  //---------------------------------identification--------------------------------
  memset( data, 0, 75 );
  error = DECAGON_5TE_10CM.identification( data );
  if (!error) Serial.print( data );
  //---------------------------------measurement-----------------------------------
  memset( data, 0, 75 );
  memset( debug, 0, 10 );
    
  error = DECAGON_5TE_10CM.measurement( debug );
  if ( !error ) Serial.print( debug );

  error = DECAGON_5TE_10CM.returnMeasurement( data, 0 );
  if (!error) Serial.print( data );
  
  delay(1000);
}

