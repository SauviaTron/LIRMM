/*  
 *  Created on: 06/02/2023
 *      Author: Andrea Sauviat
 *      E-mail: a-sauviat@laposte.net
 * 
 *  Description:  Ultra-low power 20 mm x 20 mm asset tracker consisting of :
 *                - CMWX1ZZABZ (SX1276 LoRa radio and STM32L082 host MCU)
 *                - MAX M8Q concurrent GNSS module
 *                - LIS2DW12 accelerometer for wake-on-motion/sleep-on-no-motion functionality.
 *        
 *  Buy Board     : https://www.tindie.com/products/tleracorp/gnat-loragnss-asset-tracker/
 *  Configuration : https://github.com/GrumpyOldPizza/ArduinoCore-stm32l0
 *  Master Code   : https://github.com/kriswiner/CMWX1ZZABZ/tree/master/Gnat
 * 
 */

 

#include "src/STM32L0_Custom.h"  // Management of the STM32L082CZ
#include "src/Flash_ReadWrite.h" // Management of the 196k Flash of the STM32L082CZ

#define Debug_Mode    false

//uint32_t address = 0x08021980; // adresse de départ de la mémoire flash
uint8_t data[128]; // tableau pour stocker les données lues
uint32_t count = 128; // nombre de données à lire

uint32_t flashAddress = 0x08021980 ;
uint32_t flashAddress_Updated = flashAddress ;


// —————————————————————————————————————————————————————————————————————————————————————————————— //
//          SETUP()                                                                               //
// —————————————————————————————————————————————————————————————————————————————————————————————— //

void setup() {

  STM32L0.wakeup() ;

  if( Debug_Mode == true ){ while( !Serial ){} }
  
  Serial.begin(115200) ; 

  delay(1000) ;

  STM32L0.flashErase( flashAddress , 2 ) ;

}




// —————————————————————————————————————————————————————————————————————————————————————————————— //
//          LOOP()                                                                                //
// —————————————————————————————————————————————————————————————————————————————————————————————— //

void loop() {
  
  if( 1 ){

    Serial.print( "Time flashAddress \t" ) ; Serial.print( flashAddress_Updated , HEX ) ; Serial.print("\t") ;
    flashAddress_Updated = Flash_PushToMemory_Time( 2302101615 , flashAddress_Updated , true ) ;

    Serial.print( "GPS flashAddress \t" ) ; Serial.print( flashAddress_Updated , HEX ) ; Serial.print("\t") ;
    flashAddress_Updated = Flash_PushToMemory_GPS( 43000000 , 300000015 , flashAddress_Updated , true ) ;

    Serial.print( "Acc flashAddress \t" ) ; Serial.print( flashAddress_Updated , HEX ) ; Serial.print("\t") ;
    flashAddress_Updated = Flash_PushToMemory_Acc( 1 , 2 , 3 , 15 , flashAddress_Updated , true ) ;

    Serial.print( "Other flashAddress \t" ) ; Serial.print( flashAddress_Updated , HEX ) ; Serial.print("\t") ;
    flashAddress_Updated = Flash_PushToMemory_Vbat_LoRa( 375 , 0 , 1 , flashAddress_Updated , true ) ;


    
    if( 1 ){

    Serial.print( "Flash ReadAddress : " ) ; Serial.println( 0x8021980 , HEX ) ;

    STM32L0.flashRead(flashAddress, data, count);

    STM32L0.Flash_Print_Data( data , count ) ;

    Serial.println();

    }

  }

  delay(10000) ;
  
}