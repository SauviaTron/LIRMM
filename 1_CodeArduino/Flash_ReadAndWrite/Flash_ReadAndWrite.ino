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
  
  // put your setup code here, to run once:
  Serial.begin(115200) ; 

  delay(5000) ;

  STM32L0.flashErase( flashAddress , 2 ) ;




}




// —————————————————————————————————————————————————————————————————————————————————————————————— //
//          LOOP()                                                                                //
// —————————————————————————————————————————————————————————————————————————————————————————————— //

void loop() {
  
  Serial.println("Hello Flash_Read.ino") ;

    if( 1 ){

    Serial.print( "flashAddress : " ) ; Serial.println( flashAddress_Updated , HEX ) ;
    flashAddress_Updated = Flash_PushToMemory_Time( 2302101615 , flashAddress_Updated , true ) ;
    Serial.print( "flashAddress : " ) ; Serial.println( flashAddress_Updated , HEX ) ;
    flashAddress_Updated = Flash_PushToMemory_GPS( 43000000 , 3000000 , 15 , flashAddress_Updated , true ) ;
    Serial.print( "flashAddress : " ) ; Serial.println( flashAddress_Updated , HEX ) ;

    // flashAddress = 0x8021980 ;

    if( 1 ){

    Serial.print( "Flash ReadAddress : " ) ; Serial.println( 0x8021980 , HEX ) ;

    STM32L0.flashRead(flashAddress, data, count);

    Serial.println("Données lues à partir de la mémoire flash : ");
    int Nb_Data_Display = 0 ;
    int Flash_Nb_Cell = 1 ;
    for (int i = 0; i < count; i++) {
      
        if( Nb_Data_Display == 0 ){ 
          if( Flash_Nb_Cell <= 9 ){
            Serial.print( (String)"Cell n°0" + Flash_Nb_Cell) ; Serial.print("\t") ;
            Flash_Nb_Cell += 1 ; 
          }
          else{
            Serial.print( (String)"Cell n°" + Flash_Nb_Cell) ; Serial.print("\t") ;
            Flash_Nb_Cell += 1 ; 
          }
        }
        
        String data_i_string = String( data[i] , BIN ) ;
        // Serial.println( (String)"Taille string : " + data_i_string ) ; 
        switch( data_i_string.length() ){
          case 8 :
            Serial.print(data[i], BIN); Serial.print(" ");
            break ;
          case 7 :
            Serial.print( "0" ) ; Serial.print(data[i], BIN); Serial.print(" ");
            break ;
          case 6 :
            Serial.print( "00" ) ; Serial.print(data[i], BIN); Serial.print(" ");
            break ;
          case 5 :
            Serial.print( "000" ) ; Serial.print(data[i], BIN); Serial.print(" ");
            break ;
          case 4 :
            Serial.print( "0000" ) ; Serial.print(data[i], BIN); Serial.print(" ");
            break ;
          case 3 :
            Serial.print( "00000" ) ; Serial.print(data[i], BIN); Serial.print(" ");
            break ;
          case 2 :
            Serial.print( "000000" ) ; Serial.print(data[i], BIN); Serial.print(" ");
            break ;
          case 1 :
            Serial.print( "0000000" ) ; Serial.print(data[i], BIN); Serial.print(" ");
            break ;
        } // switch( data_i_string.length() )

        Nb_Data_Display += 1 ;
        if( Nb_Data_Display == 4 ){
          Serial.println(" "); 
          Nb_Data_Display = 0 ;
        }

    }
    Serial.println();

    }
  
  }

  delay(10000) ;
  
}