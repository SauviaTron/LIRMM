/*  
 *  Created on: 01/023/2023
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

#define Debug_Mode    true
#define GNAT_L082CZ_  17

//uint32_t address = 0x08021980; // adresse de départ de la mémoire flash
uint8_t data[128]; // tableau pour stocker les données lues
uint8_t previousdata[128]; // tableau pour stocker les données lues
uint32_t count = 128; // nombre de données à lire

uint32_t flashAddress = 0x08021980 ;
uint32_t flashAddress_Updated = flashAddress ;


// —————————————————————————————————————————————————————————————————————————————————————————————— //
//          SETUP()                                                                               //
// —————————————————————————————————————————————————————————————————————————————————————————————— //

void setup() {

  STM32L0.wakeup() ;
  
  Serial.begin(115200) ; 

  if( Debug_Mode == true ){ while( !Serial ){} }

  delay(1000) ;

  STM32L0.BlueLED_Config( false ) ;
  STM32L0.BlueLED_OFF( false ) ;

  Serial.print( "Flash ReadAddress : " ) ; Serial.println( 0x8021980 , HEX ) ;

  Serial.println("Board_Name ; Battery_Level ; Time_User ; GPS_Latitude ; GPS_Longitude ; GPS_NbSatellites ; Acc_Temp ; Acc_AxeX ; Acc_AxeY ; Acc_AxeZ ; Acc_Roll ; Acc_Pitch ; Acc_Yaw ") ;

  int Data_Read = 1 ;
  //for( flashAddress = 0x08021980 ; flashAddress <= 0x0802FFFF ; flashAddress = flashAddress + 128){
  int Flash_Page_Lim = flashAddress + 128 * 100 ;
  //for( flashAddress = 0x08021980 ; flashAddress <= 0x08021980 + 128*5 ; flashAddress = flashAddress + 128){

  //STM32L0.Flash_Print_Data( 0x08021980 , data , count ) ; Serial.println() ;  STM32L0.Flash_Print_Data( 0x08021980 - 128 , data , count ) ; Serial.println() ;

  
  for( flashAddress = 0x08021980 + 5*4; flashAddress <= Flash_Page_Lim ; flashAddress = flashAddress + 5*4){
    STM32L0.flashRead(flashAddress, data, 5*4);
    STM32L0.flashRead(flashAddress - 5*4, previousdata, 5*4);
    //STM32L0.Flash_Print_Data( 0x08021980 , data , count ) ; STM32L0.Flash_Print_Data( 0x08021980 - 128 , data , count ) ; Serial.println()
    STM32L0.Flash_Decode_Data( flashAddress, data , previousdata , 5*4 , GNAT_L082CZ_ , Data_Read ) ;
    Data_Read++ ;
    //Serial.println( (String)"Page read : " + Page_Read ) ;
    delay(100) ;
  }
  

  STM32L0.BlueLED_ON( false ) ;

}




// —————————————————————————————————————————————————————————————————————————————————————————————— //
//          LOOP()                                                                                //
// —————————————————————————————————————————————————————————————————————————————————————————————— //

void loop() { 

  STM32L0.BlueLED_ON( false )  ; delay( 500 ) ;
  STM32L0.BlueLED_OFF( false ) ; delay( 500 ) ;

}