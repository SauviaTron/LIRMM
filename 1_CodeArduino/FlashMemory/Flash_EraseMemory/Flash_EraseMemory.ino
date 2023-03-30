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
uint32_t flashAddress_Updated ;


// —————————————————————————————————————————————————————————————————————————————————————————————— //
//          SETUP()                                                                               //
// —————————————————————————————————————————————————————————————————————————————————————————————— //

void setup() {

  STM32L0.wakeup() ;
  
  STM32L0.BlueLED_Config( false ) ;
  
  // put your setup code here, to run once:
  Serial.begin(115200) ; 

  while( !Serial ){}

  delay(1000) ;

  STM32L0.flashErase( flashAddress , 2 ) ;

  STM32L0.BlueLED_ON( false ) ;


}




// —————————————————————————————————————————————————————————————————————————————————————————————— //
//          LOOP()                                                                                //
// —————————————————————————————————————————————————————————————————————————————————————————————— //

void loop() {


  
  Serial.println("Hello Flash_Read.ino") ;

  delay(10000) ;
  
}