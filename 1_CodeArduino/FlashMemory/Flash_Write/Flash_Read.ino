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

#include <STM32L0.h>  // Management of the STM32L082CZ
// #include "RTC.h"      // Use Real Time Clock features


/* >>> BLUE LED <<< */
#define Blue_LED     10            // Blue led 

// /* >>> RTC <<< */
// bool RTC_Alarm_Flag = true ;
// int RTC_Timer_count = 0 ;

  uint32_t address = 0x08021980; // adresse de départ de la mémoire flash
  uint8_t data[128]; // tableau pour stocker les données lues
  uint32_t count = 128; // nombre de données à lire




// /* >>> Serial Println <<< */
bool Enable_SerialPrint_LED = false ;
// bool Enable_SerialPrint_STM32 = false ;
// bool Enable_SerialPrint_Acc = true ;


// /* >>> Functions <<< */

// void STM32_WakeUp( bool Enable_SerialPrint_STM32 ) ;
// void STM32_StopMode( bool Enable_SerialPrint_STM32 ) ;
// void STM32_Temperature( bool Enable_SerialPrint_STM32 ) ;

// void BlueLED_Config( bool Enable_SerialPrint_LED )  ;
// void BlueLED_ON( bool Enable_SerialPrint_LED ) ;
// void BlueLED_OFF( bool Enable_SerialPrint_LED );

// void RTC_Enable( bool Enable_SerialPrint );
// void RTC_Disable( bool Enable_SerialPrint );
// void RTC_Alarm_Fct_Wakeup() ;


// —————————————————————————————————————————————————————————————————————————————————————————————— //
//          SETUP()                                                                               //
// —————————————————————————————————————————————————————————————————————————————————————————————— //

void setup() {

  STM32L0.wakeup() ;
  
  // put your setup code here, to run once:
  Serial.begin(115200) ; 

  /* >>> BLUE LED <<< */
  BlueLED_Config( Enable_SerialPrint_LED ) ;

  delay(5000) ;

  // STM32L0.flashErase( address , 2 ) ;

}


// —————————————————————————————————————————————————————————————————————————————————————————————— //
//          LOOP()                                                                                //
// —————————————————————————————————————————————————————————————————————————————————————————————— //

void loop() {
  
  Serial.println("Flash_Read.ino") ;
  Serial.println("Your program") ;
 

// https://www.st.com/resource/en/reference_manual/rm0376-ultralowpower-stm32l0x2-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
// page 62

  Serial.print( "address : " ) ;
  Serial.println( address , HEX ) ;

  bool success = STM32L0.flashRead(address, data, count);

    Serial.println("Données lues à partir de la mémoire flash : ");
    for (int i = 0; i < count; i++) {
        Serial.print(data[i], BIN);
        Serial.print(" ");
    }
    Serial.println();

//  address = address + 0x80 ; // One page = 128bytes so add 128 to change page

  delay(10000) ;
  
}





// —————————————————————————————————————————————————————————————————————————————————————————————— //
//          FUNCTIONS                                                                             //
// —————————————————————————————————————————————————————————————————————————————————————————————— //

/* >>> STM32 <<< */

// void STM32_WakeUp( bool Enable_SerialPrint_STM32 ){

//   STM32L0.wakeup() ;

//   STM32_Sleeping = false ; // Setting the flag
    
//   if( Enable_SerialPrint_STM32 == true ){ Serial.println("STM32 : STM32_WakeUp"); } // WakeUp msg

// }

// void STM32_StopMode( bool Enable_SerialPrint_STM32 ){

//   STM32_Sleeping = true ; // Setting the flag
    
//   if( Enable_SerialPrint_STM32 == true ){ Serial.println("STM32 : STM32_StopMode"); } // Last msg

//   STM32L0.stop() ; // Stop Mode

// }

// void STM32_Temperature( bool Enable_SerialPrint_STM32 ){

//   STM32_Temperature_float = STM32L0.getTemperature() ;

//   if( Enable_SerialPrint_STM32 == true ){ Serial.println( (String)"STM32 : STM32_Temperature : " + STM32_Temperature_float + "°" ); }

// }


/* >>> BLUE LED <<< */

void BlueLED_Config( bool Enable_SerialPrint_LED ){
 
  pinMode(Blue_LED, OUTPUT);      
  BlueLED_ON( Enable_SerialPrint_LED ) ;

}

void BlueLED_ON( bool Enable_SerialPrint_LED ){
 
  digitalWrite(Blue_LED, LOW);

  if( Enable_SerialPrint_LED == true ){ Serial.println("LED: ON") ; }

}

void BlueLED_OFF( bool Enable_SerialPrint_LED ){
 
  digitalWrite(Blue_LED, HIGH);

  if( Enable_SerialPrint_LED == true ){ Serial.println("LED: OFF") ; }

}


// /* >>> RTC Alarm <<< */

// void RTC_Enable( bool Enable_SerialPrint ){
//     // // --- Set the RTC time --- //
//   RTC.setAlarmTime(12, 0, 0)                   ; // Setting alarm
//   RTC.enableAlarm(RTC.MATCH_ANY)         ; // Alarm once per second
//   //RTC.enableAlarm(RTC.MATCH_SS)            ; // Alarm once per minute
//   RTC.attachInterrupt( RTC_Alarm_Fct_Wakeup ) ; // Alarm interrrupt
//   if(Enable_SerialPrint == true ){ Serial.println("RTC enable.") ; };
// }

// void RTC_Disable( bool Enable_SerialPrint ){
//   RTC.disableAlarm();
//   if(Enable_SerialPrint == true ){ Serial.println("RTC disable.") ; };
// }

// void RTC_Alarm_Fct_Wakeup() {

//   //if( STM32L0.resetCause() == 1 ){RTC_Timer_count = RTC_Timer_count + 1 ;}
//   RTC_Timer_count = RTC_Timer_count + 1 ;

//   // Serial.println( (String)"RTC_Timer_count = " + RTC_Timer_count ) ;

//   if( STM32_Sleeping == true ){
//     STM32_WakeUp( Enable_SerialPrint_STM32 ) ; 
//   }
//   RTC_Alarm_Flag = true ; // Just set flag when interrupt received, don't try reading data in an interrupt handler
//   //Serial.println("RTC: Flag timer true");
// }
