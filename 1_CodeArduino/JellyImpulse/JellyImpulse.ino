/*  
 *  Created on: 30/03/2023
 *      Author: Andrea Sauviat
 *      E-mail: a-sauviat@laposte.net
 * 
 *  Description:  This code is intended to observe the pulsation of a jellyfish during its movement. 
 *                For this, we will record the data of the high frequency accelerometer in the flash memory. 
 *                To avoid blocking operations, we will not use GPS or LoRa.  
 *        
 *  Buy Board     : https://www.tindie.com/products/tleracorp/gnat-loragnss-asset-tracker/
 *  Configuration : https://github.com/GrumpyOldPizza/ArduinoCore-stm32l0
 *  Master Code   : https://github.com/kriswiner/CMWX1ZZABZ/tree/master/Gnat
 * 
 */

#include "src/STM32L0/STM32L0_Custom.h"     // Management of the STM32L082CZ
#include "src/RTC/RTC_Custom.h"         // Use Real Time Clock features
#include "src/STM32L0/Flash_ReadWrite.h"    // Use 196kB Flash Memory of the STM32L082CZ 



// >>> What to use ? <<< //
#define Debug_Mode    false
#define Use_Acc       true 
#define Use_Flash     false
bool Enable_SerialPrint_Master  = true   ;
bool Enable_SerialPrint_STM32   = true    ;
bool Enable_SerialPrint_LED     = false   ;
bool Enable_SerialPrint_RTC     = true    ;
bool Enable_SerialPrint_Acc     = true    ;
bool Enable_SerialPrint_Flash   = true    ;

int32_t Sample_Time = 0 ;



// >>> RTC <<< //
bool RTC_Alarm_Flag   = true ;
int  RTC_Timer_count  = 0    ;


// >>> LIS2DW12 - Accelerometer <<< //
#if( Use_Acc == true )
#include "src/LIS2DW12/LIS2DW12.h"               // Use Accelerometer - LIS2DW12
#define I2C_BUS    Wire               // Define the I2C bus (Wire instance) you wish to use
I2Cdev             I2C(&I2C_BUS);   // Instantiate the I2Cdev object and point to the desired I2C bus
int I2C_Frequency = 400000 ;
int16_t LIS2DWS12_Temp_Raw;      // temperature raw count output
float   LIS2DWS12_Temperature;    // Stores the real internal chip temperature in degrees Celsius
float Acc_X, Acc_Y, Acc_Z;       // variables to hold latest sensor data values 
LIS2DW12 LIS2DW12(&I2C); // instantiate LIS2DW12 class
#endif


// >>> Flash <<< //
#if( Use_Flash == true )
uint8_t data[128]; // tableau pour stocker les données lues
uint32_t Flash_count = 128; // nombre de données à lir
uint32_t flashAddress = 0x08021980 ;
uint32_t flashAddress_Updated = flashAddress ;
#endif



// >>> Functions <<< //
void RTC_Config( bool Enable_SerialPrint_RTC );
void RTC_Enable( bool Enable_SerialPrint_RTC );
void RTC_Disable( bool Enable_SerialPrint_RTC );
void RTC_Alarm_Fct_WakeUp() ;


// —————————————————————————————————————————————————————————————————————————————————————————————— //
//          SETUP()                                                                               //
// —————————————————————————————————————————————————————————————————————————————————————————————— //

void setup(){

  STM32L0.wakeup();

  // put your setup code here, to run once:
  if( Enable_SerialPrint_Master == true ){ Serial.begin(115200) ; delay(500); }
  if( Debug_Mode == true ){ while( !Serial ){} }

  delay( 500 ) ;

  Serial.println("void setup()") ;

  /* >>> BLUE LED <<< */
  STM32L0.BlueLED_Config(Enable_SerialPrint_LED);

  /* >>> LIS2DW12 - Acc <<< */
  #if (Use_Acc == true)
  I2C.Config_And_Scan( I2C_Frequency ) ;
  LIS2DW12.Acc_Config( Enable_SerialPrint_Acc ) ;
  #endif


  /* >>> RTC <<< */
  RTC_Config(Enable_SerialPrint_RTC);
  RTC_Enable(Enable_SerialPrint_RTC);

  //STM32L0.flashErase( flashAddress , 2 ) ;

  delay(5000);

  STM32L0.BlueLED_OFF( Enable_SerialPrint_LED ) ;

} // void setup()

// —————————————————————————————————————————————————————————————————————————————————————————————— //
//          LOOP()                                                                                //
// —————————————————————————————————————————————————————————————————————————————————————————————— //

void loop() {

  Serial.print( (String)"Sample time: " + Sample_Time + (String)"\t");
  
  #if (Use_Acc == true)
  LIS2DW12.Acc_Get_XYZ_Data( &Acc_X, &Acc_Y, &Acc_Z, Enable_SerialPrint_Acc);
  #endif
   

  #if( Use_Flash == true )
  Serial.print( "Acc flashAddress \t" ) ; Serial.print( flashAddress_Updated , HEX ) ; Serial.print("\t") ;
  flashAddress_Updated = Flash_PushToMemory_Acc( 1 , 2 , 3 , (int)LIS2DWS12_Temperature , flashAddress_Updated , Enable_SerialPrint_Flash ) ;
  #endif

  delay(250) ;


} // void loop()

// —————————————————————————————————————————————————————————————————————————————————————————————— //
//          FUNCTIONS                                                                             //
// —————————————————————————————————————————————————————————————————————————————————————————————— //

/* >>> RTC Alarm <<< */

/**
 * @brief Real Time Clock - Configuration
 *
 * @param Enable_SerialPrint_RTC If true, serial printing is enabled, otherwise disabled.
 * 
 * @note Set an interrup with RTC_Alam_Fct_WakeUP().
 *
 */
void RTC_Config( bool Enable_SerialPrint_RTC ){
  RTC.setAlarmTime(12, 0, 0)                   ; // Setting alarm
  RTC.attachInterrupt( RTC_Alarm_Fct_WakeUp ) ; // Alarm interrrupt
}

/**
 * @brief Real Time Clock - Enable clock
 *
 * @param Enable_SerialPrint_RTC If true, serial printing is enabled, otherwise disabled.
 * 
 * @note Set an alarm with a match per minutes for exemple.
 *
 */
void RTC_Enable( bool Enable_SerialPrint_RTC ){
  RTC.enableAlarm(RTC.MATCH_SS)         ; // Alarm once per second
  if(Enable_SerialPrint_RTC == true ){ Serial.println("RTC enable.") ; };
}

/**
 * @brief Real Time Clock - Diseable
 *
 * @param Enable_SerialPrint_RTC If true, serial printing is enabled, otherwise disabled.
 * 
 * @note Diseable alarm.
 * 
 * @warning This function need to be use with RTC_Enable for re-enable the clock.
 *
 */
void RTC_Disable( bool Enable_SerialPrint_RTC ){
  RTC.disableAlarm();
  if(Enable_SerialPrint_RTC == true ){ Serial.println("RTC disable.") ; };
}

/**
 * @brief Real Time Clock - Alarm function
 * 
 * @note This function is fired every time that the alarm is in the high mode. It's also contain the recovery mode: after a definied time, the timer of the alarm change.
 *
 * @warning This function need to be use with RTC_Enable for re-enable the clock.
 *
 */
void RTC_Alarm_Fct_WakeUp() {

  // Serial.println( (String)"RTC_Timer_count = " + RTC_Timer_count ) ;

  if( STM32_Sleeping == true ){ STM32_Sleeping = STM32L0.STM32_WakeUp( Enable_SerialPrint_STM32 ) ; }

  RTC_Alarm_Flag = true ; // Just set flag when interrupt received, don't try reading data in an interrupt handler
  
}

