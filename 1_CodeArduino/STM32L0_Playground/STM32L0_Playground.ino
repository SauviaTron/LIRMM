/*  
 *  Created on: 24/02/2023
 *      Author: Andrea Sauviat
 *      E-mail: a-sauviat@laposte.net
 * 
 *  Description:  Code for testing STM32L0 functionnalities
 *        
 *  Buy Board     : https://www.tindie.com/products/tleracorp/gnat-loragnss-asset-tracker/
 *  Configuration : https://github.com/GrumpyOldPizza/ArduinoCore-stm32l0
 *  Master Code   : https://github.com/kriswiner/CMWX1ZZABZ/tree/master/Gnat
 * 
 */

#include "src/STM32L0/STM32L0_Custom.h"     // Management of the STM32L082CZ
#include "src/RTC/RTC_Custom.h"         // Use Real Time Clock features




/* >>> What to use ? <<< */

#define Debug_Mode    false

bool Enable_SerialPrint_Master  = true   ;
bool Enable_SerialPrint_STM32   = true    ;
bool Enable_SerialPrint_LED     = false   ;
bool Enable_SerialPrint_Battery = true    ;
bool Enable_SerialPrint_RTC     = true    ;




/* >>> STM32 <<< */

bool STM32_Sleeping = false   ;
float STM32_Temperature_float ; // Updated by the function - void STM32_Temperature( bool Enable_SerialPrint_STM32 )
float BatteryTension ;



/* >>> RTC <<< */

bool RTC_Alarm_Flag   = true ;
int  RTC_Timer_count  = 0    ;
int  RTC_Timer_Rescue = 100  ; 



/* >>> Functions <<< */

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
  if( Enable_SerialPrint_Master == true ){ Serial.begin(115200) ; }
  if( Debug_Mode == true ){ while( !Serial ){} }

  delay( 1000 ) ;

  Serial.println("void setup()") ;

  /* >>> BLUE LED <<< */
  STM32L0.BlueLED_Config(Enable_SerialPrint_LED);

  /* >>> Battery <<< */
  STM32L0.Battery_Config(Enable_SerialPrint_Battery);

  STM32L0.BlueLED_OFF( Enable_SerialPrint_LED ) ;

} // void setup()

// —————————————————————————————————————————————————————————————————————————————————————————————— //
//          LOOP()                                                                                //
// —————————————————————————————————————————————————————————————————————————————————————————————— //

void loop() {
  
//  Serial.println( (String)"STM32L0.resetCause() : " + STM32L0.resetCause() );

  if (RTC_Alarm_Flag == true){
    RTC_Alarm_Flag = false;

    delay( 1000 ) ;

    STM32_Temperature_float = STM32L0.STM32_Temperature( Enable_SerialPrint_STM32 ) ;

    delay( 1000 ) ;

    BatteryTension = STM32L0.Battery_GetTension( Enable_SerialPrint_Battery ) ;

    delay( 1000 ) ;

    STM32L0.BlueLED_ON( Enable_SerialPrint_LED ) ;
    delay(100);
    STM32L0.BlueLED_OFF( Enable_SerialPrint_LED ) ;

    delay( 1000 );

  } // if( RTC_Alarm_Flag == true )

  STM32_Sleeping = STM32L0.STM32_StopMode(Enable_SerialPrint_STM32);

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

  //if( STM32L0.resetCause() == 1 ){RTC_Timer_count = RTC_Timer_count + 1 ;}
  RTC_Timer_count = RTC_Timer_count + 1 ;
  if( RTC_Timer_count == RTC_Timer_Rescue ){
    Serial.println( "RTC: Rescue mode") ;
    RTC_Disable( Enable_SerialPrint_RTC ) ;
    RTC.enableAlarm( RTC.MATCH_Every_2s);
    RTC.attachInterrupt( RTC_Alarm_Fct_WakeUp ) ;
  }

  // Serial.println( (String)"RTC_Timer_count = " + RTC_Timer_count ) ;

  if( STM32_Sleeping == true ){ STM32_Sleeping = STM32L0.STM32_WakeUp( Enable_SerialPrint_STM32 ) ; }

  RTC_Alarm_Flag = true ; // Just set flag when interrupt received, don't try reading data in an interrupt handler
  
}
