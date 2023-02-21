/*  
 *  Created on: 06/02/2023
 *      Author: Andrea Sauviat
 *      E-mail: a-sauviat@laposte.net
 * 
 *  Description:  Ultra-low power 20 mm x 20 mm asset tracker consisting of :
 *                - c (SX1276 LoRa radio and STM32L082 host MCU)
 *                - MAX M8Q concurrent GNSS module
 *                - LIS2DW12 accelerometer for wake-on-motion/sleep-on-no-motion functionality.
 *        
 *  Buy Board     : https://www.tindie.com/products/tleracorp/gnat-loragnss-asset-tracker/
 *  Configuration : https://github.com/GrumpyOldPizza/ArduinoCore-stm32l0
 *  Master Code   : https://github.com/kriswiner/CMWX1ZZABZ/tree/master/Gnat
 * 
 */

#include "src/STM32L0/STM32L0_Custom.h"     // Management of the STM32L082CZ
#include "src/RTC/RTC_Custom.h"         // Use Real Time Clock features
#include "src/GNSS/GNSS_Custom.h"        // Use GPS - MAX M8Q
#include "src/STM32L0/Flash_ReadWrite.h"    // Use 196kB Flash Memory of the STM32L082CZ 
#include "src/LoRaWAN/LoRaWAN.h"            // Use LoRaWAN
#include "src/CayenneLPP/CayenneLPP.h"  // Use Cayenne Low Power Payload


/* >>> What to use ? <<< */
#define Debug_Mode    true
#define GNAT_L082CZ_  0
#define Use_Acc       false 
#define Use_GPS       false
#define Use_LoRa      true


/* >>> LoRa codes <<< */
#if ( GNAT_L082CZ_ == 0 )
const char *devEui = "70B3D57ED004E6DC";
const char *appEui = "0000000000000000";
const char *appKey = "F1DDBCC3E8DFB001D00462D9232B5096";
int Time_Device_Sleep = 30 ; // - seconds
#endif


/* >>> STM32 <<< */
bool STM32_Sleeping = false   ;
float STM32_Temperature_float ; // Updated by the function - void STM32_Temperature( bool Enable_SerialPrint_STM32 )
float BatteryTension ;


/* >>> RTC <<< */
bool RTC_Alarm_Flag   = true ;
int  RTC_Timer_count  = 0    ;
int  RTC_Timer_Rescue = 100  ; 


/* >>> LIS2DW12 - Accelerometer <<< */
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


/* >>> MAX M8Q - GPS <<< */
#if( Use_GPS == true )
int GPS_TimerON = 45 ;
double GPS_Longitude, GPS_Latitude ;
unsigned int GPS_NbSatellites ;
#endif


/* >>> LoRa <<< */
CayenneLPP CayenneLPPayload(64) ;
char buffer[32];


/* >>> Serial Println <<< */
bool Enable_SerialPrint_Master  = true   ;
bool Enable_SerialPrint_STM32   = true    ;
bool Enable_SerialPrint_LED     = false   ;
bool Enable_SerialPrint_Battery = true    ;
bool Enable_SerialPrint_RTC     = true    ;
bool Enable_SerialPrint_Acc     = true    ;
bool Enable_SerialPrint_GPS     = true    ;
bool Enable_SerialPrint_LoRa    = true    ;


/* >>> Functions <<< */

void RTC_Config( bool Enable_SerialPrint_RTC );
void RTC_Enable( bool Enable_SerialPrint_RTC );
void RTC_Disable( bool Enable_SerialPrint_RTC );
void RTC_Alarm_Fct_WakeUp() ;

#if( Use_LoRa == true )
void LoRa_Config( bool Enable_SerialPrint_LoRa );
void LoRa_SendPayload( bool Enable_SerialPrint_LoRa ) ;
#endif


// —————————————————————————————————————————————————————————————————————————————————————————————— //
//          SETUP()                                                                               //
// —————————————————————————————————————————————————————————————————————————————————————————————— //

void setup(){

  STM32L0.wakeup();

  // put your setup code here, to run once:
  if( Enable_SerialPrint_Master == true ){ Serial.begin(115200) ; }
  if( Debug_Mode == true ){ while( !Serial ){} }

  delay( 500 ) ;

  Serial.println("void setup()") ;

  /* >>> BLUE LED <<< */
  STM32L0.BlueLED_Config(Enable_SerialPrint_LED);

  /* >>> Battery <<< */
  STM32L0.Battery_Config(Enable_SerialPrint_Battery);


  // Info about LoRa

  #if (Use_LoRa == true)
  LoRaWAN.Config_And_JoinOTAA( devEui, appEui, appKey,  Enable_SerialPrint_LoRa );
  // LoRa_Config( Enable_SerialPrint_LoRa ) ;
  #endif

  /* >>> LIS2DW12 - Acc <<< */
  #if (Use_Acc == true)
  I2C.Config_And_Scan( I2C_Frequency ) ;
  LIS2DW12.Acc_Config( Enable_SerialPrint_Acc ) ;
  #endif

  /* >>> MAX M8Q - GPS <<< */
  #if (Use_GPS == true)
  GNSS.GPS_Config(Enable_SerialPrint_GPS);
  //GNSS.GPS_First_Fix( &GPS_Latitude, &GPS_Longitude, &GPS_NbSatellites, Enable_SerialPrint_GPS );
  #endif

  /* >>> RTC <<< */
  RTC_Config(Enable_SerialPrint_RTC);
  RTC_Enable(Enable_SerialPrint_RTC);

  delay(5000);

  STM32L0.BlueLED_OFF( Enable_SerialPrint_LED ) ;

}

// —————————————————————————————————————————————————————————————————————————————————————————————— //
//          LOOP()                                                                                //
// —————————————————————————————————————————————————————————————————————————————————————————————— //

void loop() {
  
//  Serial.println( (String)"STM32L0.resetCause() : " + STM32L0.resetCause() );

  if (RTC_Alarm_Flag == true){
    RTC_Alarm_Flag = false;

    Serial.println(">>> Your program <<<");

    STM32_Temperature_float = STM32L0.STM32_Temperature( Enable_SerialPrint_STM32 ) ;
    BatteryTension = STM32L0.Battery_GetTension( Enable_SerialPrint_Battery ) ;

    #if (Use_Acc == true)
    LIS2DW12.powerUp( LIS2DW12_ODR_12_5_1_6HZ ) ;
    LIS2DW12.Acc_Get_Temperature( &LIS2DWS12_Temp_Raw , &LIS2DWS12_Temperature , Enable_SerialPrint_Acc ) ;
    LIS2DW12.Acc_Get_XYZ_Data( &Acc_X, &Acc_Y, &Acc_Z, Enable_SerialPrint_Acc);
    LIS2DW12.powerDown();
    #endif

    #if (Use_GPS == true)
    RTC_Disable(Enable_SerialPrint_RTC); delay(100);
    GNSS.GPS_ON(Enable_SerialPrint_GPS); delay(100);
    GNSS.GPS_ReadUpdate( GPS_TimerON, &GPS_Latitude, &GPS_Longitude, &GPS_NbSatellites, Enable_SerialPrint_GPS); delay(100);
    GNSS.GPS_OFF(Enable_SerialPrint_GPS); delay(100);
    RTC_Enable(Enable_SerialPrint_RTC);
    #endif

    #if( Use_LoRa == true )
    LoRa_SendPayload( Enable_SerialPrint_LoRa ) ;
    #endif

  } // if( RTC_Alarm_Flag == true )

  STM32_Sleeping = STM32L0.STM32_StopMode(Enable_SerialPrint_STM32);

}

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


/* >>> LoRa <<< */
#if( Use_LoRa == true )

void LoRa_Config(bool Enable_SerialPrint_LoRa){

    LoRaWAN.getDevEui(buffer, 18); // Get DevEUI

    // --- Configuration LoRaWAN --- //
    // Asia AS923 | Australia  AU915 | Europe EU868 | India IN865 | Korea KR920 | US US915 (64 + 8 channels)

    LoRaWAN.begin(EU868);
    LoRaWAN.setADR(false);
    LoRaWAN.setDataRate(0); // 0 => SF = 12 | 1 => SF = 11 | 2 => SF 10 ... Careful with the size of the payload
    LoRaWAN.setTxPower(0);
    LoRaWAN.setSubBand(1); // 1 for MTCAP, 2 for TT gateways

    LoRaWAN.joinOTAA(appEui, appKey, devEui);

    if (Enable_SerialPrint_LoRa == true){
      Serial.println((String) "DevEUI: " + devEui);
      Serial.println((String) "AppEUI: " + appEui);
      Serial.println((String) "AppKey: " + appKey);
    }

}

void LoRa_SendPayload( bool Enable_SerialPrint_LoRa ) {

  // --- Send Data to LoRa --- //
  if( Enable_SerialPrint_LoRa == true ){
    Serial.println( (String)"LoRaWAN: Busy   " + LoRaWAN.busy() )   ; // Display state of LoRa - 0 for false (= available) - 1 for true (= busy)
    Serial.println( (String)".        Joined " + LoRaWAN.joined() ) ; // Display LoRa connection - 0 for false (= not joined) - 1 for true (= joined)
  }

  if ( !LoRaWAN.busy() && LoRaWAN.joined() ) { // if LoRa available (not(0)=1=true) AND LoRa joined then 

    CayenneLPPayload.reset(); // Reset writing payload

    CayenneLPPayload.addTemperature( 1 , STM32_Temperature_float ) ;
    CayenneLPPayload.addBatteryLevel( 2 , BatteryTension ) ;
    #if( Use_Acc == true )
    CayenneLPPayload.addTemperature( 3 , LIS2DWS12_Temperature ) ; 
    CayenneLPPayload.addAccelerometer( 3 , Acc_X, Acc_Y, Acc_Z)          ; // add Accelerometer
    #endif
    #if( Use_GPS == true )
    // double GPS_Latitude = 43.123456 ; 
    // double GPS_Longitude = 3.123456 ;
    CayenneLPPayload.addGPS( 4 , (float)GPS_Latitude , (float)GPS_Longitude ) ; 
    #endif
  
    LoRaWAN.sendPacket(CayenneLPPayload.getBuffer(), CayenneLPPayload.getSize());

    if( Enable_SerialPrint_LoRa == true ){ Serial.println(".        Msg send") ; } // Display a msg

  } // if ( !LoRaWAN.busy() && LoRaWAN.joined() )
  else{ if( Enable_SerialPrint_LoRa == true ){ Serial.println(".        Msg not send") ; } } // Display a msg

}

#endif

