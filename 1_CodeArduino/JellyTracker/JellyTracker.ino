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
#include "src/RTC/RTC_Custom.h"             // Use Real Time Clock features
#include "src/GNSS/GNSS_Custom.h"           // Use GPS - MAX M8Q
#include "src/STM32L0/Flash_ReadWrite.h"    // Use 196kB Flash Memory of the STM32L082CZ 
#include "src/LoRaWAN/LoRaWAN.h"            // Use LoRaWAN
#include "src/MyLPP/MyLPP.h"                // Use Cayenne Low Power Payload



/* >>> What to use ? <<< */

#define Debug_Mode    false // If true, the code will run only after pening the Serial Monitor
#define GNAT_L082CZ_  0    // Float number
#define Use_Acc       true  // If true, the accelerometer will operate
#define Use_GPS       true  // ---
#define Use_LoRa      true  // ---
#define Use_Flash     true  // --- 

bool Enable_SerialPrint_Master  = true    ; // Variable enabling 'Serial.begin(115200)'
bool Enable_SerialPrint_STM32   = true    ; // Variable for debugging STM32 status
bool Enable_SerialPrint_LED     = false   ; // ---
bool Enable_SerialPrint_Battery = true    ; // ---
bool Enable_SerialPrint_RTC     = true    ; // ---
bool Enable_SerialPrint_Acc     = true    ; // ---
bool Enable_SerialPrint_GPS     = true    ; // ---
bool Enable_SerialPrint_LoRa    = true    ; // ---
bool Enable_SerialPrint_Flash   = true    ; // ---



// —————————————————————————————————————————————————————————————————————————————————————————————— //
//          GLOBAL VARIABLES                                                                      //
// —————————————————————————————————————————————————————————————————————————————————————————————— //


/* >>> STM32 <<< */

bool  STM32_Sleeping = false  ; // Default STM32 status - Updated by the function RTC_Alarm_Fct_WakeUp and STM32L0.STM32_StopMode
float STM32_Temperature_float ; // Updated by the function - void STM32_Temperature
float BatteryTension          ; // Updated by the function - STM32L0.Battery_GetTension


/* >>> RTC <<< */

bool RTC_Alarm_Flag   = true ; // Updated by the function - RTC_Alarm_Fct_WakeUp
int  RTC_Timer_count  = 0    ; // Variable for a Rescue mode
int  RTC_Timer_Rescue = 100  ; // Number of alarm rings required to activate the rescue mode


/* >>> LIS2DW12 - Accelerometer <<< */

#if( Use_Acc == true )                // If you want to use the accelerometer ...
  #include "src/LIS2DW12/LIS2DW12.h"    // ... you will need the library
  #define I2C_BUS    Wire               // ... define the I2C bus (Wire instance) you wish to use
  I2Cdev             I2C(&I2C_BUS) ;    // ... instantiate the I2Cdev object and point to the desired I2C bus
  int      I2C_Frequency = 400000  ;    // ... setup the working frequency
  int16_t  LIS2DWS12_Temp_Raw      ;    // ... default variable for temperature raw count output
  float    LIS2DWS12_Temperature   ;    // ... stores the real internal chip temperature in degrees Celsius
  float    Acc_X, Acc_Y, Acc_Z     ;    // ... variables to hold latest sensor data values 
  LIS2DW12 LIS2DW12( &I2C )        ;    // ... instantiate LIS2DW12 class
#endif


/* >>> MAX M8Q - GPS <<< */

#if( Use_GPS == true )                          // If you want to use the GPS ...
  int           GPS_TimerON = 40            ;     // ... time in sec that the board will search for the position (time GPS on)
  float         GPS_EHPE_Lim = 25.0f        ;     // ... floor if reached, the GPS is turned off (to save battery)
  double        GPS_Longitude, GPS_Latitude ;     // ... default values - updated by GNSS.GPS_ReadUpdate
  unsigned int  GPS_NbSatellites            ;     // ...
  float         GPS_EHPE                    ;     // ...
  uint32_t      Date                        ;     // ...
  int           Date_Second                 ;     // ...
#endif


/* >>> LoRa <<< */

#if( Use_LoRa == true )                                     // If you want to use the LoRa ...
  MyLPP       MyLPPayload(64) ;                               // ... define MyLPP
  char        buffer[32]      ;                               // ... setup buffer that will contain the LoRa payload
  int         LoRaWAN_Busy    ;                               // ... indicates if the network is busy
  int         LoRaWAN_Joined  ;                               // ... indicates if the board has joined the LoRa natwork
  const char *appEui = "0000000000000000"                 ;   // ... for Orange Live Object
  const char *appKey = "8F29B7C3A418BC2D7F16F8E25E9C9ABC" ;   // ... 
  const char *devEuiTable[] = { "3D3B8F7A9F26C000",           // ... List of DevEUI
    "3D3B8F7A9F26C001", "3D3B8F7A9F26C002", "3D3B8F7A9F26C003", "3D3B8F7A9F26C004", "3D3B8F7A9F26C005", 
    "3D3B8F7A9F26C006", "3D3B8F7A9F26C007", "3D3B8F7A9F26C008", "3D3B8F7A9F26C009", "3D3B8F7A9F26C010", 
    "3D3B8F7A9F26C011", "3D3B8F7A9F26C012", "3D3B8F7A9F26C013", "3D3B8F7A9F26C014", "3D3B8F7A9F26C015",
    "3D3B8F7A9F26C016", "3D3B8F7A9F26C017", "3D3B8F7A9F26C018", "3D3B8F7A9F26C019", "3D3B8F7A9F26C020"
  };
  const char *devEui = devEuiTable[GNAT_L082CZ_] ;            // ... set the DevEUI of the programmed board
#endif


/* >>> Flash <<< */
#if (Use_Flash == true)                                           // If you want to use the STM32 Flash memory ...
  uint8_t  data[128];                                               // ... table for storing data 
  uint32_t Flash_count = 128                    ;                   // ... number of data that need to be read
  uint32_t flashAddress = 0x8021980             ;                   // ... address from which to start writing (30% memory)
  uint32_t flashAddress_Updated = flashAddress  ;                   // ... cariable that will be updated by Flash_PushToMemory_
#endif



// —————————————————————————————————————————————————————————————————————————————————————————————— //
//          FUNCTIONS                                                                             //
// —————————————————————————————————————————————————————————————————————————————————————————————— //

void RTC_Config(  bool Enable_SerialPrint_RTC ) ;
void RTC_Enable(  bool Enable_SerialPrint_RTC ) ;
void RTC_Disable( bool Enable_SerialPrint_RTC ) ;
void RTC_Alarm_Fct_WakeUp( )                    ;

#if( Use_LoRa == true )
  int  LoRa_SendMsg( bool Enable_SerialPrint_LoRa )     ;
  void LoRa_SendPayload( bool Enable_SerialPrint_LoRa ) ;
#endif

#if( Use_Flash == true )
uint32_t flash_ReadLastAddressUsed( ) ;
uint32_t flash_StockData( uint32_t flashAddress_LastAddressWritten ) ;
void flash_ReadNPrintData( uint32_t flashAddress );
#endif

// —————————————————————————————————————————————————————————————————————————————————————————————— //
//          SETUP()                                                                               //
// —————————————————————————————————————————————————————————————————————————————————————————————— //

void setup(){

  STM32L0.wakeup( ) ; // Make sure that the STM32 is awake and not sleepy

  if( Enable_SerialPrint_Master == true ){                  // If the user want the Serial Minitor  
    Serial.begin(115200) ; delay(500) ;                         // ... then with activate at 115 200 baud rate  
  }
  if( Debug_Mode == true ){                                 // If the user want the debug mode
    while( !Serial ){}                                          // ... then we wating that he open the serial monitor to continue the code
  }

  delay( 5000 ) ;                                           // Add a 5 sec delay to slow down the process

  Serial.println("void setup()") ;                          // Welcom message

  STM32L0.BlueLED_Config( Enable_SerialPrint_LED ) ;        // Config and turn on the blue LED

  STM32L0.Battery_Config( Enable_SerialPrint_Battery ) ;    // Config the pin for reading the battery voltage

  #if (Use_Acc == true)                                     // If you want to use the accelerometer 
  I2C.Config_And_Scan( I2C_Frequency ) ;                        // ... you will need to setup the I2C
  LIS2DW12.Acc_Config( Enable_SerialPrint_Acc ) ;               // ... and config plus calibrate the accelerometer 
  #endif

  #if (Use_GPS == true)                                     // If you want to use the GPS
  GNSS.GPS_Config(Enable_SerialPrint_GPS);                      // ... you will need to configure the pin
  GNSS.GPS_First_Fix( &GPS_Latitude, &GPS_Longitude, &GPS_NbSatellites, &GPS_EHPE, &Date, &Date_Second, Enable_SerialPrint_GPS ) ; // // ... and get a first fix
  #endif

  #if (Use_LoRa == true)                                    // If you want to use the LoRa communication
  LoRaWAN.Config_And_JoinOTAA( devEui, appEui, appKey,  Enable_SerialPrint_LoRa ) ; // ... you will need to configure and make a first join
  #endif

  RTC_Config( Enable_SerialPrint_RTC ) ;                    // Configure the Real Time Clock of the STM32
  RTC_Enable( Enable_SerialPrint_RTC ) ;                    // Enable the Real Time Clock

  delay( 5000 ) ;                                           // Add a 5 sec delay to slow down the process

  STM32L0.BlueLED_OFF( Enable_SerialPrint_LED ) ;           // Turn off the LED to indicate the end of the setup function

} // void setup()



// —————————————————————————————————————————————————————————————————————————————————————————————— //
//          LOOP()                                                                                //
// —————————————————————————————————————————————————————————————————————————————————————————————— //

void loop() {
  
  if (RTC_Alarm_Flag == true){                                                                                                              // If the alarm ring (ie STM32 has woken up)
    
    RTC_Alarm_Flag = false ;                                                                                                                      // ... set the flag down
    
    STM32L0.BlueLED_ON( Enable_SerialPrint_LED ) ;                                                                                                // ... turn the blue LED ON
    
    RTC_Disable(Enable_SerialPrint_RTC) ; delay(100) ;                                                                                            // ... disable the RTC. Usefull to avoid corrupting code execution

    STM32_Temperature_float = STM32L0.STM32_Temperature( Enable_SerialPrint_STM32 ) ; delay(100) ;                                                // ... read STM32 temperature
    BatteryTension = STM32L0.Battery_GetTension( Enable_SerialPrint_Battery ) ;                                                                   // ... read battery voltage
    BatteryTension = BatteryTension - 2.0f ; delay(1000) ;                                                                                                     // ... rescale value for flash storage optimization (fewer bits used to store 170V (1.7V) than 370V (3.7V))

    #if (Use_Acc == true)                                                                                                                         // ... If you use the accelerometer
      LIS2DW12.powerUp( LIS2DW12_ODR_12_5_1_6HZ ) ; delay(100) ;                                                                                      // ... you need to turn it on
      LIS2DW12.Acc_Get_Temperature( &LIS2DWS12_Temp_Raw , &LIS2DWS12_Temperature , Enable_SerialPrint_Acc ) ; delay(100) ;                            // ... get the accelerometer data
      LIS2DW12.Acc_Get_XYZ_Data( &Acc_X, &Acc_Y, &Acc_Z, Enable_SerialPrint_Acc) ; delay(100) ;                                                       // ... and the temperature
      LIS2DW12.powerDown() ; delay(1000) ;                                                                                                            // ... and turn it off for power management
    #endif

    #if (Use_GPS == true)                                                                                                                        // ... If you want to use the GPS
      GNSS.GPS_ON(Enable_SerialPrint_GPS) ; delay(100) ;                                                                                             // ... you neet to turn it on
      GNSS.GPS_ReadUpdate( GPS_TimerON, GPS_EHPE_Lim, &GPS_Latitude, &GPS_Longitude, &GPS_NbSatellites, &GPS_EHPE, &Date, &Date_Second, Enable_SerialPrint_GPS) ; delay(100) ; // get the GPS data
      GNSS.GPS_OFF(Enable_SerialPrint_GPS) ; delay(1000) ;                                                                                            // ... and turn it off for power management
    #endif

    #if( Use_LoRa == true )                                                                                                                     // ... If you want to use the LoRa
      LoRa_SendPayload( Enable_SerialPrint_LoRa ) ; delay(1000) ;                                                                                   // ... you have to send the payload    
    #endif
   

    #if( Use_Flash == true )                                                                                                                    // If you want to use the flash memory
      flashAddress_Updated = flash_ReadLastAddressUsed( ) ;                                                                                       // ... you will need to know where you can write, ie the last unwrite flash address
      flashAddress_Updated = flash_StockData( flashAddress_Updated ) ;                                                                            // ... knowing that, you can write the data that you want
      flash_ReadNPrintData( 0x8021980 ) ;                                                                                                         // ... you can print data to be sure that everything is ok
      Serial.println(" ") ; flash_ReadNPrintData( 0x8021980 - 128 ) ;                                                                             // ... prinbting where you store the last flash address to see the incrementation
      delay(1000) ;
    #endif

    RTC_Enable(Enable_SerialPrint_RTC); delay(100) ;

    //STM32_Sleeping = STM32L0.STM32_StopMode(Enable_SerialPrint_STM32);

  } // if( RTC_Alarm_Flag == true )

  STM32L0.BlueLED_OFF(false);

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
  RTC.enableAlarm(RTC.MATCH_Every_2min)         ; // Alarm once per second
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
  // RTC_Timer_count = RTC_Timer_count + 1 ;
  // if( RTC_Timer_count == RTC_Timer_Rescue ){
  //   Serial.println( "RTC: Rescue mode") ;
  //   RTC_Disable( Enable_SerialPrint_RTC ) ;
  //   RTC.enableAlarm( RTC.MATCH_Every_2s);
  //   RTC.attachInterrupt( RTC_Alarm_Fct_WakeUp ) ;
  // }

  // Serial.println( (String)"RTC_Timer_count = " + RTC_Timer_count ) ;

  if( STM32_Sleeping == true ){ STM32_Sleeping = STM32L0.STM32_WakeUp( Enable_SerialPrint_STM32 ) ; }

  RTC_Alarm_Flag = true ; // Just set flag when interrupt received, don't try reading data in an interrupt handler
  
}


/* >>> LoRa <<< */
#if( Use_LoRa == true )

/**
 * @brief LoRa - Create & Send the payload
 * 
 * @param Enable_SerialPrint_LoRa If true, serial printing is enabled, otherwise disabled.
 * 
 * @note This function creates the payload to be sent by LoRa. In our case, we're only sending the battery and GPS (lat, long and EHPE).
 *
 */
int LoRa_SendMsg( bool Enable_SerialPrint_LoRa ){

  if ( (!LoRaWAN.busy() && LoRaWAN.joined())  ) {                                 // if LoRa available (not(0)=1=true) AND LoRa joined then 

      LoRaWAN_Busy = 0 ; LoRaWAN_Joined = 1 ;                                         // ... memorise the LoRa status

      MyLPPayload.reset() ;                                                           // ... reset writing payload

      // MyLPPayload.addTemperature( STM32_Temperature_float ) ;                      // ... adding STM32 temperature to the paylaod
      MyLPPayload.addBatteryLevel( BatteryTension ) ;                                 // ... adding the battery level to the payload
      
      // #if( Use_Acc == true )                                                       // ... if you want to send the accelerometer data
      // MyLPPayload.addAccelerometer( (float)Acc_X, (float)Acc_Y, (float)Acc_Z ) ;       // ... add acceleration data
      // MyLPPayload.addTemperature( LIS2DWS12_Temperature ) ;                            // ... add temperature data
      // #endif

      #if( Use_GPS == true )                                                          // ... if you use the GPS
      MyLPPayload.addGPS( (float)GPS_Latitude , (float)GPS_Longitude ) ;                  // ... latitude and longitude will be added to the payload
      // MyLPPayload.addDigit( GPS_NbSatellites ) ;                                       // ... same for the number of satellites detected
      MyLPPayload.addDigit( GPS_EHPE ) ;                                                  // ... and same for the precision of the position
      #endif
    
      /* Debug Mode - For sending information without activating modules */
      // MyLPPayload.addAccelerometer( 0.0, 0.0, 0.0 ) ;
      // MyLPPayload.addTemperature( 0 ) ;
      // MyLPPayload.addGPS( 0.0, 0.0 ) ; 
      // MyLPPayload.addDigit( 0 ) ;
      // MyLPPayload.addDigit( 0 ) ;
      
      LoRaWAN.sendPacket( MyLPPayload.getBuffer() , MyLPPayload.getSize() ) ;         // ... send the payload

      if( Enable_SerialPrint_LoRa == true ){                                          // ... if serial debug activated
        Serial.println( (String)"LoRaWAN: Busy   " + LoRaWAN_Busy )   ;                   // ... display state of LoRa - 0 for false (= available) - 1 for true (= busy)
        Serial.println( (String)".        Joined " + LoRaWAN_Joined ) ;                   // ... display LoRa connection - 0 for false (= not joined) - 1 for true (= joined)
        Serial.println(         ".        Msg send") ;                                    // ...
      }

      return 1 ;                                                                      // ... Msg was send so quit the function

    } // if ( !LoRaWAN.busy() && LoRaWAN.joined() )

    else{                                                                         // Else 
      LoRaWAN_Busy = 1 ; LoRaWAN_Joined = 1 ;                                         // ... set valuesss
      if( Enable_SerialPrint_LoRa == true ){                                          // ... if serial debug activated
        Serial.println( (String)"LoRaWAN: Busy   " + LoRaWAN_Busy )   ;               // ... Display state of LoRa - 0 for false (= available) - 1 for true (= busy)
        Serial.println( (String)".        Joined " + LoRaWAN_Joined ) ;               // ... Display LoRa connection - 0 for false (= not joined) - 1 for true (= joined)
        Serial.println(         ".        Msg not send") ;                            // ...
      }

      return 0 ;                                                                      // Msg was not send

    } // else

}

/**
 * @brief LoRa - Send LoRa Payload
 * 
 * @param Enable_SerialPrint_LoRa If true, serial printing is enabled, otherwise disabled.
 * 
 * @note This function is used to send the LoRa payload. If it fails, it will try 5 times with 10 sec between each send.
 *
 */
void LoRa_SendPayload( bool Enable_SerialPrint_LoRa ) {

  int Nb_Try   = 0 ;                                              // Number of try
  int Msg_Send = 0 ;                                              // If equal to 0 then the message was not send, if equal to 1 the message was send.

  while( ( Msg_Send == 0 ) && ( Nb_Try < 5 ) ){                   // Try 5 times to send the payload 

    if( Nb_Try > 0 ){ delay( 10000 ) ; }                              // ... wait 10 second before trying

    Nb_Try = Nb_Try + 1 ;                                             // ... increment try number
    Msg_Send = LoRa_SendMsg( Enable_SerialPrint_LoRa ) ;              // ... try to send the payload
    Serial.println( (String)".        Msg_Send : " + Msg_Send ) ;     // ... display if the message was send (=1) or not (=0)
    Serial.println( (String)".        Nb_Try   : " + Nb_Try   ) ;     // ... display try number

    // Serial.println( (String)"Test 1 : " + (Msg_Send==0) ) ;        // ... debug for condition n°1
    // Serial.println( (String)"Test 2 : " + (Nb_Try<=5) )   ;        // ... debug for condition n°2

  } // while( ( Msg_Send == 0 ) && ( Nb_Try < 5 ) )

}

#endif


/* >>> Flash Memory <<< */
#if( Use_Flash == true ) 
/**
 * @brief STM32L0 - Flash Last Address Used
 *
 * @return uint32_t flashAddress_LastAddressWritten Last address on which we wrote
 * 
 */
uint32_t flash_ReadLastAddressUsed( ){

    /* >>> Get the flashaddress where data has been written <<< */
    
    STM32L0.flashRead( 0x8021980 - 128 , data , 4 ) ; // Read the flash
  
    // Extract the information
    uint16_t  Value_Data_01   = ( data[0]        << 8 ) | data[1] ; // 1000 0001 0000 0010
    uint32_t  Value_Data_012  = ( Value_Data_01  << 8 ) | data[2] ; // 1000 0001 0000 0010 0000 0011
    uint32_t  Value_Data_0123 = ( Value_Data_012 << 8 ) | data[3] ; // 1000 0001 0000 0010 0000 0011 0000 0101

    Value_Data_0123 = Value_Data_0123 / 10 ; // Rescale

    Serial.print( "Binaire : ") ; Serial.println( Value_Data_0123 , BIN ) ;
    Serial.print( "Hexa : 0x") ; Serial.println( Value_Data_0123 , HEX ) ;
    Serial.print( "Decimal : ") ; Serial.println( Value_Data_0123 , DEC ) ;

    flashAddress_Updated = Value_Data_0123 ;
    Serial.print( "You should start writting at the following address : 0x" ) ; Serial.println( flashAddress_Updated , HEX ) ;

    return flashAddress_Updated ;

}
/**
 * @brief STM32L0 - Flash Write Data
 *
 * @param flashAddress_LastAddressWritten
 * 
 * @return flashAddress_LastAddressWritten
 */
uint32_t flash_StockData( uint32_t flashAddress_LastAddressWritten ){


    Serial.print( "Time flashAddress \t" ) ; Serial.print( flashAddress_LastAddressWritten , HEX ) ; Serial.print("\t") ;
    //int Date = 2306301000 ; int Date_Second = 1 ;
    flashAddress_LastAddressWritten = Flash_PushToMemory_Time( Date , flashAddress_LastAddressWritten , Enable_SerialPrint_Flash ) ;

    Serial.print( "GPS flashAddress \t" ) ; Serial.print( flashAddress_LastAddressWritten , HEX ) ; Serial.print("\t") ;
    // float GPS_Latitude = 43.010203 ;
    // float GPS_Longitude = 3.040506 ;
    // float GPS_NbSatellites = 15 ;
    int GPS_Latitude_4_Flash = GPS_Latitude * 1000000 ; // Rescale in order to put in the flash
    int GPS_Longitude_4_Flash = GPS_Longitude * 1000000 ; // --
    int GPS_Longitude_And_NbSatellites = GPS_Longitude_4_Flash * 100 + GPS_NbSatellites ; // Combine 2 Data to save bits in flash
    flashAddress_LastAddressWritten = Flash_PushToMemory_GPS( GPS_Latitude_4_Flash , GPS_Longitude_And_NbSatellites , flashAddress_LastAddressWritten , Enable_SerialPrint_Flash ) ;

    Serial.print( "Acc flashAddress \t" ) ; Serial.print( flashAddress_LastAddressWritten , HEX ) ; Serial.print("\t") ;
    Serial.println( (String)"Acc_X : " + Acc_X );
    Serial.println( (String)"Acc_X : " + Acc_Y );
    Serial.println( (String)"Acc_X : " + Acc_Z );
    flashAddress_LastAddressWritten = Flash_PushToMemory_Acc( 0 , Date_Second , 0 , (int)LIS2DWS12_Temperature , flashAddress_LastAddressWritten , Enable_SerialPrint_Flash ) ;
    // We don't use the accelerometer data at this time

    Serial.print( "Other flashAddress \t" ) ; Serial.print( flashAddress_LastAddressWritten , HEX ) ; Serial.print("\t") ;
    int VBattery = BatteryTension * 10 ;
    flashAddress_LastAddressWritten = Flash_PushToMemory_Vbat_LoRa( VBattery , LoRaWAN_Busy , LoRaWAN_Joined , flashAddress_LastAddressWritten , Enable_SerialPrint_Flash ) ;

    // Update the flash address that contain the last flash address written
    STM32L0.flashErase( 0x08021980 - 128 , 128 ) ;                                        // Before writting, we need to erase the flashaddress where we want to write 
    // flashAddress = flashAddress + 4 ;
    Flash_Push_to_Memory( flashAddress_LastAddressWritten * 10 , 0x8021980 - 128 ) ;                 // Then we can write. (Doesn't work if not x10... Don't know why)

    return flashAddress_LastAddressWritten ;
    
}
/**
 * @brief STM32L0 - Flash Read And Print Data
 *
 * @param flashAddress Flash address on which to read data (full page readout)
 *
 */
void flash_ReadNPrintData( uint32_t flashAddress ){
  uint8_t  data[128];
  STM32L0.flashRead( flashAddress, data , 128 ) ;
  STM32L0.Flash_Print_Data( flashAddress , data, 128 );
}
#endif