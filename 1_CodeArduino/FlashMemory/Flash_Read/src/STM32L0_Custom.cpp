/*
 * Copyright (c) 2017-2018 Thomas Roell.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Thomas Roell, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#include "Arduino.h"
#include "STM32L0_Custom.h"
#include "wiring_private.h"




/* >>> BLUE LED <<< */
#define Blue_LED     10            // Blue led 

uint64_t STM32L0Class::getSerial()
{
    uint32_t serial0, serial1, serial2;

    /* This crummy value is what the USB/DFU bootloader uses.
     */

    serial0 = *((uint32_t*)0x1ff80050);
    serial1 = *((uint32_t*)0x1ff80054);
    serial2 = *((uint32_t*)0x1ff80058);

    serial0 += serial2;

    return (((uint64_t)serial0 << 16) | ((uint64_t)serial1 >> 16));
}

void STM32L0Class::getUID(uint32_t uid[3])
{
    stm32l0_system_uid(uid);
}

bool STM32L0Class::getVBUS()
{
#if defined(STM32L0_CONFIG_PIN_VBUS)
    if (STM32L0_CONFIG_PIN_VBUS == STM32L0_GPIO_PIN_NONE) {
	return false;
    }

    return !!stm32l0_gpio_pin_read(STM32L0_CONFIG_PIN_VBUS);

#else /* defined(STM32L0_CONFIG_PIN_VBUS */

    return false;

#endif /* defined(STM32L0_CONFIG_PIN_VBUS */
}

float STM32L0Class::getVBAT()
{
#if defined(STM32L0_CONFIG_PIN_VBAT)
    int32_t vrefint_data, vbat_data;
    float vdda;

    vrefint_data = __analogReadInternal(STM32L0_ADC_CHANNEL_VREFINT, STM32L0_ADC_VREFINT_PERIOD);
    vbat_data = __analogReadInternal(STM32L0_CONFIG_CHANNEL_VBAT, STM32L0_CONFIG_VBAT_PERIOD);

    vdda = (3.0 * STM32L0_ADC_VREFINT_CAL) / vrefint_data;

    return (STM32L0_CONFIG_VBAT_SCALE * vdda * vbat_data) / 4095.0;

#else /* defined(STM32L0_CONFIG_PIN_VBAT) */
    
    return -1;

#endif /* defined(STM32L0_CONFIG_PIN_VBAT) */
}

float STM32L0Class::getVDDA()
{
    int32_t vrefint_data;

    vrefint_data = __analogReadInternal(STM32L0_ADC_CHANNEL_VREFINT, STM32L0_ADC_VREFINT_PERIOD);

    return (3.0 * STM32L0_ADC_VREFINT_CAL) / vrefint_data;
}

float STM32L0Class::getTemperature()
{
    int32_t vrefint_data, tsense_data;

    vrefint_data = __analogReadInternal(STM32L0_ADC_CHANNEL_VREFINT, STM32L0_ADC_VREFINT_PERIOD);
    tsense_data = __analogReadInternal(STM32L0_ADC_CHANNEL_TSENSE, STM32L0_ADC_TSENSE_PERIOD);

    /* Compensate TSENSE_DATA for VDDA vs. 3.0 */
    tsense_data = (tsense_data * STM32L0_ADC_VREFINT_CAL) / vrefint_data;

    return (30.0 + (100.0 * (float)(tsense_data - STM32L0_ADC_TSENSE_CAL1)) / (float)(STM32L0_ADC_TSENSE_CAL2 - STM32L0_ADC_TSENSE_CAL1));
}

uint32_t STM32L0Class::resetCause()
{
    return stm32l0_system_reset_cause();
}

void STM32L0Class::wakeup()
{
    stm32l0_system_wakeup();
}

void STM32L0Class::sleep(uint32_t timeout)
{
    stm32l0_system_sleep(STM32L0_SYSTEM_POLICY_SLEEP, timeout);
}

void STM32L0Class::stop(uint32_t timeout)
{
    if (g_swdStatus == 0) {
	stm32l0_system_swd_disable();

	g_swdStatus = 2;
    }

    stm32l0_system_sleep(STM32L0_SYSTEM_POLICY_STOP, timeout);
}

void STM32L0Class::standby()
{
    stm32l0_system_standby(0);
}

void STM32L0Class::standby(uint32_t pin)
{
    uint32_t config;

    if ( (pin >= PINS_COUNT) || !(g_APinDescription[pin].attr & (PIN_ATTR_WKUP1 | PIN_ATTR_WKUP2)))  {
	return;
    }
    
    if (g_APinDescription[pin].attr & PIN_ATTR_WKUP1) {
	config = STM32L0_SYSTEM_CONFIG_WKUP1;
    }

    if (g_APinDescription[pin].attr & PIN_ATTR_WKUP2) {
	config = STM32L0_SYSTEM_CONFIG_WKUP2;
    }

    stm32l0_system_standby(config);
}

void STM32L0Class::reset()
{
    stm32l0_system_reset();
}

void STM32L0Class::swdEnable()
{
    if (g_swdStatus != 3) {
	stm32l0_system_swd_enable();

	g_swdStatus = 1;
    }
}

void STM32L0Class::swdDisable()
{
    if (g_swdStatus != 3) {
	stm32l0_system_swd_disable();

	g_swdStatus = 2;
    }
}

void STM32L0Class::wdtEnable(uint32_t timeout)
{
    stm32l0_iwdg_enable(timeout);
}

void STM32L0Class::wdtReset()
{
    stm32l0_iwdg_reset();
}

uint32_t STM32L0Class::flashSize( )
{
    stm32l0_flash_size() ;
}

bool STM32L0Class::flashErase(uint32_t address, uint32_t count)
{
    if (address & 127) {
	return false;
    }

    count = (count + 127) & ~127;

    if ((address < FLASHSTART) || ((address + count) > FLASHEND)) {
	return false;
    }

    stm32l0_flash_unlock();
    stm32l0_flash_erase(address, count);
    stm32l0_flash_lock();
    
    return true;
}

bool STM32L0Class::flashProgram(uint32_t address, const void *data, uint32_t count)
{
    if ((address & 3) || (count & 3)) {
	return false;
    }

    if ((address < FLASHSTART) || ((address + count) > FLASHEND)) {
	return false;
    }

    if (count) {
	stm32l0_flash_unlock();
	stm32l0_flash_program(address, (const uint8_t*)data, count);
	stm32l0_flash_lock();
    }

    return true;
}

bool STM32L0Class::flashRead( uint32_t address, uint8_t *data, uint32_t count)
{
    if ((address & 3) || (count & 3) || (address < FLASH_BASE) || ((address + count) > (FLASH_BASE + stm32l0_flash_size())) ) {
	return false;
    }

    memcpy( data, (const uint8_t*)address, count);

    return true ;

}

/**
 * @brief STM32L0 - Flash display data
 *
 * @param address Flash address of the data
 * @param data Variable that contain the data stored
 * @param count Number of data that you want to read
 * 
 */
void STM32L0Class::Flash_Print_Data( uint32_t address, uint8_t *data, uint32_t count ){

    int Nb_Data_Display = 0 ;
    int Flash_Nb_Cell = 1 ;

    for (int i = 0; i < count; i++) {
      
        if( Nb_Data_Display == 0 ){ 

            Serial.print( "Flash address : 0x" ) ; Serial.print( address, HEX ) ; Serial.print("\t") ;
            address = address + 4 ;
            Flash_Nb_Cell += 1 ; 

        //   if( Flash_Nb_Cell <= 9 ){
        //     Serial.print( (String)"Cell n 0" + Flash_Nb_Cell) ; Serial.print("\t") ;
        //     Flash_Nb_Cell += 1 ; 
        //   }
        //   else{
        //     Serial.print( (String)"Cell n " + Flash_Nb_Cell) ; Serial.print("\t") ;
        //     Flash_Nb_Cell += 1 ; 
        //   }

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
}

/**
 * @brief STM32L0 - Flash decode data
 *
 * @param address Flash address of the data
 * @param data Variable that contain the data stored
 * @param previousdata Variable that contain the previous data stored
 * @param count Number of data that you want to read
 * @param Board_Name_Figure Gnat number for "GNAT L082CZ - xx"
 * 
 */
void STM32L0Class::Flash_Decode_Data( uint32_t address, uint8_t *data, uint8_t *previousdata, uint32_t count, int Board_Name_Figure, int Data_Read ){

    int Nb_Data_Display = 0 ;
    int Flash_Nb_Cell = 1 ;

    int i = 0 ;
    //for ( int i = 0 ; i < count ; i = i + 20 ) {
      
        // if ( Nb_Data_Display == 0 ) { 

        //     Serial.print( "Flash address : 0x" ) ; Serial.print( address, HEX ) ; Serial.print("\t") ;
        //     address = address + 4 ;
        //     Flash_Nb_Cell += 1 ; 

        // }
        
        uint8_t   Value_Data_0 ;
        uint8_t   Value_Data_1 ;
        uint8_t   Value_Data_2 ;
        uint8_t   Value_Data_3 ;
        uint16_t  Value_Data_01   ; 
        uint32_t  Value_Data_012  ; 
        uint32_t  Value_Data_0123 ; 


        /* >>> Get Time <<< */
        Value_Data_01   = ( data[i+0]      << 8 ) | data[i+1] ; 
        Value_Data_012  = ( Value_Data_01  << 8 ) | data[i+2] ; 
        Value_Data_0123 = ( Value_Data_012 << 8 ) | data[i+3] ; 
        uint32_t Data_Time = Value_Data_0123 ;
        // Serial.print( "Data_Time : ") ; Serial.print( Data_Time , BIN ) ; Serial.print( " \t ") ; Serial.println( Data_Time , DEC ) ;
        String dateString    = String(Data_Time);
        String year          = dateString.substring(0, 2);
        String month         = dateString.substring(2, 4);
        String day           = dateString.substring(4, 6);
        String hour          = dateString.substring(6, 8);
        String minute        = dateString.substring(8, 10);
        Value_Data_1 = data[i+1+4*3] ; String second = String(Value_Data_1) ;
        String formattedDate = year + "." + month + "." + day + " - " + hour + "h" + minute + "s" + second ;
        // Serial.println( (String)"Date : " + formattedDate) ; 

        Value_Data_01   = ( previousdata[i+0]  << 8 ) | previousdata[i+1] ; 
        Value_Data_012  = ( Value_Data_01      << 8 ) | previousdata[i+2] ; 
        Value_Data_0123 = ( Value_Data_012     << 8 ) | previousdata[i+3] ; 
        uint32_t Data_previousTime = Value_Data_0123 ;
        // Serial.print( "Data_previousTime : ") ; Serial.print( Data_previousTime , BIN ) ; Serial.print( " \t ") ; Serial.println( Data_previousTime , DEC ) ;
        String previousdateString    = String(Data_previousTime);
        String previousyear          = previousdateString.substring(0, 2);
        String previousmonth         = previousdateString.substring(2, 4);
        String previousday           = previousdateString.substring(4, 6);
        String previoushour          = previousdateString.substring(6, 8);
        String previousminute        = previousdateString.substring(8, 10);
        Value_Data_1 = previousdata[i+1+4*3] ; String previoussecond = String(Value_Data_1) ;
        String previousformattedDate = previousyear + "." + previousmonth + "." + previousday + " - " + previoushour + "h" + previousminute + "s" + previoussecond ;
        // Serial.println( (String)"Previous date : " + previousformattedDate) ; 
        

        // Calculer la différence de temps en secondes
        unsigned long timeDiff = (( year.toInt()   - previousyear.toInt()   ) * 365 * 24 * 60 * 60 ) +
                                 (( month.toInt()  - previousmonth.toInt()  ) *  30 * 24 * 60 * 60 ) +
                                 (( day.toInt()    - previousday.toInt()    ) *  24 * 60 * 60 ) +
                                 (( hour.toInt()   - previoushour.toInt()   ) *  60 * 60 ) +
                                 (( minute.toInt() - previousminute.toInt() ) *  60 ) +
                                 (( second.toInt() - previoussecond.toInt() )) ;
        // Serial.println( (String)"Time difference (s) : " + timeDiff );


        // Calculer le nombre de jours écoulés depuis le 1er janvier 1970
        unsigned long timestamp = ((year.toInt() - 1970) * 365 + (year.toInt() - 1969) / 4) * 24 * 60 * 60;  // Nombre de secondes pour les années complètes
        const int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};  // Nombre de jours dans chaque mois
        for (int i = 0; i < month.toInt() - 1; i++) {
        timestamp += daysInMonth[i] * 24 * 60 * 60;  // Ajouter les secondes pour les mois complets
        }
        timestamp += (day.toInt() - 1) * 24 * 60 * 60;  // Ajouter les secondes pour les jours
        timestamp += hour.toInt() * 60 * 60;  // Ajouter les secondes pour les heures
        timestamp += minute.toInt() * 60;  // Ajouter les secondes pour les minutes
        // Ne fonctionne pas


        /* >>> Board Identity <<< */
        String devEUI = "3D3B8F7A9F26C0";  // Chaîne de caractères initiale
        // Ajouter le nombre à la chaîne de caractères
        if( Board_Name_Figure < 10 ) { devEUI += "0"; } // Ajouter un zéro avant le nombre si nécessaire 
        devEUI += String(Board_Name_Figure);  // Ajouter le nombre à la chaîne


        /* >>> Get GPS Latitude, Longitude & Nb Satellites <<< */
        Value_Data_01   = ( data[i+0+4]    << 8 ) | data[i+1+4] ; 
        Value_Data_012  = ( Value_Data_01  << 8 ) | data[i+2+4] ; 
        Value_Data_0123 = ( Value_Data_012 << 8 ) | data[i+3+4] ; 
        uint32_t Data_GPSLatitude = Value_Data_0123 ;
        float Data_GPSLatitude_float = Data_GPSLatitude / 1000000.0f ; 
        // Serial.print( "Data_GPSLatitude : ") ; Serial.print( Data_GPSLatitude , BIN ) ; Serial.print( " \t ") ; Serial.print( Data_GPSLatitude , DEC ) ;
        // Serial.print( "\t " ) ;  Serial.println(Data_GPSLatitude_float , 6 ) ;
        Value_Data_01   = ( data[i+0+4*2]  << 8 ) | data[i+1+4*2] ; 
        Value_Data_012  = ( Value_Data_01  << 8 ) | data[i+2+4*2] ; 
        Value_Data_0123 = ( Value_Data_012 << 8 ) | data[i+3+4*2] ; 
        uint32_t Data_GPSLongitudeAndNbSatellites = Value_Data_0123 ;
        float Data_GPSLongitude_float = Data_GPSLongitudeAndNbSatellites / 100000000.0f ;
        // Serial.print( "Data_GPSLongitudeAndNbSatellites : ") ; Serial.print( Data_GPSLongitudeAndNbSatellites , BIN ) ; Serial.print( " \t ") ; Serial.print( Data_GPSLongitudeAndNbSatellites , DEC ) ;
        // Serial.print( "\t " ) ;  Serial.println(Data_GPSLongitude_float , 6 );
        String Data_GPSNbSatellites = String( Data_GPSLongitudeAndNbSatellites ).substring(7,9);

        Value_Data_01   = ( previousdata[i+0+4]      << 8 ) | previousdata[i+1+4] ; 
        Value_Data_012  = ( Value_Data_01  << 8 ) | previousdata[i+2+4] ; 
        Value_Data_0123 = ( Value_Data_012 << 8 ) | previousdata[i+3+4] ; 
        uint32_t Data_previousGPSLatitude = Value_Data_0123 ;
        float Data_previousGPSLatitude_float = Data_previousGPSLatitude / 1000000.0f ;
        // Serial.print( "Data_previousGPSLatitude : ") ; Serial.print( Data_previousGPSLatitude , BIN ) ; Serial.print( " \t ") ; Serial.print( Data_GPSLatitude , DEC ) ;
        // Serial.print( "\t " ) ;  Serial.println(Data_GPSLatitude_float , 6 );
        Value_Data_01   = ( previousdata[i+0+4*2]    << 8 ) | previousdata[i+1+4*2] ; 
        Value_Data_012  = ( Value_Data_01  << 8 ) | previousdata[i+2+4*2] ; 
        Value_Data_0123 = ( Value_Data_012 << 8 ) | previousdata[i+3+4*2] ; 
        uint32_t Data_previousGPSLongitudeAndNbSatellites = Value_Data_0123 ;
        float Data_previousGPSLongitude_float = Data_previousGPSLongitudeAndNbSatellites / 100000000.0f ;
        // Serial.print( "Data_previousGPSLongitudeAndNbSatellites : ") ; Serial.print( Data_previousGPSLongitudeAndNbSatellites , BIN ) ; Serial.print( " \t ") ; Serial.print( Data_previousGPSLongitudeAndNbSatellites , DEC ) ;
        // Serial.print( "\t " ) ;  Serial.println(Data_previousGPSLongitude_float , 6 );


        /* >>> Get distance (m) between current point and previous point <<< */
        float B2 = Data_GPSLatitude_float ; 
        float C2 = Data_GPSLongitude_float ;
        float B3 = Data_previousGPSLatitude_float ; 
        float C3 = Data_previousGPSLongitude_float ;
        // Serial.println( (String)"B2 : " + B2 );
        // Serial.println( (String)"B3 : " + B3 );
        // Serial.println( (String)"C2 : " + C2 );
        // Serial.println( (String)"C3 : " + C3 );
        float GPS_Distance = acos( sin(radians(B2))*sin(radians(B3)) + cos(radians(B2))*cos(radians(B3))*cos(radians(C2-C3)) ) * 6371000 ;
        // Serial.println( (String)"GPS_Distance : " + GPS_Distance ) ;

        float GPS_Vitesse = GPS_Distance / timeDiff ;
        // Serial.println( "GPS_Vitesse : " ) ; Serial.println( GPS_Distance , 2 ) ;

        /* >>> Get GPS direction (deg) <<< */
        float dLon = ( Data_GPSLongitude_float - Data_previousGPSLongitude_float );
        float x = sin( dLon ) * cos( Data_GPSLatitude_float ) ;
        float y = cos( Data_previousGPSLatitude_float ) * sin( Data_GPSLatitude_float ) - sin( Data_previousGPSLatitude_float ) * cos( Data_GPSLatitude_float ) * cos( dLon ) ;
        float GPS_Direction = atan( x / y ) ;
        // Serial.print( "Angle : " ) ; Serial.println( GPS_Direction , 2 ) ;


        /* >>> Get Acc XYZ & Temperature <<< */
        Value_Data_0 = data[i+0+4*3] ; 
        Value_Data_1 = data[i+1+4*3] ; 
        Value_Data_2 = data[i+2+4*3] ;
        Value_Data_3 = data[i+3+4*3] ;
        uint8_t Data_Acc_X = Value_Data_0 ;
        //uint8_t Data_Acc_Y = Value_Data_1 ;
        uint8_t Data_Acc_Y = 0 ;
        uint8_t Data_Acc_Z = Value_Data_2 ;
        uint8_t Data_Acc_Temperature = Value_Data_3 ;
        // Serial.print( "Data_Acc_X : ") ; Serial.print( Data_Acc_X , BIN ) ; Serial.print( " \t ") ; Serial.println( Data_Acc_X , DEC ) ;
        // Serial.print( "Data_Acc_Y : ") ; Serial.print( Data_Acc_Y , BIN ) ; Serial.print( " \t ") ; Serial.println( Data_Acc_Y , DEC ) ;
        // Serial.print( "Data_Acc_Z : ") ; Serial.print( Data_Acc_Z , BIN ) ; Serial.print( " \t ") ; Serial.println( Data_Acc_Z , DEC ) ;
        // Serial.print( "Data_Acc_Temperature : ") ; Serial.print( Data_Acc_Temperature , BIN ) ; Serial.print( " \t ") ; Serial.println( Data_Acc_Temperature , DEC ) ;

        float roll = atan2(Data_Acc_Y, Data_Acc_Z) * (180.0 / PI);
        float pitch = atan2(-Data_Acc_X, sqrt(Data_Acc_Y * Data_Acc_Y + Data_Acc_Z * Data_Acc_Z)) * (180.0 / PI);
        float yaw = atan2(Data_Acc_Z, Data_Acc_X) * (180.0 / PI);


        /* >>> Get VBattery & LoRaWAN.Busy/Joined <<< */
        Value_Data_01 = ( data[i+0+4*4] << 8 ) | data[i+1+4*4] ; 
        Value_Data_2  = data[i+2+4*4] ; 
        Value_Data_3  = data[i+3+4*4] ;
        uint32_t Data_VBattery = Value_Data_01 ;
        uint32_t Data_LoRaWANBusy = Value_Data_2 ;
        uint32_t Data_LoRaWANJoined = Value_Data_3 ;
        // Serial.print( "Data_VBattery : ") ; Serial.print( Data_VBattery , BIN ) ; Serial.print( " \t ") ; Serial.println( Data_VBattery , DEC ) ;
        // Serial.print( "Data_LoRaWANBusy : ") ; Serial.print( Data_LoRaWANBusy , BIN ) ; Serial.print( " \t ") ; Serial.println( Data_LoRaWANBusy , DEC ) ;
        // Serial.print( "Data_LoRaWANJoined : ") ; Serial.print( Data_LoRaWANJoined , BIN ) ; Serial.print( " \t ") ; Serial.println( Data_LoRaWANJoined , DEC ) ;

        
        /* >>> Display data in a csv looking <<< */
        
        Serial.print( "GNAT L082CZ - ") ;
        if( Board_Name_Figure < 10){ Serial.print("0"); }
        Serial.print(Board_Name_Figure) ; Serial.print(";") ;

        if( Data_VBattery > 10){ Data_VBattery = 0 ; } else{ Data_VBattery = Data_VBattery + 2.0 ;  }
        Serial.print( Data_VBattery , DEC ) ; Serial.print( ";" ) ;

        // String dateString    = String(Data_Time);
        // String year          = dateString.substring(0, 2);
        // String month         = dateString.substring(2, 4);
        // String day           = dateString.substring(4, 6);
        // String hour          = dateString.substring(6, 8);
        // String minute        = dateString.substring(8, 10);
        // String formattedDate = year + "." + month + "." + day + " - " + hour + "h" + minute;
        Serial.print(formattedDate) ; Serial.print( ";" ) ;

        // String previousdateString    = String(Data_previousTime);
        // String previousyear          = previousdateString.substring(0, 2);
        // String previousmonth         = previousdateString.substring(2, 4);
        // String previousday           = previousdateString.substring(4, 6);
        // String previoushour          = previousdateString.substring(6, 8);
        // String previousminute        = previousdateString.substring(8, 10);
        // String previousformattedDate = previousyear + "." + previousmonth + "." + previousday + " - " + previoushour + "h" + previousminute ;
        Serial.print(previousformattedDate) ; Serial.print( ";" ) ;

        // Calculer la différence de temps en secondes
        // unsigned long timeDiff = (( year.toInt()   - previousyear.toInt()   ) * 365 * 24 * 60 * 60 ) +
        //                          (( month.toInt()  - previousmonth.toInt()  ) *  30 * 24 * 60 * 60 ) +
        //                          (( day.toInt()    - previousday.toInt()    ) *  24 * 60 * 60 ) +
        //                          (( hour.toInt()   - previoushour.toInt()   ) *  60 * 60 ) +
        //                          (( minute.toInt() - previousminute.toInt() ) *  60 ) ;
        Serial.print( timeDiff ); Serial.print(";");

        // // Calculer le nombre de jours écoulés depuis le 1er janvier 1970
        // unsigned long timestamp = ((year.toInt() - 1970) * 365 + (year.toInt() - 1969) / 4) * 24 * 60 * 60;  // Nombre de secondes pour les années complètes
        // const int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};  // Nombre de jours dans chaque mois
        // for (int i = 0; i < month.toInt() - 1; i++) {
        // timestamp += daysInMonth[i] * 24 * 60 * 60;  // Ajouter les secondes pour les mois complets
        // }
        // timestamp += (day.toInt() - 1) * 24 * 60 * 60;  // Ajouter les secondes pour les jours
        // timestamp += hour.toInt() * 60 * 60;  // Ajouter les secondes pour les heures
        // timestamp += minute.toInt() * 60;  // Ajouter les secondes pour les minutes

        // Serial.print(timestamp) ; Serial.print(" ; "); // Afficher le timestamp
        Serial.print( "0" ) ; Serial.print(";"); // Timestamp doesn't work

        Serial.print( "0" ) ; Serial.print(";"); // Flash doesn't stock STM32 temp


        /* >>> LoRa codes <<< */
        // String devEUI = "3D3B8F7A9F26C0";  // Chaîne de caractères initiale
        // // Ajouter le nombre à la chaîne de caractères
        // if( Board_Name_Figure < 10 ) { devEUI += "0"; } // Ajouter un zéro avant le nombre si nécessaire 
        // devEUI += String(Board_Name_Figure);  // Ajouter le nombre à la chaîne
        Serial.print( devEUI ) ; Serial.print( ";" ) ;

        Serial.print( "0" ) ; Serial.print(";"); // ... LoRa_Network_Quality
        Serial.print( "0" ) ; Serial.print(";"); // ... LoRa_RSSI
        Serial.print( "0" ) ; Serial.print(";"); // ... LoRa_SNR
        Serial.print( "0" ) ; Serial.print(";"); // ... LoRa_ESP
        Serial.print( "0" ) ; Serial.print(";"); // ... LoRa_SF
        Serial.print( "0" ) ; Serial.print(";"); // ... LoRa_Frequency
        Serial.print( "0" ) ; Serial.print(";"); // ... LoRa_Nb_Gateways
        Serial.print( Data_Read ) ; Serial.print(";") ; // ... LoRa_fcnt
        Serial.print( "0" ) ; Serial.print(";"); // ... LoRa_Payload
        


        Serial.print(Data_GPSLatitude_float , 6 ) ; Serial.print(";");
        // String Data_GPSNbSatellites = String( Data_GPSLongitudeAndNbSatellites ).substring(7,9);
        Serial.print(Data_GPSLongitude_float , 6) ; Serial.print(";"); 
        Serial.print(Data_GPSNbSatellites) ;        Serial.print(";");
        Serial.print( "0" ) ;                       Serial.print(";"); // Flash don't store EHPE
        Serial.print(GPS_Distance , 2) ;             Serial.print(";");
        Serial.print(GPS_Vitesse , 2) ;             Serial.print(";");
        Serial.print(GPS_Direction , 2) ;               Serial.print(";");

        Serial.print( Data_Acc_Temperature , DEC ) ; Serial.print(";");
        Serial.print( Data_Acc_X , DEC ) ;           Serial.print(";");
        Serial.print( Data_Acc_Y , DEC ) ;           Serial.print(";");
        Serial.print( Data_Acc_Z , DEC ) ;           Serial.print(";");
        
        // float roll = atan2(Data_Acc_Y, Data_Acc_Z) * (180.0 / PI);
        // float pitch = atan2(-Data_Acc_X, sqrt(Data_Acc_Y * Data_Acc_Y + Data_Acc_Z * Data_Acc_Z)) * (180.0 / PI);
        // float yaw = atan2(Data_Acc_Z, Data_Acc_X) * (180.0 / PI);
        Serial.print( roll )  ; Serial.print(";");
        Serial.print( pitch ) ; Serial.print(";");
        Serial.print( yaw )   ; Serial.print(";");

        Serial.print( Data_LoRaWANBusy , DEC ) ; Serial.print(";");
        Serial.print( Data_LoRaWANJoined , DEC ) ; 
        
        
        
        
        Serial.println() ;
        
}


/**
 * @brief LED - Configure pinMode
 *
 * @param Enable_SerialPrint_LED If true, serial printing is enabled, otherwise disabled.
 *
 * @note Turn on the LED when it's done.
 */
void STM32L0Class::BlueLED_Config( bool Enable_SerialPrint_LED ){
 
  pinMode(Blue_LED, OUTPUT);      
  BlueLED_ON( Enable_SerialPrint_LED ) ;

}

/**
 * @brief LED - Turn on fonction
 *
 * @param Enable_SerialPrint_LED If true, serial printing is enabled, otherwise disabled.
 */
void STM32L0Class::BlueLED_ON( bool Enable_SerialPrint_LED ){
 
  digitalWrite(Blue_LED, LOW);

  if( Enable_SerialPrint_LED == true ){ Serial.println("LED: ON") ; }

}

/**
 * @brief LED - Turn off fonction
 *
 * @param Enable_SerialPrint_LED If true, serial printing is enabled, otherwise disabled.
 */
void STM32L0Class::BlueLED_OFF( bool Enable_SerialPrint_LED ){
 
  digitalWrite(Blue_LED, HIGH);

  if( Enable_SerialPrint_LED == true ){ Serial.println("LED: OFF") ; }

}

STM32L0Class STM32L0;
