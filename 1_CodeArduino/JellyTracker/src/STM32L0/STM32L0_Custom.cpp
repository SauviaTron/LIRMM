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


/* >>> Battery <<< */
#define Battery_Pin_ADC     A1 // LiPo battery ADC
#define Battery_Pin_Monitor  2 // LiPo battery monitor enable


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
    Serial.println( "Flash    Fail to erase" ) ;
	return false;
    }

    stm32l0_flash_unlock();
    stm32l0_flash_erase(address, count);
    stm32l0_flash_lock();
    
    Serial.println( "Flash    Erase succeded" ) ;
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
 * @brief STM32L0 - Wake-up the microcontroller
 *
 * @param Enable_SerialPrint_STM32 If true, serial printing is enabled, otherwise disabled.
 * 
 * @note Set a flag 'STM32_Sleeping' to false
 */
bool STM32L0Class::STM32_WakeUp( bool Enable_SerialPrint_STM32 ){

  wakeup() ;

  bool STM32_Sleeping = false ; // Setting the flag
    
  if( Enable_SerialPrint_STM32 == true ){ Serial.println(" ") ; Serial.println("STM32L0  Wake-Up"); } // WakeUp msg

  return STM32_Sleeping ;

}

/**
 * @brief STM32L0 - Put the microcontroller in stop mode
 *
 * @param Enable_SerialPrint_STM32 If true, serial printing is enabled, otherwise disabled.
 * 
 * @note Set a flag 'STM32_Sleeping' to true
 * 
 * @warning This function must be used with STM32_WakeUp() and a delay(10000) in setup! 
 * If not the board may never wake up
 */
bool STM32L0Class::STM32_StopMode( bool Enable_SerialPrint_STM32 ){

  bool STM32_Sleeping = true ; // Setting the flag
    
  if( Enable_SerialPrint_STM32 == true ){ Serial.println("STM32L0  Stop Mode"); } // Last msg

  stop() ; // Stop Mode

  return STM32_Sleeping ;

}

/**
 * @brief STM32L0 - Get the temperature of the microcontroller
 *
 * @param Enable_SerialPrint_STM32 If true, serial printing is enabled, otherwise disabled.
 * 
 * @return float STM32_Temperature_float = STM32L0.getTemperature
 */
float STM32L0Class::STM32_Temperature( bool Enable_SerialPrint_STM32 ){

  if( Enable_SerialPrint_STM32 == true ){ Serial.println( (String)".        Temperature : " + getTemperature() + "°" ); }

  float STM32_Temperature_float = getTemperature() ;

}

// void STM32_Flash_Write( bool Enable_SerialPrint_STM32 ){
//
//   uint32_t STM32_Flash_address = 0x8021980 ; 
//
//   /*
// https://www.st.com/resource/en/reference_manual/rm0376-ultralowpower-stm32l0x2-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
// page 62
//    * Corresponds to the address on the 1075th page. We start writing from here. We will use 30% of the flash memory. Be careful not to write.
//    *
//    * This value corresponds to the address of the 1075th page. The flash memory of the STM32L082CZ is 196kbytes. 
//    * It starts at address 0x80000000 and ends at address 0x8020FFFF.  There are 1535 pages, each offering 128bytes per page, i.e. 1024bits per page. 
//    * So there is a total of 1568kBytes available on the flash. When the code is uploaded, it is stored on the flash and therefore takes up space. 
//    * For example, it can take up 25% of the flash. So there is 75% unused space. This space can be used to store data.
//    * 
//    * In this code, we will use 30% of the flash, which is 460 pages or 58,944Bytes or 471,552bits. 
//    * To make sure we don't write our data on the code, we start at address 0x8021980 which corresponds to page number 1075. 
//    * 
//    * The memory allocation is done as follows:
//    * 
//    * 0% –––––––––––––––––––––––––––> 69% | 70% ––––––––––––––––––––> 100%
//    * .  Code implemented on the card     |     Memory space for data
//    * 
//   */
//
//   if (Message_Pushed_Count < 2 && page_number < 1536) {                          // 32,768 256-byte pages in a 8 MByte flash
//
//     data_pushed[ Message_Pushed_Count + 0 ] = 2302081615  ; // Date
//     data_pushed[ Message_Pushed_Count + 1 ] = 43000000    ; // Latitude
//     data_pushed[ Message_Pushed_Count + 2 ] = 3000000     ; // Longitude
//     data_pushed[ Message_Pushed_Count + 3 ] = 10          ; // Nb of satellites
//     data_pushed[ Message_Pushed_Count + 4 ] = 000         ; // Acc x
//     data_pushed[ Message_Pushed_Count + 5 ] = 001         ; // Acc y 
//     data_pushed[ Message_Pushed_Count + 6 ] = 002         ; // Acc z
//     data_pushed[ Message_Pushed_Count + 7 ] = 234         ; // Temperature
//     data_pushed[ Message_Pushed_Count + 8 ] = 11          ; // LoRa satus
//
//     Message_Pushed_Count ++ ;
//
//   }
//
//   else if (Message_Pushed_Count == 2 && page_number < 1536) { // if 8 number of msg or nbr of pages available
//  
//     // Unlocks the flash memory so that it can be programmed
//     STM32L0.flashUnlock();
//
//     // Program the data into flash memory
//     if( STM32L0.flashProgram(address, data_pushed, sizeof(data_pushed) ) ) { Serial.println("Données programmées en mémoire flash avec succès"); } 
//     else { Serial.println("Echec de la programmation en mémoire flash") ; }
//
//     // Verrouille la mémoire flash pour éviter tout autre accès
//     STM32L0.flashLock();
//
//     // Display a msg for each data wrote into the SPI flash
//     Serial.println( "STM32 Flash: Data wrote." ) ;
//
//     Message_Pushed_Count = 0 ; // Reset number of msg put into the page
//     STM32_Flash_address = STM32_Flash_address + 0x80 ; // Increment the page number
//     
//   }
//  
//   else { Serial.println("Reached last page of flash memory !"); Serial.println("Data logging stopped!"); } // Max page reached
//
//
//   // // Unlocks the flash memory so that it can be programmed
//   // STM32L0.flashUnlock();
//
//   // // Program the data into flash memory
//   // if (STM32L0.flashProgram(address, data_pushed, sizeof(datapushed)) {
//   //   Serial.println("Données programmées en mémoire flash avec succès");
//   // } else {
//   //   Serial.println("Echec de la programmation en mémoire flash");
//   // }
//
//   // // Verrouille la mémoire flash pour éviter tout autre accès
//   // STM32L0.flashLock();
//
// }


/* >>> BLUE LED <<< */

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


/**
 * @brief STM32L0 - Configure pinMode and resolution for battery monitoring
 *
 * @param Enable_SerialPrint_Battery If true, serial printing is enabled, otherwise disabled.
 * 
 * @note analogReadResolution( 12 ) ;
 * 
 */
void STM32L0Class::Battery_Config( bool Enable_SerialPrint_Battery ){

  if( Enable_SerialPrint_Battery == true ){ Serial.println( "Battery: Battery_Config") ; }

  pinMode(Battery_Pin_Monitor, OUTPUT) ;
  pinMode(Battery_Pin_ADC, INPUT)      ; // set up ADC battery voltage monitor pin
  analogReadResolution(12)             ; // use 12-bit ADC resolution

}

/**
 * @brief STM32L0 - Read the tension of the battery
 *
 * @param Enable_SerialPrint_Battery If true, serial printing is enabled, otherwise disabled.
 * 
 * @return float BatteryTension
 * 
 */
float STM32L0Class::Battery_GetTension(bool Enable_SerialPrint_Battery){

    float VDDA = getVDDA();
    digitalWrite(Battery_Pin_Monitor, HIGH);
    float BatteryTension = 1.27f * VDDA * ((float)analogRead(Battery_Pin_ADC)) / 4096.0f;
    digitalWrite(Battery_Pin_Monitor, LOW);

    if( Enable_SerialPrint_Battery == true ){ Serial.print( ".        Battery : ") ; Serial.print(BatteryTension, 2) ; Serial.println(" V") ; }

    return BatteryTension ;

}


STM32L0Class STM32L0;
