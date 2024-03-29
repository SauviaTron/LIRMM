#ifndef _MY_LPP_H_
#define _MY_LPP_H_

#include <Arduino.h>

//LPP_BATTERY = // TODO Unsupported in IPSO Smart Object
//LPP_PROXIMITY = // TODO Unsupported in IPSO Smart Object

#define LPP_DIGITAL_INPUT       0       // 1 byte
#define LPP_DIGITAL_OUTPUT      1       // 1 byte
#define LPP_ANALOG_INPUT        2       // 2 bytes, 0.01 signed
#define LPP_ANALOG_OUTPUT       3       // 2 bytes, 0.01 signed
#define LPP_LUMINOSITY          101     // 2 bytes, 1 lux unsigned
#define LPP_PRESENCE            102     // 1 byte, 1
#define LPP_TEMPERATURE         103     // 2 bytes, 0.1°C signed
#define LPP_RELATIVE_HUMIDITY   104     // 1 byte, 0.5% unsigned
#define LPP_ACCELEROMETER       113     // 2 bytes per axis, 0.001G -- 113 = 0x71
#define LPP_BAROMETRIC_PRESSURE 115     // 2 bytes 0.1 hPa Unsigned
#define LPP_GYROMETER           134     // 2 bytes per axis, 0.01 °/s
#define LPP_GPS                 136     // 3 byte lon/lat 0.0001 °, 3 bytes alt 0.01 meter
#define LPP_BatteryLevel        105     // 2 byte -- 105 = 0x69


// Data ID + Data Type + Data Size
#define LPP_DIGITAL_INPUT_SIZE       3       // 1 byte
#define LPP_DIGITAL_OUTPUT_SIZE      3       // 1 byte
#define LPP_ANALOG_INPUT_SIZE        4       // 2 bytes, 0.01 signed
#define LPP_ANALOG_OUTPUT_SIZE       4       // 2 bytes, 0.01 signed
#define LPP_LUMINOSITY_SIZE          4       // 2 bytes, 1 lux unsigned
#define LPP_PRESENCE_SIZE            3       // 1 byte, 1
#define LPP_TEMPERATURE_SIZE         4       // 2 bytes, 0.1°C signed
#define LPP_RELATIVE_HUMIDITY_SIZE   3       // 1 byte, 0.5% unsigned
#define LPP_ACCELEROMETER_SIZE       8       // 2 bytes per axis, 0.001G
#define LPP_ACC_AND_TEMP_SIZE        10      // 2 bytes per axis, 0.001G AND 1byte for temp(int) = 2*3 + 2 = 8 + 2 (ID = 0x--71) = 10
#define LPP_BAROMETRIC_PRESSURE_SIZE 4       // 2 bytes 0.1 hPa Unsigned
#define LPP_GYROMETER_SIZE           8       // 2 bytes per axis, 0.01 °/s
#define LPP_GPS_SIZE                 11      // 3 byte lon/lat 0.0001 °, 3 bytes alt 0.01 meter
#define LPP_BatteryLevel_SIZE        4       // 2 bytes
#define LPP_DIGIT_SIZE               3       // 2 bytes



class MyLPP {
    public:
        MyLPP(uint8_t size);
        ~MyLPP();
        
        void reset(void);
        uint8_t getSize(void);
        uint8_t* getBuffer(void);
        uint8_t copy(uint8_t* buffer);
        
        uint8_t addDigitalInput(uint8_t channel, uint8_t value);
        uint8_t addDigitalOutput(uint8_t channel, uint8_t value);

        uint8_t addAnalogInput(uint8_t channel, float value);
        uint8_t addAnalogOutput(uint8_t channel, float value);

        uint8_t addLuminosity(uint8_t channel, uint16_t lux);
        uint8_t addPresence(uint8_t channel, uint8_t value);
        //uint8_t addTemperature(uint8_t channel, float celsius);
        uint8_t addTemperature(float celsius);
        uint8_t addRelativeHumidity(uint8_t channel, float rh);
        //uint8_t addAccelerometer(uint8_t channel, float x, float y, float z);
        uint8_t addAccelerometer(float x, float y, float z);
        uint8_t addAccelerometer_And_Temperature(uint8_t channel, float x, float y, float z, float celsius);
        uint8_t addBarometricPressure(uint8_t channel, float hpa);
        uint8_t addGyrometer(uint8_t channel, float x, float y, float z);
        //uint8_t addGPS(uint8_t channel, float latitude, float longitude);
        uint8_t addGPS(float latitude, float longitude);
        uint8_t addDigit(uint8_t value);

        //uint8_t addBatteryLevel(uint8_t channel, float celsius);
        uint8_t addBatteryLevel(float BatterieTension);
    
    private:
        uint8_t *buffer;
        uint8_t maxsize;
        uint8_t cursor;
        
        
};


#endif
