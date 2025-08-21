#ifndef __SPARKFUN_BMP581_H__
#define __SPARKFUN_BMP581_H__

#include "Arduino.h"
#include "../../hal/MF_I2C.h" //modified for madflight
#include <SPI.h>

#include "bmp5_api/bmp5.h"

// SparkFun's default I2C address is opposite of Bosch's default
#define BMP581_I2C_ADDRESS_DEFAULT BMP5_I2C_ADDR_SEC    // 0x47
#define BMP581_I2C_ADDRESS_SECONDARY BMP5_I2C_ADDR_PRIM // 0x46

// Generic error code for invalid settings
#define BMP5_E_INVALID_SETTING (BMP5_E_NVM_NOT_READY-1)

// Struct to hold data about the communication interface being used (I2C or SPI)
struct BMP581_InterfaceData
{
    // Communication interface (I2C or SPI)
    bmp5_intf interface;

    // I2C settings
    uint8_t i2cAddress;
    MF_I2C *i2cPort;

    // SPI settings
    uint8_t spiCSPin;
    uint32_t spiClockFrequency;
};

// Struct to hold interrupt config data
struct BMP581_InterruptConfig
{
    // Enable or disable
    enum bmp5_intr_en_dis enable;

    // Push-pull or open-drain
    enum bmp5_intr_drive drive;
    
    // High or low signal level
    enum bmp5_intr_polarity polarity;
    
    // Latch or pulse mode
    enum bmp5_intr_mode mode;

    // Interrupt sources
    struct bmp5_int_source_select sources;
};

class BMP581
{
    public:
        // Constructor
        BMP581();

        // Sensor initialization, must specify communication interface
        int8_t beginI2C(uint8_t address, MF_I2C *i2c);
        int8_t beginSPI(uint8_t csPin, uint32_t clockFrequency = 100000);

        // Configuration control, the begin functions will set defaults for these
        int8_t setMode(bmp5_powermode mode);
        int8_t getMode(bmp5_powermode* mode);
        int8_t enablePress(uint8_t pressEnable);

        // Data acquisistion
        int8_t getSensorData(bmp5_sensor_data* data);

        // Output data rate (ODR) control
        int8_t setODRFrequency(uint8_t odr);
        int8_t getODRFrequency(uint8_t* odr);

        // Oversampling (OSR) control
        int8_t setOSRMultipliers(bmp5_osr_odr_press_config* config);
        int8_t getOSRMultipliers(bmp5_osr_odr_press_config* config);
        int8_t getOSREffective(bmp5_osr_odr_eff* osrOdrEffective);

        // IIR filter control
        int8_t setFilterConfig(bmp5_iir_config* iirConfig);
        int8_t setOORConfig(bmp5_oor_press_configuration* oorConfig);

        // Interrupt control
        int8_t setInterruptConfig(BMP581_InterruptConfig* config);
        int8_t getInterruptStatus(uint8_t* status);

        // FIFO control
        int8_t setFIFOConfig(bmp5_fifo* fifoConfig);
        int8_t getFIFOLength(uint8_t* numData);
        int8_t getFIFOData(bmp5_sensor_data* data, uint8_t numData);
        int8_t flushFIFO();

        // NVM control
        int8_t readNVM(uint8_t addr, uint16_t* data);
        int8_t writeNVM(uint8_t addr, uint16_t data);

    private:
        // Sensor initialization, after communication interface has been selected
        int8_t begin();

        // Read/write helper functions
        static BMP5_INTF_RET_TYPE readRegisters(uint8_t regAddress, uint8_t* dataBuffer, uint32_t numBytes, void* interfacePtr);
        static BMP5_INTF_RET_TYPE writeRegisters(uint8_t regAddress, const uint8_t* dataBuffer, uint32_t numBytes, void* interfacePtr);

        // Deley helper function
        static void usDelay(uint32_t period, void* interfacePtr);

        // Reference to the sensor
        struct bmp5_dev sensor;

        // Information about the selected communication interface (I2C or SPI)
        BMP581_InterfaceData interfaceData;

        // Place to store OSR/ODR config values
        bmp5_osr_odr_press_config osrOdrConfig;

        // Place to store FIFO config values
        bmp5_fifo fifo;
};

#endif