/*==========================================================================================
MIT License

Copyright (c) 2026 https://madflight.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
===========================================================================================*/

//driver for ICM42688P, ICM42605, IMM42653
//based on https://github.com/betaflight/betaflight/blob/master/src/main/drivers/accgyro/accgyro_spi_icm426xx.c

#include <Arduino.h>

#include "ICM426XX.h"

#define ICM426XX_MAX_SPI_CLK_HZ 24000000
#define ICM426XX_MAX_I2C_CLK_HZ 400000

#define MPU_RA_WHO_AM_I         0x75

#define ICM42605_WHO_AM_I_CONST             (0x42)
#define ICM42688P_WHO_AM_I_CONST            (0x47)
#define IIM42653_WHO_AM_I_CONST             (0x56)

#define ICM_42605_SPI ICM42605_WHO_AM_I_CONST
#define ICM_42688P_SPI ICM42688P_WHO_AM_I_CONST
#define IIM_42653_SPI IIM42653_WHO_AM_I_CONST

#define ICM426XX_CLKIN_FREQ                         32000

#define ICM426XX_RA_REG_BANK_SEL                    0x76
#define ICM426XX_BANK_SELECT0                       0x00
#define ICM426XX_BANK_SELECT1                       0x01
#define ICM426XX_BANK_SELECT2                       0x02
#define ICM426XX_BANK_SELECT3                       0x03
#define ICM426XX_BANK_SELECT4                       0x04

// Fix for stalls in gyro output. See https://github.com/ArduPilot/ardupilot/pull/25332
#define ICM426XX_INTF_CONFIG1                       0x4D
#define ICM426XX_INTF_CONFIG1_AFSR_MASK             0xC0
#define ICM426XX_INTF_CONFIG1_AFSR_DISABLE          0x40

#define ICM426XX_RA_PWR_MGMT0                       0x4E  // User Bank 0
#define ICM426XX_PWR_MGMT0_ACCEL_MODE_LN            (3 << 0)
#define ICM426XX_PWR_MGMT0_GYRO_MODE_LN             (3 << 2)
#define ICM426XX_PWR_MGMT0_GYRO_ACCEL_MODE_OFF      ((0 << 0) | (0 << 2))
#define ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF         (0 << 5)
#define ICM426XX_PWR_MGMT0_TEMP_DISABLE_ON          (1 << 5)

#define ICM426XX_RA_GYRO_CONFIG0                    0x4F
#define ICM426XX_RA_ACCEL_CONFIG0                   0x50

// --- Registers for gyro and acc Anti-Alias Filter ---------
#define ICM426XX_RA_GYRO_CONFIG_STATIC3             0x0C  // User Bank 1
#define ICM426XX_RA_GYRO_CONFIG_STATIC4             0x0D  // User Bank 1
#define ICM426XX_RA_GYRO_CONFIG_STATIC5             0x0E  // User Bank 1
#define ICM426XX_RA_ACCEL_CONFIG_STATIC2            0x03  // User Bank 2
#define ICM426XX_RA_ACCEL_CONFIG_STATIC3            0x04  // User Bank 2
#define ICM426XX_RA_ACCEL_CONFIG_STATIC4            0x05  // User Bank 2
// --- Register & setting for gyro and acc UI Filter --------
#define ICM426XX_RA_GYRO_ACCEL_CONFIG0              0x52  // User Bank 0
#define ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY       (15 << 4)
#define ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY        (15 << 0)
// ----------------------------------------------------------

#define ICM426XX_RA_GYRO_DATA_X1                    0x25  // User Bank 0
#define ICM426XX_RA_ACCEL_DATA_X1                   0x1F  // User Bank 0

#define ICM426XX_RA_INT_CONFIG                      0x14  // User Bank 0
#define ICM426XX_INT1_MODE_PULSED                   (0 << 2)
#define ICM426XX_INT1_MODE_LATCHED                  (1 << 2)
#define ICM426XX_INT1_DRIVE_CIRCUIT_OD              (0 << 1)
#define ICM426XX_INT1_DRIVE_CIRCUIT_PP              (1 << 1)
#define ICM426XX_INT1_POLARITY_ACTIVE_LOW           (0 << 0)
#define ICM426XX_INT1_POLARITY_ACTIVE_HIGH          (1 << 0)

#define ICM426XX_RA_INT_CONFIG0                     0x63  // User Bank 0
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR           ((0 << 5) | (0 << 4))
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR_DUPLICATE ((0 << 5) | (0 << 4)) // duplicate settings in datasheet, Rev 1.2.
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_F1BR          ((1 << 5) | (0 << 4))
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR_AND_F1BR  ((1 << 5) | (1 << 4))

#define ICM426XX_RA_INT_CONFIG1                     0x64   // User Bank 0
#define ICM426XX_INT_ASYNC_RESET_BIT                4
#define ICM426XX_INT_TDEASSERT_DISABLE_BIT          5
#define ICM426XX_INT_TDEASSERT_ENABLED              (0 << ICM426XX_INT_TDEASSERT_DISABLE_BIT)
#define ICM426XX_INT_TDEASSERT_DISABLED             (1 << ICM426XX_INT_TDEASSERT_DISABLE_BIT)
#define ICM426XX_INT_TPULSE_DURATION_BIT            6
#define ICM426XX_INT_TPULSE_DURATION_100            (0 << ICM426XX_INT_TPULSE_DURATION_BIT)
#define ICM426XX_INT_TPULSE_DURATION_8              (1 << ICM426XX_INT_TPULSE_DURATION_BIT)

#define ICM426XX_RA_INT_SOURCE0                     0x65  // User Bank 0
#define ICM426XX_UI_DRDY_INT1_EN_DISABLED           (0 << 3)
#define ICM426XX_UI_DRDY_INT1_EN_ENABLED            (1 << 3)

// specific to CLKIN configuration
#define ICM426XX_INTF_CONFIG5                       0x7B  // User Bank 1
#define ICM426XX_INTF_CONFIG1_CLKIN                 (1 << 2)
#define ICM426XX_INTF_CONFIG5_PIN9_FUNCTION_MASK    (3 << 1)   // PIN9 mode config
#define ICM426XX_INTF_CONFIG5_PIN9_FUNCTION_CLKIN   (2 << 1)   // PIN9 as CLKIN


#define ICM426XX_RA_INTF_CONFIG0                     0x4C  // User Bank 0
#define ICM426XX_INTF_CONFIG0_LITTLE_ENDIAN          0x00  // select little endian data mode

typedef enum {
    ODR_CONFIG_8K = 0,
    ODR_CONFIG_4K,
    ODR_CONFIG_2K,
    ODR_CONFIG_1K,
    ODR_CONFIG_COUNT
} odrConfig_e;

typedef enum {
    AAF_CONFIG_258HZ = 0,
    AAF_CONFIG_536HZ,
    AAF_CONFIG_997HZ,
    AAF_CONFIG_1962HZ,
    AAF_CONFIG_COUNT
} aafConfig_e;

typedef struct aafConfig_s {
    uint8_t delt;
    uint16_t deltSqr;
    uint8_t bitshift;
} aafConfig_t;

// Possible output data rates (ODRs)
static uint8_t odrLUT[ODR_CONFIG_COUNT] = {  // see GYRO_ODR in section 5.6
    [ODR_CONFIG_8K] = 3,
    [ODR_CONFIG_4K] = 4,
    [ODR_CONFIG_2K] = 5,
    [ODR_CONFIG_1K] = 6,
};

// Possible gyro Anti-Alias Filter (AAF) cutoffs for ICM-42688P
static aafConfig_t aafLUT42688[AAF_CONFIG_COUNT] = {  // see table in section 5.3
    [AAF_CONFIG_258HZ]  = {  6,   36, 10 },
    [AAF_CONFIG_536HZ]  = { 12,  144,  8 },
    [AAF_CONFIG_997HZ]  = { 21,  440,  6 },
    [AAF_CONFIG_1962HZ] = { 37, 1376,  4 },
};

// Possible gyro Anti-Alias Filter (AAF) cutoffs for ICM-42605
// actual cutoff differs slightly from those of the 42688P
static aafConfig_t aafLUT42605[AAF_CONFIG_COUNT] = {  // see table in section 5.3
    [AAF_CONFIG_258HZ]  = { 21,  440,  6 }, // actually 249 Hz
    [AAF_CONFIG_536HZ]  = { 39, 1536,  4 }, // actually 524 Hz
    [AAF_CONFIG_997HZ]  = { 63, 3968,  3 }, // actually 995 Hz
    [AAF_CONFIG_1962HZ] = { 63, 3968,  3 }, // 995 Hz is the max cutoff on the 42605
};

void ICM426XX::setUserBank(const uint8_t user_bank)
{
    dev->writeReg(ICM426XX_RA_REG_BANK_SEL, user_bank & 7);
}


const char* ICM426XX::type_name() {
    switch (whoAmI) {
    case ICM_42605_SPI:
      return "ICM42605";
    case ICM_42688P_SPI:
      return "ICM42688P";
    case IIM_42653_SPI:
      return "IIM42653";
   }
   return "UNKNOWN";
}

bool ICM426XX::detect(SensorDevice* dev) {
    const uint8_t wai = dev->readReg(MPU_RA_WHO_AM_I);
    return ((wai == ICM42605_WHO_AM_I_CONST ) || (wai == ICM42688P_WHO_AM_I_CONST) || (wai == IIM42653_WHO_AM_I_CONST));
}

ICM426XX* ICM426XX::create(SensorDevice *dev, int pin_clkin)
{
    dev->setLowSpeed();

    //dev->writeReg(ICM426XX_RA_PWR_MGMT0, 0x00);

    uint8_t attemptsRemaining = 20;
    do {
        const uint8_t whoAmI = dev->readReg(MPU_RA_WHO_AM_I);
        switch (whoAmI) {
        case ICM42605_WHO_AM_I_CONST:
        case ICM42688P_WHO_AM_I_CONST:
        case IIM42653_WHO_AM_I_CONST: {
          auto icm = new ICM426XX(dev, whoAmI, pin_clkin);
          return icm;
        }
        delay(150);
       }
    } while (attemptsRemaining--);

    return nullptr;
}


ICM426XX::ICM426XX(SensorDevice *dev, uint8_t whoAmI, int pin_clkin) {
    this->dev = dev;
    this->whoAmI = whoAmI;

    switch (whoAmI) {
      case ICM_42605_SPI:
      case ICM_42688P_SPI:
        acc_scale = 16.0 / 32768; // Accel scale 16g (2048 LSB/g)
        gyr_scale = 2000.0 / 32768; //2000DPS
        break;
      case IIM_42653_SPI:
        acc_scale = 32.0 / 32768; // Accel scale 32g (1024 LSB/g)
        gyr_scale = 4000.0 / 32768; //4000DPS
        break;
    }

    //dev->setFreq(ICM426XX_MAX_SPI_CLK_HZ);

    // Turn off ACC and GYRO so they can be configured
    // See section 12.9 in ICM-42688-P datasheet v1.7
    setUserBank(ICM426XX_BANK_SELECT0);
    dev->writeReg(ICM426XX_RA_PWR_MGMT0, ICM426XX_PWR_MGMT0_GYRO_ACCEL_MODE_OFF);

    //Anti-Alias Filter selection
    aafConfig_t aafConfig;
    switch (whoAmI) {
    case ICM_42605_SPI:
        aafConfig = aafLUT42605[AAF_CONFIG_258HZ];
        break;
    case ICM_42688P_SPI:
    case IIM_42653_SPI:
    default:
        aafConfig = aafLUT42688[AAF_CONFIG_258HZ];
    }

    // Configure gyro Anti-Alias Filter (see section 5.3 "ANTI-ALIAS FILTER")
    setUserBank(ICM426XX_BANK_SELECT1);
    dev->writeReg(ICM426XX_RA_GYRO_CONFIG_STATIC3, aafConfig.delt);
    dev->writeReg(ICM426XX_RA_GYRO_CONFIG_STATIC4, aafConfig.deltSqr & 0xFF);
    dev->writeReg(ICM426XX_RA_GYRO_CONFIG_STATIC5, (aafConfig.deltSqr >> 8) | (aafConfig.bitshift << 4));

    // Configure acc Anti-Alias Filter for 1kHz sample rate (see tasks.c)
    setUserBank(ICM426XX_BANK_SELECT2);
    dev->writeReg(ICM426XX_RA_ACCEL_CONFIG_STATIC2, aafConfig.delt << 1);
    dev->writeReg(ICM426XX_RA_ACCEL_CONFIG_STATIC3, aafConfig.deltSqr & 0xFF);
    dev->writeReg(ICM426XX_RA_ACCEL_CONFIG_STATIC4, (aafConfig.deltSqr >> 8) | (aafConfig.bitshift << 4));

    // Configure gyro and acc UI Filters
    setUserBank(ICM426XX_BANK_SELECT0);
    dev->writeReg(ICM426XX_RA_GYRO_ACCEL_CONFIG0, ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY | ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY);

    // Configure interrupt pin
    dev->writeReg(ICM426XX_RA_INT_CONFIG, ICM426XX_INT1_MODE_PULSED | ICM426XX_INT1_DRIVE_CIRCUIT_PP | ICM426XX_INT1_POLARITY_ACTIVE_HIGH);
    dev->writeReg(ICM426XX_RA_INT_CONFIG0, ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR);
    dev->writeReg(ICM426XX_RA_INT_SOURCE0, ICM426XX_UI_DRDY_INT1_EN_ENABLED);

    uint8_t intConfig1Value = dev->readReg(ICM426XX_RA_INT_CONFIG1);
    // Datasheet says: "User should change setting to 0 from default setting of 1, for proper INT1 and INT2 pin operation"
    intConfig1Value &= ~(1 << ICM426XX_INT_ASYNC_RESET_BIT);
    intConfig1Value |= (ICM426XX_INT_TPULSE_DURATION_8 | ICM426XX_INT_TDEASSERT_DISABLED);

    dev->writeReg(ICM426XX_RA_INT_CONFIG1, intConfig1Value);

    // Disable AFSR to prevent stalls in gyro output
    uint8_t intfConfig1Value = dev->readReg(ICM426XX_INTF_CONFIG1);
    intfConfig1Value &= ~ICM426XX_INTF_CONFIG1_AFSR_MASK;
    intfConfig1Value |= ICM426XX_INTF_CONFIG1_AFSR_DISABLE;
    dev->writeReg(ICM426XX_INTF_CONFIG1, intfConfig1Value);

    // Turn on gyro and acc on again so ODR and FSR can be configured
    dev->writeReg(ICM426XX_RA_PWR_MGMT0, ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF | ICM426XX_PWR_MGMT0_ACCEL_MODE_LN | ICM426XX_PWR_MGMT0_GYRO_MODE_LN);
    delay(1);

    // Get desired output data rate
    uint8_t odrConfig;
    odrConfig = odrLUT[ODR_CONFIG_1K];
    sampling_rate_hz = 1000;

    // This sets the gyro/accel to the maximum FSR, depending on the chip
    // ICM42605, ICM_42688P: 2000DPS and 16G.
    // IIM42653: 4000DPS and 32G
    dev->writeReg(ICM426XX_RA_GYRO_CONFIG0, (0 << 5) | (odrConfig & 0x0F));
    delay(15);
    dev->writeReg(ICM426XX_RA_ACCEL_CONFIG0, (0 << 5) | (odrConfig & 0x0F));
    delay(15);
    
    // Select little-endian data mode
    dev->writeReg(ICM426XX_RA_INTF_CONFIG0, ICM426XX_INTF_CONFIG0_LITTLE_ENDIAN);

    //enable CLKIN on ICM_42688P
    if(pin_clkin >= 0 && whoAmI == ICM_42688P_SPI) {
        //setup 32kHz output on clkin_pin
        float freq = 32000;
        clkin.begin(pin_clkin, freq, 0, 1e6 / freq); //32kHz pulse 0-31.25 us
        clkin.writeMicroseconds(1e6 / freq / 2);

        // Switch to Bank 1 and set bits 2:1 in INTF_CONFIG5 (0x7B) to enable CLKIN on PIN9
        setUserBank(ICM426XX_BANK_SELECT1);
        uint8_t intf_config5 = dev->readReg(ICM426XX_INTF_CONFIG5);
        intf_config5 = (intf_config5 & ~ICM426XX_INTF_CONFIG5_PIN9_FUNCTION_MASK) | ICM426XX_INTF_CONFIG5_PIN9_FUNCTION_CLKIN;  // Clear & set bits 2:1 to 0b10 for CLKIN
        dev->writeReg(ICM426XX_INTF_CONFIG5, intf_config5);

        // Switch to Bank 0 and set bit 2 in RTC_MODE (0x4D) to enable external CLK signal
        setUserBank(ICM426XX_BANK_SELECT0);
        uint8_t rtc_mode = dev->readReg(ICM426XX_INTF_CONFIG1);
        rtc_mode |= ICM426XX_INTF_CONFIG1_CLKIN; // Enable external CLK signal
        dev->writeReg(ICM426XX_INTF_CONFIG1, rtc_mode);
    }

    if(dev->isSPI()) {
        dev->setFreq(ICM426XX_MAX_SPI_CLK_HZ);
    }else{
        dev->setFreq(ICM426XX_MAX_I2C_CLK_HZ);
    }
}

void ICM426XX::read(int16_t *accgyr) {
  dev->readRegs(ICM426XX_RA_ACCEL_DATA_X1, (uint8_t*)accgyr, 12); //ax,ay,az,gx,gy,gz little endian, no byte juggling needed for RP2,ESP32,STM32
}

/*
#if defined(USE_GYRO_CLKIN)
static pwmOutputPort_t pwmGyroClk = {0};

static bool initExternalClock(const SensorDevice *dev)
{
    int cfg;
    if (&gyro.gyroSensor1.gyroDev.dev == dev) {
        cfg = 0;
    } else if (&gyro.gyroSensor2.gyroDev.dev == dev) {
        cfg = 1;
    } else {
        // only gyroSensor<n> device supported
        return false;
    }
    const ioTag_t tag = gyroDeviceConfig(cfg)->clkIn;
    const IO_t io = IOGetByTag(tag);
    if (pwmGyroClk.enabled) {
       // pwm is already taken, but test for shared clkIn pin
       return pwmGyroClk.io == io;
    }

    const timerHardware_t *timer = timerAllocate(tag, OWNER_GYRO_CLKIN, RESOURCE_INDEX(cfg));
    if (!timer) {
        // Error handling: failed to allocate timer
        return false;
    }

    pwmGyroClk.io = io;
    pwmGyroClk.enabled = true;

    IOInit(io, OWNER_GYRO_CLKIN, RESOURCE_INDEX(cfg));
    IOConfigGPIOAF(io, IOCFG_AF_PP, timer->alternateFunction);

    const uint32_t clock = timerClock(timer->tim);  // Get the timer clock frequency
    const uint16_t period = clock / ICM426XX_CLKIN_FREQ;

    // Calculate duty cycle value for 50%
    const uint16_t value = period / 2;

    // Configure PWM output
    pwmOutConfig(&pwmGyroClk.channel, timer, clock, period - 1, value - 1, 0);

    // Set CCR value
    *pwmGyroClk.channel.ccr = value - 1;

    return true;
}

static void icm426xxEnableExternalClock(const SensorDevice *dev)
{
    if (initExternalClock(dev)) {
        // Switch to Bank 1 and set bits 2:1 in INTF_CONFIG5 (0x7B) to enable CLKIN on PIN9
        setUserBank(ICM426XX_BANK_SELECT1);
        uint8_t intf_config5 = dev->readReg(ICM426XX_INTF_CONFIG5);
        intf_config5 = (intf_config5 & ~ICM426XX_INTF_CONFIG5_PIN9_FUNCTION_MASK) | ICM426XX_INTF_CONFIG5_PIN9_FUNCTION_CLKIN;  // Clear & set bits 2:1 to 0b10 for CLKIN
        dev->writeReg(ICM426XX_INTF_CONFIG5, intf_config5);

        // Switch to Bank 0 and set bit 2 in RTC_MODE (0x4D) to enable external CLK signal
        setUserBank(ICM426XX_BANK_SELECT0);
        uint8_t rtc_mode = dev->readReg(ICM426XX_INTF_CONFIG1);
        rtc_mode |= ICM426XX_INTF_CONFIG1_CLKIN; // Enable external CLK signal
        dev->writeReg(ICM426XX_INTF_CONFIG1, rtc_mode);
    }
}
#endif //#if defined(USE_GYRO_CLKIN)

*/
