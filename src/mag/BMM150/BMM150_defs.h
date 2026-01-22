/*!
 * @file bmm150_defs.h
 * @brief Define the infrastructure of the DFRobot_BMM150 class and the implementation of the underlying methods
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author      [ZhixinLiu](zhixin.liu@dfrobot.com)
 * @version     V1.0.0
 * @date        2020-04-21
 * @url         https://github.com/DFRobot/DFRobot_BMM150
 */
#ifndef _BMM150_DEFS_H
#define _BMM150_DEFS_H
#include <stdint.h>
#include <stddef.h>
#include <Arduino.h>

/******************************************************************************/
/*! @name       Common macros                         */
/******************************************************************************/
#ifdef __KERNEL__
#if (LONG_MAX) > 0x7fffffff
#define __have_long64  1
#elif (LONG_MAX) == 0x7fffffff
#define __have_long32  1
#endif
#endif

#if !defined(UINT8_C)
#define INT8_C(x)      x
#if (INT_MAX) > 0x7f
#define UINT8_C(x)     x
#else
#define UINT8_C(x)     x
#endif
#endif

#if !defined(UINT16_C)
#define INT16_C(x)     x
#if (INT_MAX) > 0x7fff
#define UINT16_C(x)    x
#else
#define UINT16_C(x)    x##U
#endif
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#if __have_long32
#define INT32_C(x)     x##L
#define UINT32_C(x)    x##UL
#else
#define INT32_C(x)     x
#define UINT32_C(x)    x##U
#endif
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#if __have_long64
#define INT64_C(x)     x##L
#define UINT64_C(x)    x##UL
#else
#define INT64_C(x)     x##LL
#define UINT64_C(x)    x##ULL
#endif
#endif

#ifndef NULL
#ifdef __cplusplus
#define NULL           0
#else
#define NULL           ((void *) 0)
#endif
#endif

/**
 * @name Compiler switch macros Definitions
 */
#ifndef BMM150_USE_FLOATING_POINT //< Check if floating point (using BMM150_USE_FLOATING_POINT) is enabled 
#ifndef BMM150_USE_FIXED_POINT //< If floating point is not enabled then enable BMM150_USE_FIXED_POINT */
#define BMM150_USE_FIXED_POINT
#endif
#endif

/**
 * @name General Macro Definitions
 */
#define BMM150_OK                                 INT8_C(0)
#define MODE_SETTING_SEL                UINT16_C(0x000F)
#define INTERRUPT_PIN_SETTING_SEL       UINT16_C(0x01F0) //< Interrupt pin settings like polarity,latch and int_pin enable 
#define INTERRUPT_CONFIG_SEL            UINT16_C(0x1E00) //< Settings to enable/disable interrupts
#define INTERRUPT_THRESHOLD_CONFIG_SEL  UINT16_C(0x6000) //< Interrupt settings for configuring threshold values

/**
 * @name To define TRUE or FALSE 
 */
#define BMM150_TRUE                               UINT8_C(1)
#define BMM150_FALSE                              UINT8_C(0)

/**
 * @name API error codes
 */
#define BMM150_E_NULL_PTR                         INT8_C(-1)
#define BMM150_E_DEV_NOT_FOUND                    INT8_C(-2)
#define BMM150_E_INVALID_CONFIG                   INT8_C(-3)
#define BMM150_E_COM_FAIL                         INT8_C(-4)

/**
 * @name API warning codes
 */
#define BMM150_W_NORMAL_SELF_TEST_YZ_FAIL         INT8_C(1)
#define BMM150_W_NORMAL_SELF_TEST_XZ_FAIL         INT8_C(2)
#define BMM150_W_NORMAL_SELF_TEST_Z_FAIL          INT8_C(3)
#define BMM150_W_NORMAL_SELF_TEST_XY_FAIL         INT8_C(4)
#define BMM150_W_NORMAL_SELF_TEST_Y_FAIL          INT8_C(5)
#define BMM150_W_NORMAL_SELF_TEST_X_FAIL          INT8_C(6)

#define SELF_TEST_XYZ_FAIL                        INT8_C(7)
#define SELF_TEST_XYZ_SUCCESS                     INT8_C(0)

/**
 * @name CHIP ID & SOFT RESET VALUES 
 */
#define BMM150_CHIP_ID                            UINT8_C(0x32)
#define BMM150_SET_SOFT_RESET                     UINT8_C(0x82)

/**
 * @name POWER MODE DEFINTIONS
 */
#define BMM150_POWERMODE_NORMAL                   UINT8_C(0x00)
#define BMM150_POWERMODE_FORCED                   UINT8_C(0x01)
#define BMM150_POWERMODE_SLEEP                    UINT8_C(0x03)
#define BMM150_POWERMODE_SUSPEND                  UINT8_C(0x04)

/**
 * @name Power mode settings
 */
#define BMM150_POWER_CNTRL_DISABLE                UINT8_C(0x00)
#define BMM150_POWER_CNTRL_ENABLE                 UINT8_C(0x01)

/**
 * @name I2C ADDRESS
 */
#define BMM150_DEFAULT_I2C_ADDRESS                UINT8_C(0x10)
#define BMM150_I2C_ADDRESS_CSB_LOW_SDO_HIGH       UINT8_C(0x11)
#define BMM150_I2C_ADDRESS_CSB_HIGH_SDO_LOW       UINT8_C(0x12)
#define BMM150_I2C_ADDRESS_CSB_HIGH_SDO_HIGH      UINT8_C(0x13)

/**
 * @name Sensor delay time settings
 */
#define BMM150_DELAY_SOFT_RESET                   UINT8_C(1000)
#define BMM150_DELAY_NORMAL_SELF_TEST             UINT8_C(2000)
#define BMM150_START_UP_TIME                      UINT8_C(3000)
#define BMM150_DELAY_ADV_SELF_TEST                UINT8_C(4000)

/**
 * @name ENABLE/DISABLE DEFINITIONS
 */
#define BMM150_XYZ_CHANNEL_ENABLE                 UINT8_C(0x00)
#define BMM150_XYZ_CHANNEL_DISABLE                UINT8_C(0x07)

/**
 * @name Register Address
 */
#define BMM150_REG_CHIP_ID                        UINT8_C(0x40)
#define BMM150_REG_DATA_X_LSB                     UINT8_C(0x42)
#define BMM150_REG_DATA_READY_STATUS              UINT8_C(0x48)
#define BMM150_REG_INTERRUPT_STATUS               UINT8_C(0x4A)
#define BMM150_REG_POWER_CONTROL                  UINT8_C(0x4B)
#define BMM150_REG_OP_MODE                        UINT8_C(0x4C)
#define BMM150_REG_INT_CONFIG                     UINT8_C(0x4D)
#define BMM150_REG_AXES_ENABLE                    UINT8_C(0x4E)
#define BMM150_REG_LOW_THRESHOLD                  UINT8_C(0x4F)
#define BMM150_REG_HIGH_THRESHOLD                 UINT8_C(0x50)
#define BMM150_REG_REP_XY                         UINT8_C(0x51)
#define BMM150_REG_REP_Z                          UINT8_C(0x52)

/**
 * @name Macros to select the sensor settings to be set by the user These values are internal for API implementation. Don't relate this to data sheet.
 */
#define BMM150_SEL_DATA_RATE                      UINT16_C(1)
#define BMM150_SEL_CONTROL_MEASURE                UINT16_C(1 << 1)
#define BMM150_SEL_XY_REP                         UINT16_C(1 << 2)
#define BMM150_SEL_Z_REP                          UINT16_C(1 << 3)
#define BMM150_SEL_DRDY_PIN_EN                    UINT16_C(1 << 4)
#define BMM150_SEL_INT_PIN_EN                     UINT16_C(1 << 5)
#define BMM150_SEL_DRDY_POLARITY                  UINT16_C(1 << 6)
#define BMM150_SEL_INT_LATCH                      UINT16_C(1 << 7)
#define BMM150_SEL_INT_POLARITY                   UINT16_C(1 << 8)
#define BMM150_SEL_DATA_OVERRUN_INT               UINT16_C(1 << 9)
#define BMM150_SEL_OVERFLOW_INT                   UINT16_C(1 << 10)
#define BMM150_SEL_HIGH_THRESHOLD_INT             UINT16_C(1 << 11)
#define BMM150_SEL_LOW_THRESHOLD_INT              UINT16_C(1 << 12)
#define BMM150_SEL_LOW_THRESHOLD_SETTING          UINT16_C(1 << 13)
#define BMM150_SEL_HIGH_THRESHOLD_SETTING         UINT16_C(1 << 14)

/**
 * @name  DATA RATE DEFINITIONS
 */
#define BMM150_DATA_RATE_10HZ                     UINT8_C(0x00)
#define BMM150_DATA_RATE_02HZ                     UINT8_C(0x01)
#define BMM150_DATA_RATE_06HZ                     UINT8_C(0x02)
#define BMM150_DATA_RATE_08HZ                     UINT8_C(0x03)
#define BMM150_DATA_RATE_15HZ                     UINT8_C(0x04)
#define BMM150_DATA_RATE_20HZ                     UINT8_C(0x05)
#define BMM150_DATA_RATE_25HZ                     UINT8_C(0x06)
#define BMM150_DATA_RATE_30HZ                     UINT8_C(0x07)
#define BMM150_ODR_MAX                            UINT8_C(0x07)
#define BMM150_ODR_MSK                            UINT8_C(0x38)
#define BMM150_ODR_POS                            UINT8_C(0x03)

/**
 * @name  TRIM REGISTERS Trim Extended Registers
 */
#define BMM150_DIG_X1                             UINT8_C(0x5D)
#define BMM150_DIG_Y1                             UINT8_C(0x5E)
#define BMM150_DIG_Z4_LSB                         UINT8_C(0x62)
#define BMM150_DIG_Z4_MSB                         UINT8_C(0x63)
#define BMM150_DIG_X2                             UINT8_C(0x64)
#define BMM150_DIG_Y2                             UINT8_C(0x65)
#define BMM150_DIG_Z2_LSB                         UINT8_C(0x68)
#define BMM150_DIG_Z2_MSB                         UINT8_C(0x69)
#define BMM150_DIG_Z1_LSB                         UINT8_C(0x6A)
#define BMM150_DIG_Z1_MSB                         UINT8_C(0x6B)
#define BMM150_DIG_XYZ1_LSB                       UINT8_C(0x6C)
#define BMM150_DIG_XYZ1_MSB                       UINT8_C(0x6D)
#define BMM150_DIG_Z3_LSB                         UINT8_C(0x6E)
#define BMM150_DIG_Z3_MSB                         UINT8_C(0x6F)
#define BMM150_DIG_XY2                            UINT8_C(0x70)
#define BMM150_DIG_XY1                            UINT8_C(0x71)

/**
 * @name  Threshold interrupt setting macros for x,y,z axes selection
 */
#define BMM150_THRESHOLD_X                        UINT8_C(0x06)
#define BMM150_THRESHOLD_Y                        UINT8_C(0x05)
#define BMM150_THRESHOLD_Z                        UINT8_C(0x03)

#define BMM150_HIGH_THRESHOLD_INT_MSK             UINT8_C(0x38)
#define BMM150_HIGH_THRESHOLD_INT_POS             UINT8_C(0x03)
#define BMM150_LOW_THRESHOLD_INT_MSK              UINT8_C(0x07)

/**
 * @name  User configurable interrupt setting macros
 */
#define BMM150_INT_ENABLE                         UINT8_C(0x01)
#define BMM150_INT_DISABLE                        UINT8_C(0x00)
#define BMM150_ACTIVE_HIGH_POLARITY               UINT8_C(0x01)
#define BMM150_ACTIVE_LOW_POLARITY                UINT8_C(0x00)
#define BMM150_LATCHED                            UINT8_C(0x01)
#define BMM150_NON_LATCHED                        UINT8_C(0x00)

/**
 * @name  Interrupt status
 */
#define BMM150_INT_THRESHOLD_X_LOW                UINT16_C(0x0001)
#define BMM150_INT_THRESHOLD_Y_LOW                UINT16_C(0x0002)
#define BMM150_INT_THRESHOLD_Z_LOW                UINT16_C(0x0004)
#define BMM150_INT_THRESHOLD_X_HIGH               UINT16_C(0x0008)
#define BMM150_INT_THRESHOLD_Y_HIGH               UINT16_C(0x0010)
#define BMM150_INT_THRESHOLD_Z_HIGH               UINT16_C(0x0020)
#define BMM150_INT_DATA_OVERFLOW                  UINT16_C(0x0040)
#define BMM150_INT_DATA_OVERRUN                   UINT16_C(0x0080)
#define BMM150_INT_DATA_READY                     UINT16_C(0x0100)

#define BMM150_DRDY_EN_MSK                        UINT8_C(0x80)
#define BMM150_DRDY_EN_POS                        UINT8_C(0x07)
#define BMM150_DRDY_POLARITY_MSK                  UINT8_C(0x04)
#define BMM150_DRDY_POLARITY_POS                  UINT8_C(0x02)
#define BMM150_INT_PIN_EN_MSK                     UINT8_C(0x40)
#define BMM150_INT_PIN_EN_POS                     UINT8_C(0x06)
#define BMM150_INT_LATCH_MSK                      UINT8_C(0x02)
#define BMM150_INT_LATCH_POS                      UINT8_C(0x01)
#define BMM150_INT_POLARITY_MSK                   UINT8_C(0x01)
#define BMM150_DRDY_STATUS_MSK                    UINT8_C(0x01)

/**
 * @name  Interrupt status macros
 */
#define BMM150_INT_ASSERTED_DRDY                  UINT16_C(0x0100)
#define BMM150_INT_ASSERTED_LOW_THRES             UINT16_C(0x0007)
#define BMM150_INT_ASSERTED_HIGH_THRES            UINT16_C(0x0380)

/**
 * @name  Power control bit macros
 */
#define BMM150_PWR_CNTRL_MSK                      UINT8_C(0x01)
#define BMM150_CONTROL_MEASURE_MSK                UINT8_C(0x38)
#define BMM150_CONTROL_MEASURE_POS                UINT8_C(0x03)
#define BMM150_POWER_CONTROL_BIT_MSK              UINT8_C(0x01)
#define BMM150_POWER_CONTROL_BIT_POS              UINT8_C(0x00)

/**
 * @name  Data macros
 */
#define BMM150_DATA_X_MSK                         UINT8_C(0xF8)
#define BMM150_DATA_X_POS                         UINT8_C(0x03)

#define BMM150_DATA_Y_MSK                         UINT8_C(0xF8)
#define BMM150_DATA_Y_POS                         UINT8_C(0x03)

#define BMM150_DATA_Z_MSK                         UINT8_C(0xFE)
#define BMM150_DATA_Z_POS                         UINT8_C(0x01)

#define BMM150_DATA_RHALL_MSK                     UINT8_C(0xFC)
#define BMM150_DATA_RHALL_POS                     UINT8_C(0x02)

#define BMM150_DATA_OVERRUN_INT_MSK               UINT8_C(0x80)
#define BMM150_DATA_OVERRUN_INT_POS               UINT8_C(0x07)

#define BMM150_OVERFLOW_INT_MSK                   UINT8_C(0x40)
#define BMM150_OVERFLOW_INT_POS                   UINT8_C(0x06)

/**
 * @name  OVERFLOW DEFINITIONS
 */
#define BMM150_OVERFLOW_ADCVAL_XYAXES_FLIP        INT16_C(-4096)
#define BMM150_OVERFLOW_ADCVAL_ZAXIS_HALL         INT16_C(-16384)
#define BMM150_OVERFLOW_OUTPUT                    INT16_C(-32768)
#define BMM150_NEGATIVE_SATURATION_Z              INT16_C(-32767)
#define BMM150_POSITIVE_SATURATION_Z              INT16_C(32767)
#ifdef BMM150_USE_FLOATING_POINT
#define BMM150_OVERFLOW_OUTPUT_FLOAT              0.0f
#endif

/**
 * @name  PRESET MODE DEFINITIONS 
 */
#define BMM150_PRESETMODE_LOWPOWER                UINT8_C(0x01)
#define BMM150_PRESETMODE_REGULAR                 UINT8_C(0x02)
#define BMM150_PRESETMODE_HIGHACCURACY            UINT8_C(0x03)
#define BMM150_PRESETMODE_ENHANCED                UINT8_C(0x04)

#define BMM150_OP_MODE_MSK                        UINT8_C(0x06)
#define BMM150_OP_MODE_POS                        UINT8_C(0x01)

/**
 * @name  PRESET MODES - REPETITIONS-XY RATES
 */
#define BMM150_REPXY_LOWPOWER                     UINT8_C(0x01)
#define BMM150_REPXY_REGULAR                      UINT8_C(0x04)
#define BMM150_REPXY_ENHANCED                     UINT8_C(0x07)
#define BMM150_REPXY_HIGHACCURACY                 UINT8_C(0x17)

/**
 * @name  PRESET MODES - REPETITIONS-Z RATES
 */
#define BMM150_REPZ_LOWPOWER                      UINT8_C(0x01)
#define BMM150_REPZ_REGULAR                       UINT8_C(0x07)
#define BMM150_REPZ_ENHANCED                      UINT8_C(0x0D)
#define BMM150_REPZ_HIGHACCURACY                  UINT8_C(0x29)

/**
 * @name  Self test settings
 */
#define BMM150_DISABLE_XY_AXIS                    UINT8_C(0x03)
#define BMM150_SELF_TEST_REP_Z                    UINT8_C(0x04)

/**
 * @name  Self test selection macros
 */
#define BMM150_SELF_TEST_NORMAL                   UINT8_C(0)
#define BMM150_SELF_TEST_ADVANCED                 UINT8_C(1)

/**
 * @name  Advanced self-test current settings
 */
#define BMM150_DISABLE_SELF_TEST_CURRENT          UINT8_C(0x00)
#define BMM150_ENABLE_NEGATIVE_CURRENT            UINT8_C(0x02)
#define BMM150_ENABLE_POSITIVE_CURRENT            UINT8_C(0x03)

/**
 * @name  Normal self-test status
 */
#define BMM150_SELF_TEST_STATUS_XYZ_FAIL          UINT8_C(0x00)
#define BMM150_SELF_TEST_STATUS_SUCCESS           UINT8_C(0x07)

#define BMM150_SELF_TEST_MSK                      UINT8_C(0x01)
#define BMM150_ADV_SELF_TEST_MSK                  UINT8_C(0xC0)
#define BMM150_ADV_SELF_TEST_POS                  UINT8_C(0x06)

/**
 * @name  Register read lengths
 */
#define BMM150_LEN_SELF_TEST                      UINT8_C(5)
#define BMM150_LEN_SETTING_DATA                   UINT8_C(8)
#define BMM150_LEN_XYZR_DATA                      UINT8_C(8)

/**
 * @name  Boundary check macros
 */
#define BMM150_BOUNDARY_MAXIMUM                   UINT8_C(0)
#define BMM150_BOUNDARY_MINIMUM                   UINT8_C(1)

#define POLARITY_HIGH                    1
#define POLARITY_LOW                     0

#define DRDY_ENABLE                      1
#define DRDY_DISABLE                     0
#define INTERRUPUT_LATCH_ENABLE          1
#define INTERRUPUT_LATCH_DISABLE         0
#define MEASUREMENT_X_ENABLE             0
#define MEASUREMENT_Y_ENABLE             0
#define MEASUREMENT_Z_ENABLE             0
#define MEASUREMENT_X_DISABLE            1
#define MEASUREMENT_Y_DISABLE            1
#define MEASUREMENT_Z_DISABLE            1
#define DATA_OVERRUN_ENABLE              1
#define DATA_OVERRUN_DISABLE             0
#define OVERFLOW_INT_ENABLE              1
#define OVERFLOW_INT_DISABLE             0
#define INTERRUPT_X_ENABLE               0
#define INTERRUPT_Y_ENABLE               0
#define INTERRUPT_Z_ENABLE               0
#define INTERRUPT_X_DISABLE              1
#define INTERRUPT_Y_DISABLE              1
#define INTERRUPT_Z_DISABLE              1
#define CHANNEL_X                        1
#define CHANNEL_Y                        2
#define CHANNEL_Z                        3
#define ENABLE_INTERRUPT_PIN             1
#define DISABLE_INTERRUPT_PIN            0
#define I2C_ADDRESS_1                    0x10
#define I2C_ADDRESS_2                    0x11
#define I2C_ADDRESS_3                    0x12
#define I2C_ADDRESS_4                    0x13

#define LOW_THRESHOLD_INTERRUPT          0
#define HIGH_THRESHOLD_INTERRUPT         1

#define NO_DATA                          -32768

/**
 * @name  Macro to SET and GET BITS of a register
 */
#define BMM150_SET_BITS(reg_data, bitname, data)        ((reg_data & ~(bitname##_MSK)) | ((data << bitname##_POS) & bitname##_MSK))
#define BMM150_GET_BITS(reg_data, bitname)              ((reg_data & (bitname##_MSK)) >> (bitname##_POS))

#define BMM150_SET_BITS_POS_0(reg_data, bitname, data)  ((reg_data & ~(bitname##_MSK)) | (data & bitname##_MSK))

#define BMM150_GET_BITS_POS_0(reg_data, bitname)        (reg_data & (bitname##_MSK))


#ifndef BMM150_INTF_RET_TYPE
#define BMM150_INTF_RET_TYPE     int8_t
#endif

#ifndef BMM150_INTF_RET_SUCCESS
#define BMM150_INTF_RET_SUCCESS  INT8_C(0)
#endif


/**
 * @brief bmm150 trim data structure
 */
typedef struct{
   int8_t digX1;    /**< trim x1 data */
   int8_t digY1;    /**< trim y1 data */
   int8_t digX2;    /**< trim x2 data */
   int8_t digY2;    /**< trim y2 data */
   uint16_t digZ1;  /**< trim z1 data */
   int16_t digZ2;   /**< trim z2 data */
   int16_t digZ3;   /**< trim z3 data */
   int16_t digZ4;   /**< trim z4 data */
   uint8_t digXY1;  /**< trim xy1 data */
   int8_t digXY2;   /**< trim xy2 data */
   uint16_t digXYZ1;/**< trim xyz1 data */
}sBmm150Trim_t;

/**
 * @brief bmm150 un-compensated (raw) magnetometer data
 */
typedef struct{
  int16_t rawDataX;/**< Raw mag X data */
  int16_t rawDataY;/**< Raw mag Y data */
  int16_t rawDataZ;/**< Raw mag Z data */
  uint16_t rawDataR;/**< Raw mag resistance value */
}sBmm150RawMagData_t;

/**
 * @brief bmm150 compensated magnetometer data in int16_t format
 */
typedef struct{
  int16_t x; /**< compensated mag X data */
  int16_t y; /**< compensated mag Y data */
  int16_t z; /**< compensated mag Z data */
  float xx;
  float yy;
  float zz;
}sBmm150MagData_t;

typedef struct{
  int16_t x;     /**< mag X data */
  int16_t y;     /**< mag Y data */
  int16_t z;     /**< mag Z data */
  uint8_t value; /**< Binary threshold state*/
  String state;  /**< String threshold state*/
}sBmm150ThresholdData_t;
#endif