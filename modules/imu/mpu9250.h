#ifndef _MPU9250_INCLUDED
#define _MPU9250_INCLUDED
//-------------------------MODULES USED-------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <string.h>


//-------------------------DEFINITIONS AND MACORS---------------------------
#define MPU9250_OK 0
#define MPU9250_ILLEGAL_VALUE -1
#define MPU9250_DRIVER_INIT_FAIL -2
#define MPU9250_DRIVER_READWRITE_FAIL -3
#define MPU9250_BUFFER_SIZE -4
#define MPU9250_NULL_PTR -5


/*
 * MPU-9250
 */
/** @name MPU-9250 Registers
    Register addressise.
*/
/* @{ */
#define MPU9250_REG_SELF_TEST_X_GYRO    (0)     /*!< SELF_TEST_X_GYRO */
#define MPU9250_REG_SELF_TEST_Y_GYRO    (1)     /*!< SELF_TEST_Y_GYRO */
#define MPU9250_REG_SELF_TEST_Z_GYRO    (2)     /*!< SELF_TEST_Z_GYRO */
/* --- */
#define MPU9250_REG_SELF_TEST_X_ACCEL   (13)    /*!< SELF_TEST_X_ACCEL */
#define MPU9250_REG_SELF_TEST_Y_ACCEL   (14)    /*!< SELF_TEST_Y_ACCEL */
#define MPU9250_REG_SELF_TEST_Z_ACCEL   (15)    /*!< SELF_TEST_Z_ACCEL */
/* --- */
#define MPU9250_REG_XG_OFFSET_HL        (19)    /*!< XG_OFFSET_HL //2 Bytes */
#define MPU9250_REG_YG_OFFSET_HL        (21)    //2 Bytes
#define MPU9250_REG_ZG_OFFSET_HL        (23)    //2 Bytes
#define MPU9250_REG_SMPLRT_DIV          (25)
#define MPU9250_REG_CONFIG              (26)
#define MPU9250_REG_GYRO_CONFIG         (27)
#define MPU9250_REG_ACCEL_CONFIG        (28)
#define MPU9250_REG_ACCEL_CONFIG2       (29)
#define MPU9250_REG_LP_ACCEL_ODR        (30)
#define MPU9250_REG_WOM_THR             (31)
/* --- */
#define MPU9250_REG_FIFO_EN             (35)
#define MPU9250_REG_I2C_MST_CTRL        (36)
#define MPU9250_REG_I2C_SLV0_ADDR       (37)
#define MPU9250_REG_I2C_SLV0_REG        (38)
#define MPU9250_REG_I2C_SLV0_CTRL       (39)
#define MPU9250_REG_I2C_SLV1_ADDR       (40)
#define MPU9250_REG_I2C_SLV1_REG        (41)
#define MPU9250_REG_I2C_SLV1_CTRL       (42)
#define MPU9250_REG_I2C_SLV2_ADDR       (43)
#define MPU9250_REG_I2C_SLV2_REG        (44)
#define MPU9250_REG_I2C_SLV2_CTRL       (45)
#define MPU9250_REG_I2C_SLV3_ADDR       (46)
#define MPU9250_REG_I2C_SLV3_REG        (47)
#define MPU9250_REG_I2C_SLV3_CTRL       (48)
#define MPU9250_REG_I2C_SLV4_ADDR       (49)
#define MPU9250_REG_I2C_SLV4_REG        (50)
#define MPU9250_REG_I2C_SLV4_DO         (51)
#define MPU9250_REG_I2C_SLV4_CTRL       (52)
#define MPU9250_REG_I2C_SLV4_DI         (53)
#define MPU9250_REG_I2C_MST_STATUS      (54)
#define MPU9250_REG_INT_PIN_CFG         (55)
#define MPU9250_REG_INT_ENABLE          (56)
/* --- */
#define MPU9250_REG_INT_STATUS          (58)
#define MPU9250_REG_ACCEL_XOUT_HL       (59)    //2 Bytes
#define MPU9250_REG_ACCEL_YOUT_HL       (61)    //2 Bytes
#define MPU9250_REG_ACCEL_ZOUT_HL       (63)    //2 Bytes
#define MPU9250_REG_TEMP_HL             (65)    //2 Bytes
#define MPU9250_REG_GYRO_XOUT_HL        (67)    //2 Bytes
#define MPU9250_REG_GYRO_YOUT_HL        (69)    //2 Bytes
#define MPU9250_REG_GYRO_ZOUT_HL        (71)    //2 Bytes
#define MPU9250_REG_EXT_SENS_DATA_00    (73)    //Max 24 Bytes
/* --- */
#define MPU9250_REG_I2C_SLV0_DO         (99)
#define MPU9250_REG_I2C_SLV1_DO         (100)
#define MPU9250_REG_I2C_SLV2_DO         (101)
#define MPU9250_REG_I2C_SLV3_DO         (102)
#define MPU9250_REG_I2C_MST_DELAY_CTRL  (103)
#define MPU9250_REG_SIGNAL_PATH_RESET   (104)
#define MPU9250_REG_MOT_DETECT_CTRL     (105)
#define MPU9250_REG_USER_CTRL           (106)
#define MPU9250_REG_PWR_MGMT_1          (107)
#define MPU9250_REG_PWR_MGMT_2          (108)
/* --- */
#define MPU9250_REG_FIFO_COUNT_HL       (114)   //2 Bytes
#define MPU9250_REG_FIFO_R_W            (116)
#define MPU9250_REG_WHO_AM_I            (117)
/* --- */
#define MPU9250_REG_XA_OFFSET_HL        (119)   //2 Bytes
/* --- */
#define MPU9250_REG_YA_OFFSET_HL        (122)   //2 Bytes
/* --- */
#define MPU9250_REG_ZA_OFFSET_HL        (125)   //2 Bytes
/* @} */

/**
 * AK8963 (Included MPU-9250 package.)
 */
#define AK8963_I2C_ADDR                 (0x0c)
/* Registers. */
#define AK8963_REG_WIA                  (0x00)
#define AK8963_REG_INFO                 (0x01)
#define AK8963_REG_ST1                  (0x02)
#define AK8963_REG_HX_LH                (0x03)  //2 Bytes
#define AK8963_REG_HY_LH                (0x05)  //2 Bytes
#define AK8963_REG_HZ_LH                (0x07)  //2 Bytes
#define AK8963_REG_ST2                  (0x09)
#define AK8963_REG_CNTL1                (0x0A)
#define AK8963_REG_CNTL2                (0x0B)
#define AK8963_REG_ASTC                 (0x0C)
/* --- */
#define AK8963_REG_ASAX                 (0x10)
#define AK8963_REG_ASAY                 (0x11)
#define AK8963_REG_ASAZ                 (0x12)
//-------------------------TYPEDEFS AND STRUCTURES--------------------------
typedef struct {
    float x;        /*!< Computed X-axis value (digree/s) */
    float y;        /*!< Computed Y-axis value (digree/s) */
    float z;        /*!< Computed Z-axis value (digeee/s) */
    uint16_t raw_x; /*!< Raw X-axis value */
    uint16_t raw_y; /*!< Raw Y-axis value */
    uint16_t raw_z; /*!< Raw Z-axis value */
} MPU9250_gyro_val;

typedef struct {
    float x;        /*!< Computed X-axis value (G) */
    float y;        /*!< Computed Y-axis value (G) */
    float z;        /*!< Computed Z-axis value (G) */
    uint16_t raw_x; /*!< Raw X-axis value */
    uint16_t raw_y; /*!< Raw Y-axis value */
    uint16_t raw_z; /*!< Raw Z-axis value */
} MPU9250_accel_val;

typedef struct {
    float x;        /*!< Computed X-axis value (uH) */
    float y;        /*!< Computed Y-axis value (uH) */
    float z;        /*!< Computed Z-axis value (uH) */
    uint16_t raw_x; /*!< Raw X-axis value */
    uint16_t raw_y; /*!< Raw Y-axis value */
    uint16_t raw_z; /*!< Raw Z-axis value */
} MPU9250_magnetometer_val;

typedef enum {
    MPU9250_BIT_ACCEL_FS_SEL_2G = 0x00,
    MPU9250_BIT_ACCEL_FS_SEL_4G = 0x08,
    MPU9250_BIT_ACCEL_FS_SEL_8G = 0x10,
    MPU9250_BIT_ACCEL_FS_SEL_16G = 0x18,
    MPU9250_BIT_ACCEL_FS_SEL_MASK = 0x18
}   MPU9250_BIT_ACCEL_FS_SEL;

typedef enum {
    MPU9250_BIT_GYRO_FS_SEL_250DPS = 0x00,
    MPU9250_BIT_GYRO_FS_SEL_500DPS = 0x08,
    MPU9250_BIT_GYRO_FS_SEL_1000DPS = 0x10,
    MPU9250_BIT_GYRO_FS_SEL_2000DPS = 0x18,
    MPU9250_BIT_GYRO_FS_SEL_MASK = 0x18
}   MPU9250_BIT_GYRO_FS_SEL;

typedef enum {
    MPU9250_BIT_DLPF_CFG_250HZ = 0x00,
    MPU9250_BIT_DLPF_CFG_184HZ = 0x01,
    MPU9250_BIT_DLPF_CFG_92HZ = 0x02,
    MPU9250_BIT_DLPF_CFG_41HZ = 0x03,
    MPU9250_BIT_DLPF_CFG_20HZ = 0x04,
    MPU9250_BIT_DLPF_CFG_10HZ = 0x05,
    MPU9250_BIT_DLPF_CFG_5HZ = 0x06,
    MPU9250_BIT_DLPF_CFG_3600HZ = 0x07,
    MPU9250_BIT_DLPF_CFG_MASK = 0x07
}   MPU9250_BIT_DLPF_CFG;

typedef enum {
    MPU9250_BIT_A_DLPFCFG_460HZ = 0x00,
    MPU9250_BIT_A_DLPFCFG_184HZ = 0x01,
    MPU9250_BIT_A_DLPFCFG_92HZ = 0x02,
    MPU9250_BIT_A_DLPFCFG_41HZ = 0x03,
    MPU9250_BIT_A_DLPFCFG_20HZ = 0x04,
    MPU9250_BIT_A_DLPFCFG_10HZ = 0x05,
    MPU9250_BIT_A_DLPFCFG_5HZ = 0x06,
    MPU9250_BIT_A_DLPFCFG_460HZ_2 = 0x07,
    MPU9250_BIT_A_DLPFCFG_MASK = 0x07
}   MPU9250_BIT_A_DLPFCFG;

//-------------------------EXPORTED VARIABLES ------------------------------
#ifndef _MPU9250_C_SRC



#endif



//-------------------------EXPORTED FUNCTIONS-------------------------------
int32_t mpu9250_init(void);
int32_t mpu9250_drv_write(uint8_t reg_addr, uint8_t const * const p_write_buf, size_t len);
int32_t mpu9250_drv_write_blocking(uint8_t reg_addr, uint8_t const * const p_write_buf, size_t len);
int32_t mpu9250_drv_read(uint8_t reg_addr, size_t len);
int32_t mpu9250_drv_read_blocking(uint8_t reg_addr, uint8_t * p_buf, size_t len);
int32_t mpu9250_start_measure(MPU9250_BIT_GYRO_FS_SEL gyro_fs, MPU9250_BIT_ACCEL_FS_SEL accel_fs, MPU9250_BIT_DLPF_CFG dlpf_cfg, MPU9250_BIT_A_DLPFCFG a_dlpfcfg);
int32_t mpu9250_drv_stop_measuring(void);
int32_t mpu9250_drv_read_accel(MPU9250_accel_val *accel_val);
int32_t mpu9250_drv_read_gyro(MPU9250_gyro_val * gyro_val);
int32_t mpu9250_drv_read_magnetometer(MPU9250_magnetometer_val *magnetometer_val);
int32_t mpu9250_stream_data(void);
#endif
