#define _MPU9250_C_SRC

//-------------------------MODULES USED-------------------------------------
#include "nrfx_spi.h"
#include "nrf_gpio.h"


#include "mpu9250.h"


//-------------------------DEFINITIONS AND MACORS---------------------------
#define SPI_INSTANCE  0
#define BUF_LEN 128

#define NRFX_SPI_SCK_PIN  30
#define NRFX_SPI_SDO_PIN 29
#define NRFX_SPI_SDI_PIN 31
#define NRFX_SPI_SS_PIN   28




//-------------------------TYPEDEFS AND STRUCTURES--------------------------



//-------------------------PROTOTYPES OF LOCAL FUNCTIONS--------------------
static void nrfx_spi_evt_handler(nrfx_spi_evt_t const * p_event, void * p_context);


//-------------------------EXPORTED VARIABLES ------------------------------



//-------------------------GLOBAL VARIABLES---------------------------------
static const nrfx_spi_t g_spi = NRFX_SPI_INSTANCE(SPI_INSTANCE);
static uint8_t g_tx_buf[BUF_LEN];
static uint8_t g_rx_buf[BUF_LEN];
static size_t g_tx_len = 0;
static size_t g_rx_len = 0;
static bool g_xfer_in_progress = false;
static uint8_t g_magnetometer_calib[3];
static float g_gyro_div;
static float g_accel_div;

//-------------------------EXPORTED FUNCTIONS-------------------------------

int32_t mpu9250_init(void)
{
    nrfx_spi_config_t spi_config;

    spi_config.sck_pin = NRFX_SPI_SCK_PIN;
    spi_config.mosi_pin = NRFX_SPI_SDO_PIN;
    spi_config.miso_pin = NRFX_SPI_SDI_PIN;
    spi_config.ss_pin = NRFX_SPI_SS_PIN;
    spi_config.irq_priority = 4;
    spi_config.orc = 'a';
    spi_config.frequency = NRF_SPI_FREQ_1M;
    spi_config.mode = NRF_SPI_MODE_0;
    spi_config.bit_order = NRF_SPI_BIT_ORDER_MSB_FIRST;
    int32_t ret = nrfx_spi_init(&g_spi, &spi_config, nrfx_spi_evt_handler, NULL);
    if(NRFX_SUCCESS != ret){
        return MPU9250_DRIVER_INIT_FAIL;
    }

    uint8_t buf = 0;
    ret = mpu9250_drv_read_blocking(MPU9250_REG_WHO_AM_I, &buf, 1);
    if(MPU9250_OK != ret){
        return MPU9250_DRIVER_INIT_FAIL;
    }
    if(0x71 != buf){
        return MPU9250_DRIVER_INIT_FAIL;
    }
    const uint8_t accel_gyro_init_conf[][2] = {
                              {MPU9250_REG_PWR_MGMT_1, 0x80},  /* reset */
                              {MPU9250_REG_PWR_MGMT_1, 0x01},  /* clock source */
                              {MPU9250_REG_PWR_MGMT_2, 0x3f},  /* Disable Accel & Gyro */
                              {MPU9250_REG_INT_PIN_CFG, 0x30},  /* Pin config */
                              {MPU9250_REG_I2C_MST_CTRL, 0x0D},  /* I2C multi-master / 400kHz */
                              {MPU9250_REG_USER_CTRL, 0x20},  /* I2C master mode */
                              {0xde, 0xad}};
    for(int i=0;accel_gyro_init_conf[i][0] != 0xde;i++){
        mpu9250_drv_write_blocking(accel_gyro_init_conf[i][0], &accel_gyro_init_conf[i][1], 1);
    }

    const uint8_t mag_init_conf[][2] = {
                          {MPU9250_REG_I2C_SLV0_ADDR, AK8963_I2C_ADDR},  /* reset */
                          {MPU9250_REG_I2C_SLV0_REG, AK8963_REG_CNTL2},  /* clock source */
                          {MPU9250_REG_I2C_SLV0_DO, 0x01},  /* Disable Accel & Gyro */
                          {MPU9250_REG_I2C_SLV0_CTRL, 0x80},  /* Pin config */
                          {0xde, 0xad}};
    for(int i=0;mag_init_conf[i][0] != 0xde;i++){
        mpu9250_drv_write_blocking(mag_init_conf[i][0], &mag_init_conf[i][1], 1);
    }

    //read calibration data
    buf = AK8963_I2C_ADDR|0x80;
    mpu9250_drv_write_blocking(MPU9250_REG_I2C_SLV0_ADDR, &buf, 1);
    buf = AK8963_REG_ASAX;
    mpu9250_drv_write_blocking(MPU9250_REG_I2C_SLV0_REG, &buf, 1);
    buf = 0x83;
    mpu9250_drv_write_blocking(MPU9250_REG_I2C_SLV0_CTRL, &buf, 1);
    mpu9250_drv_read_blocking(MPU9250_REG_EXT_SENS_DATA_00, g_magnetometer_calib, 3);

    //16 bit periodic mode (8hz)
    buf = AK8963_I2C_ADDR;
    mpu9250_drv_write_blocking(MPU9250_REG_I2C_SLV0_ADDR, &buf, 1);
    buf = AK8963_REG_CNTL1;
    mpu9250_drv_write_blocking(MPU9250_REG_I2C_SLV0_REG, &buf, 1);
    buf = 0x12;
    mpu9250_drv_write_blocking(MPU9250_REG_I2C_SLV0_DO, &buf, 1);
    buf = 0x81;
    mpu9250_drv_write_blocking(MPU9250_REG_I2C_SLV0_CTRL, &buf, 1);

    //read whoami
    buf = AK8963_I2C_ADDR | 0x80;
    mpu9250_drv_write_blocking(MPU9250_REG_I2C_SLV0_ADDR, &buf, 1);
    buf = AK8963_REG_WIA;
    mpu9250_drv_write_blocking(MPU9250_REG_I2C_SLV0_REG,  &buf, 1);
    buf = 0x81;
    mpu9250_drv_write_blocking(MPU9250_REG_I2C_SLV0_CTRL, &buf, 1);

    ret = mpu9250_drv_read_blocking(MPU9250_REG_EXT_SENS_DATA_00, &buf, 1);
    if(MPU9250_OK != ret){
        return MPU9250_DRIVER_INIT_FAIL;
    }
    if(0x48 != buf){
        return MPU9250_DRIVER_INIT_FAIL;
    }

    return MPU9250_OK;
}

int32_t mpu9250_uninit(void)
{
    nrfx_spi_uninit(&g_spi);
    return MPU9250_OK;
}


int32_t mpu9250_start_measure(MPU9250_BIT_GYRO_FS_SEL gyro_fs, MPU9250_BIT_ACCEL_FS_SEL accel_fs, MPU9250_BIT_DLPF_CFG dlpf_cfg, MPU9250_BIT_A_DLPFCFG a_dlpfcfg)
{
    g_xfer_in_progress = false;
    const uint8_t init_conf[][2] = {
                              {MPU9250_REG_PWR_MGMT_2,    0x00},                      /* Enable Accel & Gyro */
                              {MPU9250_REG_CONFIG,        (uint8_t)dlpf_cfg},         /* Gyro LPF */
                              {MPU9250_REG_GYRO_CONFIG,   (uint8_t)gyro_fs},          /* Gyro configuration */
                              {MPU9250_REG_ACCEL_CONFIG,  (uint8_t)accel_fs},         /* Accel configuration */
                              {MPU9250_REG_ACCEL_CONFIG2, 0x08 | (uint8_t)a_dlpfcfg}, /* Accel LPF */
                              {0xde,                      0xad}
    };
    for (int i = 0; init_conf[i][0] != 0xde; i++) {
        int32_t ret = mpu9250_drv_write_blocking(init_conf[i][0], &init_conf[i][1], 1);
        if(ret != MPU9250_OK){
            return ret;
        }
    }
    switch(gyro_fs){
        case MPU9250_BIT_GYRO_FS_SEL_250DPS:{
            g_gyro_div = 131.0;
            break;
        }

        case MPU9250_BIT_GYRO_FS_SEL_500DPS:{
            g_gyro_div = 65.5;
            break;
        }

        case MPU9250_BIT_GYRO_FS_SEL_1000DPS:{
            g_gyro_div = 32.8;
            break;
        }

        case MPU9250_BIT_GYRO_FS_SEL_2000DPS:{
            g_gyro_div = 16.4;
            break;
        }

        default:{
            g_gyro_div = 131.0;
            break;
        }
    }

    switch (accel_fs) {
        case MPU9250_BIT_ACCEL_FS_SEL_2G:{
            g_accel_div = 16384;
            break;
        }

        case MPU9250_BIT_ACCEL_FS_SEL_4G:{
            g_accel_div = 8192;
            break;
        }

        case MPU9250_BIT_ACCEL_FS_SEL_8G:{
            g_accel_div = 4096;
            break;
        }

        case MPU9250_BIT_ACCEL_FS_SEL_16G:{
            g_accel_div = 2048;
            break;
        }

        default:{
            g_accel_div = 16384;
            break;
        }
    }

    return MPU9250_OK;
}

int32_t mpu9250_drv_stop_measuring(void)
{
    //disable accelerometer and gyroscope
    uint8_t buf = 0x3f;
    return mpu9250_drv_write_blocking(MPU9250_REG_PWR_MGMT_2, &buf, 1);
}

int32_t mpu9250_drv_read_gyro(void)
{

    return mpu9250_drv_read(MPU9250_REG_GYRO_XOUT_HL, 6);
}

void mpu9250_drv_process_raw_gyro(MPU9250_accel_val * p_gyro_val)
{
    p_gyro_val->raw_x = ((uint16_t)g_rx_buf[0 + 1] << 8) | g_rx_buf[1 + 1];
    p_gyro_val->raw_y = ((uint16_t)g_rx_buf[2 + 1] << 8) | g_rx_buf[3 + 1];
    p_gyro_val->raw_z = ((uint16_t)g_rx_buf[4 + 1] << 8) | g_rx_buf[5 + 1];

    p_gyro_val->x = (float)(int16_t)p_gyro_val->raw_x / g_gyro_div;
    p_gyro_val->y = (float)(int16_t)p_gyro_val->raw_y / g_gyro_div;
    p_gyro_val->z = (float)(int16_t)p_gyro_val->raw_z / g_gyro_div;
}

int32_t mpu9250_drv_read_gyro_blocking(MPU9250_gyro_val * gyro_val)
{
    if(NULL == gyro_val){
        return MPU9250_NULL_PTR;
    }

    uint8_t vals[6] = {0};
    int32_t ret = mpu9250_drv_read_blocking(MPU9250_REG_GYRO_XOUT_HL, vals, 6);
    if(ret != MPU9250_OK){
        return ret;
    }
    gyro_val->raw_x = ((uint16_t)vals[0] << 8) | vals[1];
    gyro_val->raw_y = ((uint16_t)vals[2] << 8) | vals[3];
    gyro_val->raw_z = ((uint16_t)vals[4] << 8) | vals[5];

    gyro_val->x = (float)(int16_t)gyro_val->raw_x / g_gyro_div;
    gyro_val->y = (float)(int16_t)gyro_val->raw_y / g_gyro_div;
    gyro_val->z = (float)(int16_t)gyro_val->raw_z / g_gyro_div;

    return MPU9250_OK;
}

int32_t mpu9250_drv_read_accel(void)
{

    return mpu9250_drv_read(MPU9250_REG_ACCEL_XOUT_HL, 6);
}

void mpu9250_drv_process_raw_accel(MPU9250_accel_val * p_accel_val)
{
    p_accel_val->raw_x = ((uint16_t)g_rx_buf[0 + 1] << 8) | g_rx_buf[1 + 1];
    p_accel_val->raw_y = ((uint16_t)g_rx_buf[2 + 1] << 8) | g_rx_buf[3 + 1];
    p_accel_val->raw_z = ((uint16_t)g_rx_buf[4 + 1] << 8) | g_rx_buf[5 + 1];

    p_accel_val->x = (float)(int16_t)p_accel_val->raw_x / g_accel_div;
    p_accel_val->y = (float)(int16_t)p_accel_val->raw_y / g_accel_div;
    p_accel_val->z = (float)(int16_t)p_accel_val->raw_z / g_accel_div;
}


int32_t mpu9250_drv_read_accel_blocking(MPU9250_accel_val *accel_val)
{
    if(NULL == accel_val){
        return MPU9250_NULL_PTR;
    }

    uint8_t vals[6];
    int32_t ret = mpu9250_drv_read_blocking(MPU9250_REG_ACCEL_XOUT_HL, vals, 6);
    if(ret != MPU9250_OK){
        return ret;
    }
    accel_val->raw_x = ((uint16_t)vals[0] << 8) | vals[1];
    accel_val->raw_y = ((uint16_t)vals[2] << 8) | vals[3];
    accel_val->raw_z = ((uint16_t)vals[4] << 8) | vals[5];

    accel_val->x = (float)(int16_t)accel_val->raw_x / g_accel_div;
    accel_val->y = (float)(int16_t)accel_val->raw_y / g_accel_div;
    accel_val->z = (float)(int16_t)accel_val->raw_z / g_accel_div;

    return MPU9250_OK;
}

int32_t mpu9250_drv_read_magnetometer_blocking(MPU9250_magnetometer_val *magnetometer_val)
{
    if(NULL == magnetometer_val){
        return MPU9250_NULL_PTR;
    }
    uint8_t vals[8];
    // set read flag & slave address.
    uint8_t buf = AK8963_I2C_ADDR | 0x80;
    mpu9250_drv_write_blocking(MPU9250_REG_I2C_SLV0_ADDR, &buf, 1);
    // set register address.
    buf = AK8963_REG_ST1;
    mpu9250_drv_write_blocking(MPU9250_REG_I2C_SLV0_REG, &buf, 1);
    // transfer
    buf = 0x88;
    mpu9250_drv_write_blocking(MPU9250_REG_I2C_SLV0_CTRL, &buf, 1);

    int32_t ret = mpu9250_drv_read_blocking(MPU9250_REG_EXT_SENS_DATA_00, vals, 8);
    if(ret != MPU9250_OK){
        return ret;
    }
    magnetometer_val->raw_x = ((uint16_t)vals[2] << 8) | vals[1];
    magnetometer_val->raw_y = ((uint16_t)vals[4] << 8) | vals[3];
    magnetometer_val->raw_z = ((uint16_t)vals[6] << 8) | vals[5];
    /* Real data */
    magnetometer_val->x = (int16_t)magnetometer_val->raw_x * ((((float)(int8_t)g_magnetometer_calib[0] - 128) / 256) + 1);
    magnetometer_val->y = (int16_t)magnetometer_val->raw_y * ((((float)(int8_t)g_magnetometer_calib[1] - 128) / 256) + 1);
    magnetometer_val->z = (int16_t)magnetometer_val->raw_z * ((((float)(int8_t)g_magnetometer_calib[2] - 128) / 256) + 1);

    return MPU9250_OK;
}




int32_t mpu9250_drv_write(uint8_t reg_addr, uint8_t const * const p_write_buf, size_t len)
{
    if(len > (BUF_LEN - 1)){
        return MPU9250_BUFFER_SIZE;
    }
    if(true == g_xfer_in_progress){
        return MPU9250_BUSY;
    }
    g_xfer_in_progress = true;
    g_tx_buf[0] = reg_addr;
    memcpy(g_tx_buf+1, p_write_buf, len);
    g_tx_len = len+1;
    g_rx_len = 0;
    nrfx_spi_xfer_desc_t xfer_desc = NRFX_SPI_XFER_TX(g_tx_buf, g_tx_len);
    nrfx_err_t err = nrfx_spi_xfer(&g_spi, &xfer_desc, 0);
    if(NRFX_SUCCESS != err){
        return  MPU9250_DRIVER_READWRITE_FAIL;
    }

    return MPU9250_OK;
}

int32_t mpu9250_drv_write_blocking(uint8_t reg_addr, uint8_t const * const p_write_buf, size_t len)
{
    int32_t ret = mpu9250_drv_write(reg_addr, p_write_buf, len);
    if(MPU9250_OK != ret){
        return ret;
    }
    while(true == g_xfer_in_progress);
    return ret;
}

int32_t mpu9250_drv_read(uint8_t reg_addr, size_t len)
{
    if(len > (BUF_LEN - 1)){
        return MPU9250_BUFFER_SIZE;
    }
    if(true == g_xfer_in_progress){
        return MPU9250_BUSY;
    }

    g_xfer_in_progress = true;
    reg_addr |= 0x80;
    g_tx_buf[0] = reg_addr;
    g_tx_len = len+1;
    g_rx_len = len+1;
    nrfx_spi_xfer_desc_t xfer_desc = NRFX_SPI_SINGLE_XFER(g_tx_buf, g_tx_len, g_rx_buf, g_rx_len);
    nrfx_err_t err = nrfx_spi_xfer(&g_spi, &xfer_desc, 0);
    if(NRFX_SUCCESS != err){
        return  MPU9250_DRIVER_READWRITE_FAIL;
    }

    return MPU9250_OK;
}

int32_t mpu9250_drv_read_blocking(uint8_t reg_addr, uint8_t * p_buf, size_t len)
{
    int32_t ret = mpu9250_drv_read(reg_addr, len);
    if(MPU9250_OK != ret){
        return ret;
    }
    while(true == g_xfer_in_progress);
    memcpy(p_buf, g_rx_buf+1, len);
    return ret;

}

bool mpu9250_drv_readwrite_active(void)
{
    return g_xfer_in_progress;
}
//-------------------------LOCAL FUNCTIONS----------------------------------
static void nrfx_spi_evt_handler(nrfx_spi_evt_t const * p_event, void * p_context)
{
    switch(p_event->type){
        case NRFX_SPI_EVENT_DONE:{
            g_xfer_in_progress = false;
            //nrfx_spi_xfer_desc_t * p_desc = *(p_event->xfer_desc);
            break;
        }

        default:{
            break;
        }
    }
}
