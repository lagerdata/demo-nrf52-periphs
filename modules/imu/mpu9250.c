#define _MPU9250_C_SRC

//-------------------------MODULES USED-------------------------------------
#include "nrfx_spi.h"
#include "nrf_gpio.h"


#include "mpu9250.h"


//-------------------------DEFINITIONS AND MACORS---------------------------
#define SPI_INSTANCE  0
#define BUF_LEN (0x20)
//MSG_BUF_LEN must be a multiple of 0x10
#define MSG_BUF_LEN (0x20)

#define NRFX_SPI_SCK_PIN  30
#define NRFX_SPI_SDO_PIN 31
#define NRFX_SPI_SDI_PIN 28
#define NRFX_SPI_SS_PIN   29




//-------------------------TYPEDEFS AND STRUCTURES--------------------------
typedef struct spi_msg{
    uint8_t tx_buf[BUF_LEN];
    size_t tx_len;
    uint8_t *p_rx_buf;
    uint8_t rx_buf[BUF_LEN];
    size_t rx_len;
}spi_msg_t;

typedef struct spi_buffer{
    spi_msg_t msg[MSG_BUF_LEN];
    uint32_t cnt;
    uint32_t active_index;
    bool xfer_in_prog;
}spi_buffer_t;
//-------------------------PROTOTYPES OF LOCAL FUNCTIONS--------------------
static void nrfx_spi_evt_handler(nrfx_spi_evt_t const * p_event, void * p_context);


//-------------------------EXPORTED VARIABLES ------------------------------



//-------------------------GLOBAL VARIABLES---------------------------------
static const nrfx_spi_t g_spi = NRFX_SPI_INSTANCE(SPI_INSTANCE);
static spi_buffer_t g_spi_buf = {.cnt = 0,
                                 .active_index = 0,
                                 . xfer_in_prog = false};
//static uint8_t g_magnetometer_calib[3];
static float g_gyro_div;
static float g_accel_div;

//-------------------------EXPORTED FUNCTIONS-------------------------------

int32_t mpu9250_init(void)
{
    g_spi_buf.cnt = 0;
    g_spi_buf.active_index = 0;
    g_spi_buf.xfer_in_prog = false;

    nrfx_spi_config_t spi_config;
    spi_config.sck_pin = NRFX_SPI_SCK_PIN;
    spi_config.mosi_pin = NRFX_SPI_SDO_PIN;
    spi_config.miso_pin = NRFX_SPI_SDI_PIN;
    spi_config.ss_pin = NRFX_SPI_SS_PIN;
    spi_config.irq_priority = 2;
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
                              {MPU9250_REG_PWR_MGMT_2, 0x00},  /* Disable Accel & Gyro */
                              {MPU9250_REG_CONFIG, 0x00},
                              {MPU9250_REG_GYRO_CONFIG, 0x18},
                              {MPU9250_REG_ACCEL_CONFIG, 0x08},
                              {MPU9250_REG_ACCEL_CONFIG2, 0x09},
                              {MPU9250_REG_INT_PIN_CFG, 0x30},  /* Pin config */
                              //{MPU9250_REG_I2C_MST_CTRL, 0x0D},  /* I2C multi-master / 400kHz */
                              {MPU9250_REG_USER_CTRL, 0x20},  /* I2C master mode */
                              {0xde, 0xad}};
    for(int i=0;accel_gyro_init_conf[i][0] != 0xde;i++){
        mpu9250_drv_write(accel_gyro_init_conf[i][0], &accel_gyro_init_conf[i][1], 1);
    }

    const uint8_t mag_init_conf[][2] = {
                          {MPU9250_REG_I2C_SLV0_ADDR, AK8963_I2C_ADDR},  /* reset */
                          {MPU9250_REG_I2C_SLV0_REG, AK8963_REG_CNTL2},  /* clock source */
                          {MPU9250_REG_I2C_SLV0_DO, 0x01},  /* Disable Accel & Gyro */
                          {MPU9250_REG_I2C_SLV0_CTRL, 0x80},  /* Pin config */
                          {0xde, 0xad}};
    for(int i=0;mag_init_conf[i][0] != 0xde;i++){
        mpu9250_drv_write(mag_init_conf[i][0], &mag_init_conf[i][1], 1);
    }

    #if 0
    //read calibration data
    buf = AK8963_I2C_ADDR|0x80;
    mpu9250_drv_write(MPU9250_REG_I2C_SLV0_ADDR, &buf, 1);
    buf = AK8963_REG_ASAX;
    mpu9250_drv_write(MPU9250_REG_I2C_SLV0_REG, &buf, 1);
    buf = 0x83;
    mpu9250_drv_write(MPU9250_REG_I2C_SLV0_CTRL, &buf, 1);
    mpu9250_drv_read_blocking(MPU9250_REG_EXT_SENS_DATA_00, g_magnetometer_calib, 3);


    //16 bit periodic mode (8hz)
    buf = AK8963_I2C_ADDR;
    mpu9250_drv_write(MPU9250_REG_I2C_SLV0_ADDR, &buf, 1);
    buf = AK8963_REG_CNTL1;
    mpu9250_drv_write(MPU9250_REG_I2C_SLV0_REG, &buf, 1);
    buf = 0x12;
    mpu9250_drv_write(MPU9250_REG_I2C_SLV0_DO, &buf, 1);
    buf = 0x81;
    mpu9250_drv_write(MPU9250_REG_I2C_SLV0_CTRL, &buf, 1);

    //read whoami
    buf = AK8963_I2C_ADDR | 0x80;
    mpu9250_drv_write(MPU9250_REG_I2C_SLV0_ADDR, &buf, 1);
    buf = AK8963_REG_WIA;
    mpu9250_drv_write(MPU9250_REG_I2C_SLV0_REG,  &buf, 1);
    buf = 0x81;
    mpu9250_drv_write(MPU9250_REG_I2C_SLV0_CTRL, &buf, 1);


    ret = mpu9250_drv_read_blocking(MPU9250_REG_EXT_SENS_DATA_00, &buf, 1);
    if(MPU9250_OK != ret){
        return MPU9250_DRIVER_INIT_FAIL;
    }
    if(0x48 != buf){
        return MPU9250_DRIVER_INIT_FAIL;
    }
    #endif

    return MPU9250_OK;
}

int32_t mpu9250_uninit(void)
{
    nrfx_spi_uninit(&g_spi);
    return MPU9250_OK;
}


int32_t mpu9250_start_measure(MPU9250_BIT_GYRO_FS_SEL gyro_fs, MPU9250_BIT_ACCEL_FS_SEL accel_fs, MPU9250_BIT_DLPF_CFG dlpf_cfg, MPU9250_BIT_A_DLPFCFG a_dlpfcfg)
{

    #if 0
    const uint8_t init_conf[][2] = {
                              {MPU9250_REG_PWR_MGMT_2,    0x00},                      /* Enable Accel & Gyro */
                              {MPU9250_REG_CONFIG,        (uint8_t)dlpf_cfg},         /* Gyro LPF */
                              {MPU9250_REG_GYRO_CONFIG,   (uint8_t)gyro_fs},          /* Gyro configuration */
                              {MPU9250_REG_ACCEL_CONFIG,  (uint8_t)accel_fs},         /* Accel configuration */
                              {MPU9250_REG_ACCEL_CONFIG2, 0x08 | (uint8_t)a_dlpfcfg}, /* Accel LPF */
                              {0xde,                      0xad}
    };
    for (int i = 0; init_conf[i][0] != 0xde; i++) {
        int32_t ret = mpu9250_drv_write(init_conf[i][0], &init_conf[i][1], 1);
        if(ret != MPU9250_OK){
            return ret;
        }
    }
    #endif
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
    return mpu9250_drv_write(MPU9250_REG_PWR_MGMT_2, &buf, 1);
}

int32_t mpu9250_drv_read_gyro(uint8_t (* p_gyro_buf)[6])
{

    return mpu9250_drv_read_blocking(MPU9250_REG_GYRO_XOUT_HL, (uint8_t *)p_gyro_buf, 6);
}

void mpu9250_drv_process_raw_gyro(MPU9250_accel_val * p_gyro_val, uint8_t (*p_gyro_raw_buf)[6])
{
    p_gyro_val->raw_x = ((uint16_t)(*p_gyro_raw_buf)[0] << 8) | (*p_gyro_raw_buf)[1];
    p_gyro_val->raw_y = ((uint16_t)(*p_gyro_raw_buf)[2] << 8) | (*p_gyro_raw_buf)[3];
    p_gyro_val->raw_z = ((uint16_t)(*p_gyro_raw_buf)[4] << 8) | (*p_gyro_raw_buf)[5];

    p_gyro_val->x = (float)(int16_t)p_gyro_val->raw_x / g_gyro_div;
    p_gyro_val->y = (float)(int16_t)p_gyro_val->raw_y / g_gyro_div;
    p_gyro_val->z = (float)(int16_t)p_gyro_val->raw_z / g_gyro_div;
}



int32_t mpu9250_drv_read_accel(uint8_t (* p_accel_buf)[6])
{

    return mpu9250_drv_read_blocking(MPU9250_REG_ACCEL_XOUT_HL, (uint8_t *)p_accel_buf, 6);
}

void mpu9250_drv_process_raw_accel(MPU9250_accel_val * p_accel_val, uint8_t (*p_accel_raw_buf)[6])
{
    p_accel_val->raw_x = ((uint16_t)(*p_accel_raw_buf[0]) << 8) | (*p_accel_raw_buf)[1];
    p_accel_val->raw_y = ((uint16_t)(*p_accel_raw_buf[2]) << 8) | (*p_accel_raw_buf)[3];
    p_accel_val->raw_z = ((uint16_t)(*p_accel_raw_buf[4]) << 8) | (*p_accel_raw_buf)[5];

    p_accel_val->x = (float)(int16_t)p_accel_val->raw_x / g_accel_div;
    p_accel_val->y = (float)(int16_t)p_accel_val->raw_y / g_accel_div;
    p_accel_val->z = (float)(int16_t)p_accel_val->raw_z / g_accel_div;
}



int32_t mpu9250_drv_write(uint8_t reg_addr, uint8_t const * const p_write_buf, size_t len)
{
    if(len > (BUF_LEN - 1)){
        return MPU9250_BUFFER_SIZE;
    }
    g_spi_buf.msg[g_spi_buf.cnt].rx_len = 0;
    g_spi_buf.msg[g_spi_buf.cnt].p_rx_buf = NULL;

    uint8_t * p_tx_buf = &g_spi_buf.msg[g_spi_buf.cnt].tx_buf[0];
    size_t * p_tx_len =  &g_spi_buf.msg[g_spi_buf.cnt].tx_len;

    p_tx_buf[0] = reg_addr;
    memcpy(p_tx_buf+1, p_write_buf, len);
    *p_tx_len = len + 1;
    //Increment cnt and wrap
    g_spi_buf.cnt++;
    g_spi_buf.cnt = (g_spi_buf.cnt)&(MSG_BUF_LEN-1);

    if(false == g_spi_buf.xfer_in_prog){
        g_spi_buf.xfer_in_prog = true;
        p_tx_buf = &g_spi_buf.msg[g_spi_buf.active_index].tx_buf[0];
        p_tx_len =  &g_spi_buf.msg[g_spi_buf.active_index].tx_len;

        nrfx_spi_xfer_desc_t xfer_desc = NRFX_SPI_XFER_TX(p_tx_buf, *p_tx_len);
        nrfx_err_t err = nrfx_spi_xfer(&g_spi, &xfer_desc, 0);
        if(NRFX_SUCCESS != err){
            return  MPU9250_DRIVER_READWRITE_FAIL;
        }
    }

    return MPU9250_OK;
}


int32_t mpu9250_drv_read(uint8_t reg_addr, uint8_t * p_read_buf, size_t len)
{
    if(len > (BUF_LEN - 1)){
        return MPU9250_BUFFER_SIZE;
    }

    uint8_t * p_tx_buf = &g_spi_buf.msg[g_spi_buf.cnt].tx_buf[0];
    size_t * p_tx_len =  &g_spi_buf.msg[g_spi_buf.cnt].tx_len;

    g_spi_buf.msg[g_spi_buf.cnt].p_rx_buf = p_read_buf;
    size_t * p_rx_len =  &g_spi_buf.msg[g_spi_buf.cnt].rx_len;

    reg_addr |= 0x80;
    p_tx_buf[0] = reg_addr;

    *p_tx_len = len + 1;
    *p_rx_len = len;
    //Increment cnt and wrap
    g_spi_buf.cnt++;
    g_spi_buf.cnt = (g_spi_buf.cnt)&(MSG_BUF_LEN-1);

    if(false == g_spi_buf.xfer_in_prog){
        g_spi_buf.xfer_in_prog = true;
        p_tx_buf = &g_spi_buf.msg[g_spi_buf.active_index].tx_buf[0];
        p_tx_len =  &g_spi_buf.msg[g_spi_buf.active_index].tx_len;
        uint8_t * p_rx_buf = &g_spi_buf.msg[g_spi_buf.active_index].rx_buf[0];

        nrfx_spi_xfer_desc_t xfer_desc = NRFX_SPI_SINGLE_XFER(p_tx_buf, *p_tx_len, p_rx_buf, *p_tx_len);
        nrfx_err_t err = nrfx_spi_xfer(&g_spi, &xfer_desc, 0);
        if(NRFX_SUCCESS != err){
            return  MPU9250_DRIVER_READWRITE_FAIL;
        }
    }
    return MPU9250_OK;
}

int32_t mpu9250_drv_read_blocking(uint8_t reg_addr, uint8_t * p_read_buf, size_t len)
{

    int32_t ret = mpu9250_drv_read(reg_addr, p_read_buf, len);
    //get index associated with this message
    uint32_t index = 0;
    for(uint32_t i = 0;i<MSG_BUF_LEN;i++){
        if(p_read_buf == g_spi_buf.msg[i].p_rx_buf){
            index = i;
        }
    }
    
    while(g_spi_buf.msg[index].rx_len > 0);

    return ret;
}


//-------------------------LOCAL FUNCTIONS----------------------------------
static void nrfx_spi_evt_handler(nrfx_spi_evt_t const * p_event, void * p_context)
{
    switch(p_event->type){
        case NRFX_SPI_EVENT_DONE:{
            if(NULL != g_spi_buf.msg[g_spi_buf.active_index].p_rx_buf){
                //copy data into provided buffer
                memcpy(g_spi_buf.msg[g_spi_buf.active_index].p_rx_buf, g_spi_buf.msg[g_spi_buf.active_index].rx_buf+1, g_spi_buf.msg[g_spi_buf.active_index].rx_len);
                //Set rx length to 0 for so that blocking calls know that the transfer is complete
                g_spi_buf.msg[g_spi_buf.active_index].rx_len = 0;
            }

            //increment active index and wrap
            g_spi_buf.active_index++;
            g_spi_buf.active_index = ((g_spi_buf.active_index)&(MSG_BUF_LEN-1));
            if(g_spi_buf.cnt == g_spi_buf.active_index){//Active index has caught up to counter, no messages left in buffer
                g_spi_buf.xfer_in_prog = false;
            }else{//still have messages in buffer
                uint8_t * p_tx_buf = &g_spi_buf.msg[g_spi_buf.active_index].tx_buf[0];
                size_t * p_tx_len =  &g_spi_buf.msg[g_spi_buf.active_index].tx_len;

                uint8_t * p_rx_buf = &g_spi_buf.msg[g_spi_buf.active_index].rx_buf[0];

                if(0 == g_spi_buf.msg[g_spi_buf.active_index].rx_len){//spi write
                    nrfx_spi_xfer_desc_t xfer_desc = NRFX_SPI_XFER_TX(p_tx_buf, *p_tx_len);
                    nrfx_spi_xfer(&g_spi, &xfer_desc, 0);

                }else{//spi read
                    nrfx_spi_xfer_desc_t xfer_desc = NRFX_SPI_SINGLE_XFER(p_tx_buf, *p_tx_len, p_rx_buf, *p_tx_len);
                    nrfx_spi_xfer(&g_spi, &xfer_desc, 0);
                }
            }

            //nrfx_spi_xfer_desc_t * p_desc = *(p_event->xfer_desc);
            break;

        }

        default:{
            break;
        }
    }
}
