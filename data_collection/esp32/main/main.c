#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"


#define TAG "MPU9250"

/* MPU registers */
#define WHO_AM_I        0x75
#define PWR_MGMT_1      0x6B
#define PWR_MGMT_2      0x6C
#define USER_CTRL       0x6A
#define CONFIG          0x1A
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define ACCEL_CONFIG2   0x1D

#define I2C_MST_CTRL    0x24
#define I2C_SLV0_ADDR   0x25
#define I2C_SLV0_REG    0x26
#define I2C_SLV0_CTRL   0x27

#define ACCEL_XOUT_H    0x3B
#define EXT_SENS_DATA_00 0x49

static spi_device_handle_t mpu;

/* ---------- SPI helpers ---------- */

static void mpu_write(uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = { reg & 0x7F, data };
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
    };
    spi_device_transmit(mpu, &t);
}

static void mpu_read(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t tx[len + 1];
    uint8_t rx[len + 1];
    memset(tx, 0, sizeof(tx));

    tx[0] = reg | 0x80;  // read bit

    spi_transaction_t t = {
        .length = (len + 1) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    spi_device_transmit(mpu, &t);

    memcpy(buf, &rx[1], len); // discard dummy byte
}

/* ---------- MPU init ---------- */

static void mpu_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));

    mpu_write(PWR_MGMT_1, 0x80);  // reset
    vTaskDelay(pdMS_TO_TICKS(100));

    mpu_write(PWR_MGMT_1, 0x01);  // PLL X gyro
    mpu_write(PWR_MGMT_2, 0x00);

    mpu_write(USER_CTRL, 0x10);   // I2C_IF_DIS
    mpu_write(CONFIG, 0x02);
    mpu_write(GYRO_CONFIG, 0x08);   // ±500 dps
    mpu_write(ACCEL_CONFIG, 0x08);  // ±4g
    mpu_write(ACCEL_CONFIG2, 0x02);

    /* Enable I2C master for magnetometer */
    mpu_write(USER_CTRL, 0x30);   // I2C_MST_EN + I2C_IF_DIS
    mpu_write(I2C_MST_CTRL, 0x0D); // 400 kHz

    /* AK8963: power down */
    mpu_write(I2C_SLV0_ADDR, 0x0C);
    mpu_write(I2C_SLV0_REG, 0x0A);
    mpu_write(I2C_SLV0_CTRL, 0x81);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* AK8963: continuous mode 2 (100 Hz) */
    mpu_write(I2C_SLV0_ADDR, 0x0C);
    mpu_write(I2C_SLV0_REG, 0x0A);
    mpu_write(I2C_SLV0_CTRL, 0x16 | 0x80);
}

/* ---------- app_main ---------- */

void app_main(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = 23,
        .miso_io_num = 19,
        .sclk_io_num = 18,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = 5,
        .queue_size = 1
    };

    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI2_HOST, &devcfg, &mpu);

    mpu_init();

    uint8_t raw[14];
    uint8_t mag[6];

    while (1) {
        mpu_read(ACCEL_XOUT_H, raw, 14);
        mpu_read(EXT_SENS_DATA_00, mag, 6);

        int16_t ax = (raw[0] << 8) | raw[1];
        int16_t ay = (raw[2] << 8) | raw[3];
        int16_t az = (raw[4] << 8) | raw[5];

        int16_t gx = (raw[8] << 8) | raw[9];
        int16_t gy = (raw[10] << 8) | raw[11];
        int16_t gz = (raw[12] << 8) | raw[13];

        int16_t mx = (mag[1] << 8) | mag[0];
        int16_t my = (mag[3] << 8) | mag[2];
        int16_t mz = (mag[5] << 8) | mag[4];

        ESP_LOGI(TAG,
            "ACC[%d %d %d] GYR[%d %d %d] MAG[%d %d %d]",
            ax, ay, az, gx, gy, gz, mx, my, mz);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

