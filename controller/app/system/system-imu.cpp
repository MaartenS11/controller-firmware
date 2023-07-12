#include <assert.h>
#include <config/appconfig.h>
#include <cstring>
#include <gscope/gscope.hpp>
#include <lsm6dsl/lsm6dsl.hpp>
#include <stm-hal/hal-gpio.hpp>
#include <stm-hal/hal-tim.hpp>
#include <stm-hal/hal-spi.hpp>
#include <system/system-freertos.hpp>
#include <complementary-filter/complementary-filter.hpp>

GScopeChannel(s_accelerometer, "imu_accel_n2m", float, LSM6DS::TOTAL_AXIS)
GScopeChannel(s_gyro, "imu_gyro_dps", float, LSM6DS::TOTAL_AXIS)
GScopeChannel(s_gyro_integrated, "imu_gyro_integrated_rad", float, LSM6DS::TOTAL_AXIS)
GScopeChannel(s_gyro_omega, "imu_gyro_omega_rad_s", float, LSM6DS::TOTAL_AXIS)
GScopeChannel(s_imu_roll_pitch, "imu_roll_pitch", float, 2)

static SemaphoreHandle_t s_wait_tx_finish;
static uint8_t s_tx_buffer[16];
static uint8_t s_rx_buffer[16];

static void s_set_chip_select(bool enable)
{
    enable ? hal_gpio_reset_pin(SYS_IMU_CS_IO) : hal_gpio_set_pin(SYS_IMU_CS_IO);
}

static bool s_spi_write(uint8_t, uint8_t reg, uint8_t* buf, uint16_t size)
{
    assert (size < (sizeof(s_tx_buffer) - 1));

    s_tx_buffer[0] = reg;
    std::memcpy(s_tx_buffer + 1, buf, size);

    hal_spi_transmit_receive(SPI_TYPE_IMU_FLASH, s_tx_buffer, s_rx_buffer, size + 1);

    // Wait until transmition finish
    return system_freertos_semaphore_take(s_wait_tx_finish, 200);
}

static bool s_spi_read(uint8_t, uint8_t reg, uint8_t* buf, uint16_t size)
{
    assert (size < sizeof(s_rx_buffer));

    std::memset(s_tx_buffer, 0, sizeof(s_tx_buffer));
    s_tx_buffer[0] = reg | 0x80;

    hal_spi_transmit_receive(SPI_TYPE_IMU_FLASH, s_tx_buffer, s_rx_buffer, size + 1);

    // Wait until transmition finish
    bool success = system_freertos_semaphore_take(s_wait_tx_finish, 200);
    std::memcpy(buf, s_rx_buffer + 1, size);

    return success;
}

static void s_spi_end_callback(void*)
{
    system_freertos_semaphore_give(s_wait_tx_finish);
}

static constexpr LSM6DS::Config s_imu_config =
{
    .read_cb = s_spi_read,
    .write_cb = s_spi_write,
    .chip_select_cb = s_set_chip_select,
    .get_timestamp_ms32_cb = hal_timer_32_ms,
    .address = 0, // SPI is un use
};

static LSM6DS s_imu(s_imu_config);
static ComplementaryFilter s_complementary_filter;

static float accel[LSM6DS::TOTAL_AXIS];
static float gyro[LSM6DS::TOTAL_AXIS];
static float roll_pitch[2];
static uint32_t s_timestamp_us;

// ------------------------------------------------------------------------------------
//                                      PUBLIC API
// ------------------------------------------------------------------------------------

/**
 * Init the system IMU.
*/
void system_imu_init()
{
    // Mutex to wait for SPI transaction to finish
    static StaticSemaphore_t s_wait_ts_sem;
    s_wait_tx_finish = xSemaphoreCreateBinaryStatic(&s_wait_ts_sem);

    hal_spi_init(SPI_TYPE_IMU_FLASH, s_spi_end_callback, nullptr);

    s_complementary_filter.init();

    s_imu.init();
}

/**
 * Update the system IMU.
*/
void system_imu_update()
{
    uint32_t now = hal_timer_32_us();
    uint32_t elapse = now - s_timestamp_us;
    s_timestamp_us = now;

    s_imu.update();
    s_imu.get_acceleration(accel);
    s_imu.get_gyro(gyro);

    // Produce raw accelerometer and gyro values
    s_accelerometer.produce(accel);
    s_gyro.produce(gyro);
    s_complementary_filter.update(accel, gyro, static_cast<float>(elapse) * 1e-6);

    // Produce some output of the complementary filter
    s_gyro_integrated.produce(s_complementary_filter.get_integrated_gyro_rad());
    s_gyro_omega.produce(s_complementary_filter.get_angular_speed_rad_s());
    s_complementary_filter.get_roll_pitch(roll_pitch);
    s_imu_roll_pitch.produce(roll_pitch);
}

/**
 * Get the acceleration values of the system IMU.
 *
 * @return float*
*/
float* system_imu_accel()
{
    return accel;
}

/**
 * Get the gyro values of the system IMU.
 *
 * @return float*
*/
float* system_imu_gyro()
{
    return gyro;
}

/**
 * Get the roll pitch of the system IMU.
 *
 * @return float*
*/
float* system_imu_roll_pitch()
{
    return roll_pitch;
}
