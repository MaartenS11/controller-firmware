#pragma once

#include <actuator/hardware-actuator.hpp>
#include <actuator/lego-motor.hpp>
#include <led/board-neoled.hpp>
#include <led/board-led.hpp>
#include <sensor/hardware-sensor.hpp>
#include <sensor/lego-sensor.hpp>
#include <imu/board-imu.hpp>
#include <battery/board-battery.hpp>
#include <button/board-buttons.hpp>
#include <mutex>

#include <hardware/actuator/motor/ev3-large-motor.hpp>
#include <module/module-output.hpp>

#include <../../controller/stm-hal/hal-gpio.hpp>
#include <../../controller/stm-hal/hal-adc.hpp>

static constexpr uint16_t PORT_COUNT = 4;

enum class Lego_Motor_Port
{
    ACTUATOR_A = 0,
    ACTUATOR_B,
    ACTUATOR_C,
    ACTUATOR_D
};

enum class Lego_Sensor_Port
{
    SENSOR_1 = 0,
    SENSOR_2,
    SENSOR_3,
    SENSOR_4
};

struct Hardware_Config
{
    uint16_t neoled_update_interval;
    uint16_t led_update_interval;
    uint16_t imu_update_interval;
    uint16_t button_update_interval;
};

class HardwareManager
{
    public:

        HardwareManager() {};

        void init();

        void update();

        Hardware_Config get_hardware_config();

        LegoMotor* get_lego_motor(Lego_Motor_Port actuator);

        LegoSensor* get_lego_sensor(Lego_Sensor_Port sensor);

        NeoLED& get_neoled();

        LED& get_led();

        IMU& get_imu();

        Buttons& get_buttons();

    private:
        Hardware_Config m_hardware_config;

        OutputPort port = OutputPort({
                .pin5_adc_enable_n_io = MOTORA_PIN5_DETECT_IO,
                .pin6_adc_enable_n_io = MOTORA_PIN6_DETECT_IO,
                .encoder_id = TIMER_TYPE_MOTOR_A_ENCODER,
                .motor_pwm_id = TIMER_TYPE_MOTOR_A_PWM,
                .fault_io = MOTORAB_FAULT_N_IO,
                .adc_pin5_channel = ADC_CHANNEL_TYPE_PORT_OUTPUT1_PIN5,
                .adc_pin6_channel = ADC_CHANNEL_TYPE_PORT_OUTPUT1_PIN6,
                .invert_encoder_polarity = false,
            });
        EV3LargeMotor motor = EV3LargeMotor(&port, TimerType::TIMER_TYPE_MOTOR_A_PWM, TimerType::TIMER_TYPE_MOTOR_A_ENCODER);

        std::array<LegoMotor*, PORT_COUNT> m_lego_motors =
        {
            &motor,
            nullptr,
            nullptr,
            nullptr
        };

        std::array<LegoSensor*, PORT_COUNT> m_lego_sensors =
        {
            nullptr,
            nullptr,
            nullptr,
            nullptr
        };

        NeoLED m_neoled;

        LED m_led;

        IMU m_imu;

        Buttons m_buttons;
};
