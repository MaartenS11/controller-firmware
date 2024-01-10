#include <hardware/hardware-manager.hpp>
#include <hardware/actuator/motor/ev3-large-motor.hpp>
#include <module/module-output.hpp>

#include <../../controller/stm-hal/hal-gpio.hpp>
#include <../../controller/stm-hal/hal-adc.hpp>

// ------------------------------------------------------------------------------------
//                                      PUBLIC API
// ------------------------------------------------------------------------------------

/**
 * Init everything within the hardware manager.
*/
void HardwareManager::init()
{
    NeoLED m_neoled;

    LED m_led;

    IMU m_imu;

    m_hardware_config = Hardware_Config
    {
        .neoled_update_interval = 10,
        .led_update_interval = 250,
        .imu_update_interval = 5,
        .button_update_interval = 5,
    };
    /*m_lego_motors[static_cast<uint8_t>(Lego_Motor_Port::ACTUATOR_A)] = new EV3LargeMotor(new OutputPort(
        {
            .pin5_adc_enable_n_io = MOTORA_PIN5_DETECT_IO,
            .pin6_adc_enable_n_io = MOTORA_PIN6_DETECT_IO,
            .encoder_id = TIMER_TYPE_MOTOR_A_ENCODER,
            .motor_pwm_id = TIMER_TYPE_MOTOR_A_PWM,
            .fault_io = MOTORAB_FAULT_N_IO,
            .adc_pin5_channel = ADC_CHANNEL_TYPE_PORT_OUTPUT1_PIN5,
            .adc_pin6_channel = ADC_CHANNEL_TYPE_PORT_OUTPUT1_PIN6,
            .invert_encoder_polarity = false,
        }
    ), TimerType::TIMER_TYPE_MOTOR_A_PWM, TimerType::TIMER_TYPE_MOTOR_A_ENCODER);*/
}

/**
 * Update everything within the hardware manager.
*/
void HardwareManager::update()
{
    // will update all devices within the hardware manager.
}

/**
 * Gets the config of the hardware manager.
 *
 * @return Hardware_Config
*/
Hardware_Config HardwareManager::get_hardware_config()
{
    return m_hardware_config;
}

/**
 * Get pointer of given actuator type.
 *
 * @return Actuator*
*/
LegoMotor* HardwareManager::get_lego_motor(Lego_Motor_Port actuator_type)
{
    uint8_t actuator = static_cast<uint8_t>(actuator_type);
    return m_lego_motors[actuator];
}

/**
 * Get pointer of given sensor type.
 *
 * @return Sensor*
*/
LegoSensor* HardwareManager::get_lego_sensor(Lego_Sensor_Port sensor_type)
{
    uint8_t sensor = static_cast<uint8_t>(sensor_type);
    return m_lego_sensors[sensor];
}

/**
 * Returns a reference to Neoled.
 *
 * @return Neoled&
*/
NeoLED& HardwareManager::get_neoled()
{
    return m_neoled;
}

/**
 * Returns a reference to Led.
 *
 * @return Led&
*/
LED& HardwareManager::get_led()
{
    return m_led;
}

/**
 * Returns a reference to IMU.
 *
 * @return IMU&
*/
IMU& HardwareManager::get_imu()
{
    return m_imu;
}

/**
 * Returns a reference to Buttons.
 *
 * @return IMU&
*/
Buttons& HardwareManager::get_buttons()
{
    return m_buttons;
}

// ------------------------------------------------------------------------------------
//                                      PRIVATE API
// ------------------------------------------------------------------------------------
