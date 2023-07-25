#include <config/appconfig.h>
#include <system/system-freertos.hpp>
#include <hardware/hardware-manager.hpp>
#include <cstdint>
#include <stdbool.h>
#include <stm-hal/hal-tim.hpp>
#include <gscope/gscope-debug.hpp>

static const TickType_t s_tick_delay = 5;
static const float s_roll_threshold = 25.0f;

static TaskHandle_t s_task_handler;
static bool s_led_on = false;
static bool s_rgb_on = true;

static HardwareManager s_manager;
static Hardware_Config s_hardware_config;
static Neoled s_neoled;
static Led s_led;
static IMU s_imu;

static Neoled_Colour s_colour = NEO_WHITE;
static IMU_Data s_imu_out;

/**
 * Hardware manager task.
 *
 * The colour of the RGB LED changes depending on roll pitch from imu.
*/
static void s_hardware_manager_thread(void*)
{
    s_manager.init();

    s_hardware_config = s_manager.get_hardware_config();
    s_neoled = s_manager.get_neoled();
    s_led = s_manager.get_led();
    s_imu = s_manager.get_imu();

    s_neoled.init();
    s_led.init();
    s_imu.init();

    s_neoled.set_enable(s_rgb_on);

    for ( int i = 0 ;; i++ )
    {
        s_imu.update();
        s_imu_out = s_imu.get_imu_data();

        uint32_t now = hal_timer_32_ms();
        static uint32_t s_neo_timestamp;

        if ((now - s_neo_timestamp) >= s_hardware_config.neoled_update_interval)
        {
            s_neo_timestamp = now;

            s_colour = (s_imu_out.roll[0] > s_roll_threshold || s_imu_out.roll[0] < -s_roll_threshold) ? NEO_GREEN : NEO_RED;

            s_neoled.set_brightness(NEO_BRI_1);
            s_neoled.set_colour(s_colour);
            s_neoled.update();
        }

        static uint32_t s_led_timestamp;

        if ((now - s_led_timestamp) >= s_hardware_config.led_update_interval)
        {
            s_led_timestamp = now;

            s_led.set_led_2(!s_led_on);                            // top led
            s_led.set_led_1((s_imu_out.roll[0] > -s_roll_threshold));   // middle led
            s_led.set_led_3((s_imu_out.roll[0] < s_roll_threshold));    // bottom led

            s_led_on = !s_led_on;
            s_led.update();
            i = 0;
        }

        vTaskDelay(s_tick_delay);
    }
}

// ------------------------------------------------------------------------------------
//                                      PUBLIC API
// ------------------------------------------------------------------------------------

void task_hardware_manager_init()
{
    // Create rainbow task
    static StaticTask_t s_task_buffer;
    static StackType_t s_stack[SIZE_HARDWARE_MAN];

    s_task_handler = xTaskCreateStatic(
        s_hardware_manager_thread,
        "Hardware",
        SIZE_HARDWARE_MAN,
        0,
        PRI_HARDWARE_MAN,
        s_stack,
        &s_task_buffer
    );
}
