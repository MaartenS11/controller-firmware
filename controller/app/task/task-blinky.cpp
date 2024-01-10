#include <config/appconfig.h>
#include <system/system-freertos.hpp>
#include <stm-hal/hal-gpio.hpp>
#include <hardware/hardware-manager.hpp>

static TaskHandle_t s_task_handler;
static bool s_enabled = false;

static HardwareManager s_manager;

static void s_blinky_thread(void*)
{
    s_manager.init();

    while(1)
    {
        s_enabled = !s_enabled;

        if (s_enabled)
        {
            hal_gpio_set_pin(LED1_IO);
            hal_gpio_reset_pin(LED2_IO);
            hal_gpio_set_pin(LED3_IO);
        }
        else
        {
            hal_gpio_reset_pin(LED1_IO);
            hal_gpio_set_pin(LED2_IO);
            hal_gpio_reset_pin(LED3_IO);
        }

        vTaskDelay(1000);

        LegoMotor *lego_motor = s_manager.get_lego_motor(Lego_Motor_Port::ACTUATOR_A);

        if (lego_motor) {
            hal_gpio_set_pin(LED2_IO);
        }

        lego_motor->set_motor_speed(DEFAULT_MOTOR_SPEED);
        lego_motor->forward(1);
        vTaskDelay(1000);
        lego_motor->stop();

        vTaskDelay(5000);

        vTaskDelay(550);
    }
}

// ------------------------------------------------------------------------------------
//                                      PUBLIC API
// ------------------------------------------------------------------------------------
void task_blinky_init()
{
    // Create task for Blinky App
    static StaticTask_t s_task_buffer;
    static StackType_t s_stack[SIZE_BLINKY];

    s_task_handler = xTaskCreateStatic(s_blinky_thread, "Blinky", SIZE_BLINKY,
        0, PRI_BLINKY, s_stack, &s_task_buffer);
}