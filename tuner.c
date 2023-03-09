#include "tuner.h"
#include "LCD_1in28.h"
#include "imu.h"
#include "core1_main.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "pico/multicore.h"


int main(void)
{
    /*
     *  Do all hardware and software initialization.
     */

    DEV_Module_Init();

    // These lines seems to duplicate code in DEV_Config.c:
    adc_init();
    adc_gpio_init(BAT_ADC_PIN);
    adc_select_input(BAT_CHANNEL);

    // Initialise the LCD:
    LCD_1IN28_Init(HORIZONTAL);

    // Initializing the IMU in this core (core 0) means that its interrupts will be service by this core.
    // This call starts data collection from the IMU:
    imu_initialize();

    /*
     *  Ready to kick off the other core.
     */

    multicore_launch_core1(core1_main);

    // Twiddle our thumbs while the interrupt handler does the work.
    while (1) {
    }


    DEV_Module_Exit();
    return 0;
}
