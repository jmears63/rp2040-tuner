#include "tuner.h"
#include "LCD_1in28.h"
#include "imu.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"


static void DrawCentredString_EN(UWORD Ystart, const char * pString, sFONT* Font, int fontWidth, UWORD Color_Foreground, UWORD Color_Background)
{
    const size_t len = strlen(pString);
    Paint_DrawString_EN((LCD_1IN28.WIDTH - len * fontWidth) >> 1, Ystart, pString, &Font24, Color_Foreground, Color_Background);
}



int main(void)
{
    DEV_Module_Init();

    // These lines seems to duplicate code in DEV_Config.c:
    adc_init();
    adc_gpio_init(BAT_ADC_PIN);
    adc_select_input(BAT_CHANNEL);

    // Initialise the LCD:
    LCD_1IN28_Init(HORIZONTAL);
    LCD_1IN28_Clear(WHITE);
    // Turn on the back light. Allowed values 0-100:
    DEV_SET_PWM(60);    

    // Create an image buffer to populate:
    UDOUBLE Imagesize = LCD_1IN28_HEIGHT * LCD_1IN28_WIDTH * 2;
    void *ImageBuffer = malloc(Imagesize);  // void* because typing is not very consistent.

    // Create a new image cache from the bugger, unrotated, and fill it with white:
    Paint_NewImage(ImageBuffer, LCD_1IN28.WIDTH, LCD_1IN28.HEIGHT, 0, WHITE);
    // Select the number of colours: 2 4 16 or 65 (65K) are supported.
    Paint_SetScale(65);     

    // Not sure why we clear in both rotations:
    Paint_Clear(WHITE);
    Paint_SetRotate(ROTATE_0);      // 90 degree multiples are available.
    Paint_Clear(WHITE);

    const int fontWidth = 17;
    DrawCentredString_EN(50, "Tuner", &Font24, fontWidth, BLACK, WHITE);        // Foreground, background colours.

    LCD_1IN28_Display(ImageBuffer);

    imu_initialize();

    float acc[3], gyro[3];
    unsigned int tim_count = 0;

    while (1)
    {
    //    QMI8658_read_xyz(acc, gyro, &tim_count);

        // This seems to overwrite pre-existing numbers in the buffer:
        Paint_DrawNum(120, 90, acc[0], &Font16, 2, BLACK, WHITE);
        Paint_DrawNum(120, 105, acc[1], &Font16, 2, BLACK, WHITE);
        Paint_DrawNum(120, 120, acc[2], &Font16, 2, BLACK, WHITE);

        LCD_1IN28_Display(ImageBuffer);
        DEV_Delay_ms(100);
    }

    // We never get here, but let's do the right thing:
    free(ImageBuffer);
    ImageBuffer = NULL;

    DEV_Module_Exit();
    return 0;
}
