#include "tuner.h"
#include "core1_main.h"
#include "LCD_1in28.h"
#include "imu.h"
#include "processing.h"
#include <string.h>


static void DrawCentredString_EN(UWORD Ystart, const char * pString, sFONT* Font, int fontWidth, UWORD Color_Foreground, UWORD Color_Background)
{
    const size_t len = strlen(pString);
    Paint_DrawString_EN((LCD_1IN28.WIDTH - len * fontWidth) >> 1, Ystart, pString, &Font24, Color_Foreground, Color_Background);
}


void core1_main()
{
    /*
     * Core 1's job is to update the LCD based on data collected by core 0.
     */

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

    volatile clock_t elapsed = 0;

    while (1)
    {
        clock_t t1 = time_us_64();
        Paint_ClearWindows(80, 100, 160, 120, RED);
        for (int i = 0; i < 100; i++) {
            LCD_1IN28_Display(ImageBuffer);
        }
        clock_t t2 = time_us_64();
        elapsed = t2 - t1;
    }

    float acc[3], gyro[3];
    unsigned int tim_count = 0;
    char buf[32];
    while (1)
    {
        sleep_ms(1000);

        snprintf(buf, sizeof(buf), "%d", imu_zc_count);
        imu_zc_count = 0;

        Paint_ClearWindows(80, 100, 160, 120, WHITE);

        DrawCentredString_EN(100, buf, &Font24, fontWidth, BLACK, WHITE);        // Foreground, background colours.
        LCD_1IN28_Display(ImageBuffer);
    }

    // We never get here, but let's do the right thing:
    free(ImageBuffer);
    ImageBuffer = NULL;
}