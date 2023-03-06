#include "tuner.h"
#include "LCD_1in28.h"
#include "QMI8658.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

#define IMU_INT2_PIN 24
#define IMU_INT1_PIN 23


// Register addresses for FIFO according to the QMU8658 data sheet, different from the
// sample code:
enum FIFO_REGISTERS {
    FIFO_WTM_TH = 19,
    FIFO_CTRL = 20,
    FIFO_SMPL_CNT_LS = 21,
    FIFO_STATUS = 22,
    FIFO_DATA = 23
};

#define CTRL_CMD_REQ_FIFO 0x05

#define FIFO_HIGH_WATER_THRESHOLD 64      // The FIFO High Water Threshold.


static void DrawCentredString_EN(UWORD Ystart, const char * pString, sFONT* Font, int fontWidth, UWORD Color_Foreground, UWORD Color_Background)
{
    const size_t len = strlen(pString);
    Paint_DrawString_EN((LCD_1IN28.WIDTH - len * fontWidth) >> 1, Ystart, pString, &Font24, Color_Foreground, Color_Background);
}

int counter = 0;

int read_pending = false;

/**
 * This interrupt handle INT2 events, which generally indicate that data is ready
 * to be read from the IMU.
 */
void imu_int2_handler(uint gpio, uint32_t events)
{

    unsigned char sample_count_ls, fifo_status;
    QMI8658_read_reg(FIFO_SMPL_CNT_LS, &sample_count_ls, 1);
    QMI8658_read_reg(FIFO_STATUS, &fifo_status, 1);

    // Read the data: issue a host command that will result in INT1 when the data is ready to be read:
    read_pending = true;
    QMI8658_write_reg(QMI8658Register_Ctrl9, CTRL_CMD_REQ_FIFO);

    counter++;
}

/**
 * This interrupt handle INT1 events, which generally indicates that a ctrl9 command has completed.
 */
void imu_int1_handler(uint gpio, uint32_t events)
{
    // Bit 0 should be set at this point; reading it clears it and the interrupt:
    unsigned char stat1;
    QMI8658_read_reg(QMI8658Register_Status1, &stat1, 1);

    if (read_pending) {
        read_pending = false;
        // Finally we can actually read the data - HTW sets of values.
        unsigned char data_buf[6];
        QMI8658_read_reg(FIFO_DATA, data_buf, sizeof(data_buf));
    }
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

    // TODO: extract the following code to a function.

    float acc[3], gyro[3];
    unsigned int tim_count = 0;
    QMI8658_init();

    struct QMI8658Config QMI8658_config;
    QMI8658_config.inputSelection = QMI8658_CTRL7_DISABLE_ALL; // Initially nothing enabled.
    QMI8658_config.accRange = QMI8658AccRange_8g;
    QMI8658_config.accOdr = QMI8658AccOdr_1000Hz;
    QMI8658_config.gyrRange = QMI8658GyrRange_512dps; // QMI8658GyrRange_2048dps   QMI8658GyrRange_1024dps
    QMI8658_config.gyrOdr = QMI8658GyrOdr_1000Hz;
    QMI8658_config.magOdr = QMI8658MagOdr_125Hz;
    QMI8658_config.magDev = MagDev_AKM09918;
    QMI8658_config.aeOdr = QMI8658AeOdr_128Hz;
    QMI8658_Config_apply(&QMI8658_config);

    // All inputs are disabled while we set things up.

    // Enable FIFO with a HWM:
    QMI8658_write_reg(FIFO_CTRL, 0x0D);     // 0000 1101    FIFO mode, 128 samples size.
    QMI8658_write_reg(FIFO_WTM_TH, FIFO_HIGH_WATER_THRESHOLD);     // High water mark for generating interrupt INT2.

    // Configure GPIO24 (INT2) as a source of interrupt: INT2 signals that data is ready to be read.
    gpio_set_irq_enabled_with_callback(IMU_INT2_PIN, GPIO_IRQ_EDGE_RISE, true, &imu_int2_handler);
    gpio_set_irq_enabled_with_callback(IMU_INT1_PIN, GPIO_IRQ_EDGE_RISE, true, &imu_int1_handler);

    // Finally enable just the accelerometer input:
    QMI8658_config.inputSelection = QMI8658_CONFIG_ACC_ENABLE;
    QMI8658_Config_apply(&QMI8658_config);


    while (1)
    {
        QMI8658_read_xyz(acc, gyro, &tim_count);

        // This seems to overwrite pre-existing numbers in the buffer:
        Paint_DrawNum(120, 90, acc[0], &Font16, 2, BLACK, WHITE);
        Paint_DrawNum(120, 105, acc[1], &Font16, 2, BLACK, WHITE);
        Paint_DrawNum(120, 120, acc[2], &Font16, 2, BLACK, WHITE);

        LCD_1IN28_Display(ImageBuffer);
        DEV_Delay_ms(100);
    }


#if 0

    float acc[3], gyro[3];
    unsigned int tim_count = 0;
    QMI8658_init();
    printf("QMI8658_init\r\n");

    while (true)
    {
        const float conversion_factor = 3.3f / (1 << 12) * 2;
        uint16_t result = adc_read();
        printf("Raw value: 0x%03x, voltage: %f V\n", result, result * conversion_factor);
        Paint_Clear(WHITE);
        QMI8658_read_xyz(acc, gyro, &tim_count);
        printf("acc_x   = %4.3fmg , acc_y  = %4.3fmg , acc_z  = %4.3fmg\r\n", acc[0], acc[1], acc[2]);
        printf("gyro_x  = %4.3fdps, gyro_y = %4.3fdps, gyro_z = %4.3fdps\r\n", gyro[0], gyro[1], gyro[2]);

        printf("tim_count = %d\r\n", tim_count);
        Paint_DrawString_EN(30, 50, "ACC_X = ", &Font16, WHITE, BLACK);
        Paint_DrawString_EN(30, 75, "ACC_Y = ", &Font16, WHITE, BLACK);
        Paint_DrawString_EN(30, 100, "ACC_Z = ", &Font16, WHITE, BLACK);
        Paint_DrawString_EN(30, 125, "GYR_X = ", &Font16, WHITE, BLACK);
        Paint_DrawString_EN(30, 150, "GYR_Y = ", &Font16, WHITE, BLACK);
        Paint_DrawString_EN(30, 175, "GYR_Z = ", &Font16, WHITE, BLACK);
        Paint_DrawString_EN(30, 175, "GYR_Z = ", &Font16, WHITE, BLACK);
        Paint_DrawNum(120, 50, acc[0], &Font16, 2, BLACK, WHITE);
        Paint_DrawNum(120, 75, acc[1], &Font16, 2, BLACK, WHITE);
        Paint_DrawNum(120, 100, acc[2], &Font16, 2, BLACK, WHITE);
        Paint_DrawNum(120, 125, gyro[0], &Font16, 2, BLACK, WHITE);
        Paint_DrawNum(120, 150, gyro[1], &Font16, 2, BLACK, WHITE);
        Paint_DrawNum(120, 175, gyro[2], &Font16, 2, BLACK, WHITE);
        Paint_DrawString_EN(50, 200, "BAT(V)=", &Font16, WHITE, BLACK);
        Paint_DrawNum(130, 200, result * conversion_factor, &Font16, 2, BLACK, WHITE);

        LCD_1IN28_Display(BlackImage);
        DEV_Delay_ms(100);
    }

#endif

    /* Module Exit */
    free(ImageBuffer);
    ImageBuffer = NULL;

    DEV_Module_Exit();
    return 0;
}
