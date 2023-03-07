#include "tuner.h"
#include "LCD_1in28.h"
#include "QMI8658.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

#define IMU_INT2_PIN 24
#define IMU_INT1_PIN 23

/*
    FIFO data transfer:
        The IMU chip behaves more like a QMI865A (rather than a C), documented here:
            https://www.qstcorp.com/upload/pdf/202301/13-52-25%20QMI8658A%20Datasheet%20Rev%20A.pdf
        The protocol for reading FIFO data is somewhat different, amd works.
*/

// Register addresses for FIFO according to the QMU8658 data sheet, different from the
// sample code:
enum FIFO_REGISTERS {
    FIFO_WTM_TH = 19,
    FIFO_CTRL = 20,
    FIFO_SMPL_CNT_LS = 21,
    FIFO_STATUS = 22,
    FIFO_DATA = 23
};

typedef enum imu_commands {
    CTRL_CMD_ACK = 0x00,
    CTRL_CMD_RST_FIFO = 0x04,
    CTRL_CMD_REQ_FIFO = 0x05,           // The data sheet confusingly also mentions 0x0D for this command. 5 seems to work.
    NO_COMMAND = -1
} imu_commands;

#define FIFO_HIGH_WATER_THRESHOLD 16        // The FIFO High Water Threshold.
#define FIFO_CTRL_VALUE 0x0D                // 0000 1101    stream mode, 128 samples FIFO size.
#define FIFO_MAX_SAMPLES 128                // Must match previous definition.


static void imu_command(imu_commands cmd);

static void DrawCentredString_EN(UWORD Ystart, const char * pString, sFONT* Font, int fontWidth, UWORD Color_Foreground, UWORD Color_Background)
{
    const size_t len = strlen(pString);
    Paint_DrawString_EN((LCD_1IN28.WIDTH - len * fontWidth) >> 1, Ystart, pString, &Font24, Color_Foreground, Color_Background);
}

static volatile int counter = 0;

static volatile imu_commands pending_imu_command = NO_COMMAND;

/**
 * This handles INT2 events, which generally indicate that data is ready
 * to be read from the IMU.
 */
static void imu_int2_handler()
{
    unsigned char sample_count_ls, fifo_status;
    QMI8658_read_reg(FIFO_SMPL_CNT_LS, &sample_count_ls, 1);
    QMI8658_read_reg(FIFO_STATUS, &fifo_status, 1);

    volatile uint16_t sample_count = (uint16_t) sample_count_ls + (((uint16_t) (fifo_status & 0x03)) << 8);

    // Read the data from the FIFO:
    imu_command(CTRL_CMD_REQ_FIFO);

    counter++;
}

/**
 * This handles INT1 events, which indicates one of two things:
 * - IMU device reset.
 * - a ctrl9 command has completed.
 */
static void imu_int1_handler()
{
    // Bit 0 should be set at this point; reading it clears it and the interrupt:
    unsigned char status1;
    QMI8658_read_reg(QMI8658Register_Status1, &status1, 1);

    volatile int i = status1;
    
    switch (pending_imu_command)
    {
        case NO_COMMAND:
            // No command pending. Nothing to do.
            i = 1;
            break;

        case CTRL_CMD_RST_FIFO:
            // The FIFO has been reset.
            i = 2;
            break;

        case CTRL_CMD_REQ_FIFO:
            // Disable address auto increment:
            QMI8658_write_reg(QMI8658Register_Ctrl1, 0x20); 

            // See if the FIFO_rd_mode bit is 1 (it should be):
            unsigned char FifoCtrl;
            QMI8658_read_reg(FIFO_CTRL, &FifoCtrl, 1);

            // Ack the command
            QMI8658_write_reg(QMI8658Register_Ctrl9, CTRL_CMD_ACK);

            // Read the data. In one test, I found 102 bytes ( = 6 * 17, and HWM was 16), byt looking at the values.
            unsigned char data_buf[FIFO_MAX_SAMPLES * 6];
            QMI8658_read_reg(FIFO_DATA, data_buf, sizeof(data_buf));    // Really we should read the number of samples.

            // Clear the FIFO_rd_mode bit:
            QMI8658_write_reg(FIFO_CTRL, FIFO_CTRL_VALUE);

            break;

        default:
            i = 3;
            break;        
    }

    pending_imu_command = NO_COMMAND;
}

/** Interrupt event handler.
 */
void gpio_irq_dispatcher(uint gpio, uint32_t events)
{
    switch (gpio)
    {
        case IMU_INT1_PIN:
            imu_int1_handler();
            break;    
        case IMU_INT2_PIN:
            imu_int2_handler();
            break;    
    }
}


static void imu_command(imu_commands cmd)
{
    pending_imu_command = cmd;
    QMI8658_write_reg(QMI8658Register_Ctrl9, cmd);
}

static void imu_command_spinwait()
{
    // Spin lock waiting for the command to complete via INT1:
    while (pending_imu_command != NO_COMMAND)
        ;
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

    // ***************** TODO: extract the following initialization code to a function.

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

    // Set ourselves up to get the interrupts: callback, gpio config and IRQ bank:
    gpio_set_irq_callback(&gpio_irq_dispatcher);
    gpio_set_irq_enabled(IMU_INT1_PIN, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(IMU_INT2_PIN, GPIO_IRQ_EDGE_RISE, true);
    irq_set_enabled(IO_IRQ_BANK0, true);

    // Enable the accelerometer input:
    QMI8658_enableSensors(QMI8658_CONFIG_ACC_ENABLE);

    // Enable FIFO with a HWM:
    QMI8658_write_reg(FIFO_CTRL, 0x0D);                         // 0000 1101    FIFO mode, 128 samples size.
    QMI8658_write_reg(FIFO_CTRL, FIFO_CTRL_VALUE);
    QMI8658_write_reg(FIFO_WTM_TH, FIFO_HIGH_WATER_THRESHOLD);  // High water mark for generating interrupt INT2.

    DEV_Delay_ms(3000);

    // Reset the FIFO:
    // imu_command(CTRL_CMD_RST_FIFO);
    // imu_command_spinwait();

    DEV_Delay_ms(3000);

    // ***************** 

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
