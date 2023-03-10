#include "imu.h"
#include "processing.h"

/*
    IMPORTANT
    
    The IMU chip behaves like a QMI865A (not a QMI865C as specified by Waveshare), documented here:
        https://www.qstcorp.com/upload/pdf/202301/13-52-25%20QMI8658A%20Datasheet%20Rev%20A.pdf
    The protocol for reading FIFO data is somewhat different, and works.
*/


// The IMU interrupts connection to these GPIO pins:
#define IMU_INT2_PIN 24
#define IMU_INT1_PIN 23

// Register addresses for FIFO according to the QMU8658A data sheet, different from the
// sample code:
enum FIFO_REGISTERS {
    FIFO_WTM_TH = 19,
    FIFO_CTRL = 20,
    FIFO_SMPL_CNT_LS = 21,
    FIFO_STATUS = 22,
    FIFO_DATA = 23
};

#define FIFO_HIGH_WATER_THRESHOLD 16        // The FIFO High Water Threshold in ODRs (so per 3 WORDs or xyz_sample structs).
#define FIFO_CTRL_VALUE 0x0D                // 0000 1101:  FIFO mode, 128 samples (3 WORDS each, xyz_sample) total FIFO size.

#define RAW_SAMPLE_RATE_HZ 1000

static volatile imu_commands pending_imu_command = NO_COMMAND;

/**
 * This handles INT2 events, which generally indicate that data is ready
 * to be read from the IMU.
 */
static void imu_int2_handler()
{
    // Issue a command to read the data from the FIFO. That will result in an INT1.
    imu_command(CTRL_CMD_REQ_FIFO);
}


/**
 * This handles INT1 events, which indicates one of two things:
 * - IMU device reset.
 * - a ctrl9 command has completed.
 */
static void imu_int1_handler()
{
    switch (pending_imu_command)
    {
        case CTRL_CMD_REQ_FIFO:

            // How much data is waiting in the FIFO?
            unsigned char sample_count_ls, fifo_status;
            QMI8658_read_reg(FIFO_SMPL_CNT_LS, &sample_count_ls, 1);
            QMI8658_read_reg(FIFO_STATUS, &fifo_status, 1);

            // This is the sample count in WORDs:
            uint16_t sample_count_words = (uint16_t) sample_count_ls + (((uint16_t) (fifo_status & 0x03)) << 8);
            sample_count_words = MIN(FIFO_MAX_SAMPLES, sample_count_words);

            // Out of curiosity, see if the FIFO_rd_mode bit is 1 (it should be):
            unsigned char FifoCtrl;
            QMI8658_read_reg(FIFO_CTRL, &FifoCtrl, 1);

            // Ack the command:
            QMI8658_write_reg(QMI8658Register_Ctrl9, CTRL_CMD_ACK);

            // Read the data. If we over read, the undefined values return as -1.
            uint16_t bytes_to_read = sample_count_words * sizeof(xyz_sample);
            QMI8658_read_reg(FIFO_DATA, (unsigned char *) raw_data_buf, bytes_to_read); 

            // Clear the FIFO_rd_mode bit so that the IMU can start putting new readins into the FIFO.
            // If we do this too late, we can miss data.
            QMI8658_write_reg(FIFO_CTRL, FIFO_CTRL_VALUE);

            processing_process(sample_count_words / 3);

            break;

        case CTRL_CMD_RST_FIFO: 
            // Ack the command so that things can continue:
            QMI8658_write_reg(QMI8658Register_Ctrl9, CTRL_CMD_ACK);
            break;

        default:
            break;        
    }

    pending_imu_command = NO_COMMAND;
}


/** 
 * Interrupt event handler.
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


void imu_initialize(void)
{
    processing_initialize();

    QMI8658_init();

    struct QMI8658Config QMI8658_config;
    QMI8658_config.inputSelection = QMI8658_CTRL7_DISABLE_ALL;  // Initially, no inputs enabled.
    QMI8658_config.accRange = QMI8658AccRange_8g;
    QMI8658_config.accOdr = QMI8658AccOdr_1000Hz;               // Must match value of RAW_SAMPLE_RATE_HZ above.
    QMI8658_Config_apply(&QMI8658_config);

    // All inputs are disabled while we set things up.

    // Set ourselves up to get the interrupts: callback, gpio config and IRQ bank:
    gpio_set_irq_callback(&gpio_irq_dispatcher);
    gpio_set_irq_enabled(IMU_INT1_PIN, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(IMU_INT2_PIN, GPIO_IRQ_EDGE_RISE, true);
    irq_set_enabled(IO_IRQ_BANK0, true);

    // Disable address auto increment so we can read the data from the FIFO_DATA: 
    QMI8658_write_reg(QMI8658Register_Ctrl1, 0x20); 

    // Enable FIFO with a HWM:
    QMI8658_write_reg(FIFO_CTRL, 0x0D);                         // 0000 1101    FIFO mode, 128 samples size.
    QMI8658_write_reg(FIFO_CTRL, FIFO_CTRL_VALUE);
    QMI8658_write_reg(FIFO_WTM_TH, FIFO_HIGH_WATER_THRESHOLD);  // High water mark for generating interrupt INT2.

    // Reset the FIFO (me neither):
    imu_command(CTRL_CMD_RST_FIFO);

    // Enable the accelerometer input:
    QMI8658_enableSensors(QMI8658_CONFIG_ACC_ENABLE);
}

void imu_command(imu_commands cmd)
{
    pending_imu_command = cmd;
    QMI8658_write_reg(QMI8658Register_Ctrl9, cmd);
}


void imu_command_spinwait(void)
{
    // Spin lock waiting for the command to complete via INT1:
    while (pending_imu_command != NO_COMMAND)
        ;
}

