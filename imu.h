#ifndef IMU_H
#define IMU_H

#include "QMI8658.h"

typedef enum {
    CTRL_CMD_ACK = 0x00,                // Used to acknowledge the command when it is handled in INT1.
    CTRL_CMD_RST_FIFO = 0x04,
    CTRL_CMD_REQ_FIFO = 0x05,           // The data sheet confusingly also mentions 0x0D for this command.
                                        // 5 seems to work and is what is in the QMU8658A data sheet.
    NO_COMMAND = -1                     // Not a command.
} imu_commands;

#define FIFO_MAX_SAMPLES 256                // Maximum FIFO length based on the IMU data sheet.

void imu_initialize(void);
void imu_command(imu_commands cmd);
void imu_command_spinwait(void);

#endif // IMU_H