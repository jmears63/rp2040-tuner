#ifndef PROCESSING_H
#define PROCESSING_H

#include <stdint.h>

/**
 * Data structure to match the data read from the IMU.
*/
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
} __attribute__((packed)) xyz_sample;


extern volatile uint32_t imu_zc_count;
extern xyz_sample raw_data_buf[];

void processing_initialize(void);
void processing_process(int accel_samples);

#endif // PROCESSING_H