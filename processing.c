#include "processing.h"
#include "imu.h"
#include "biquad.h"


// Globals:
xyz_sample raw_data_buf[FIFO_MAX_SAMPLES];
volatile uint32_t imu_zc_count = 0;

// Statics:
static int16_t raw_accel_data[FIFO_MAX_SAMPLES];
static int16_t lpf_accel_data[FIFO_MAX_SAMPLES];
static int16_t circular_data[FIFO_MAX_SAMPLES * 10];

#define DEBUG_DATA_LEN 256
static int16_t debug_data[DEBUG_DATA_LEN];
static int debug_index = 0;     // Next free data location index.


static int n_negative, n_positive;              // Used for zero crossing detection.

#define BIQUAD_STAGES 2
static arm_biquad_casd_df1_inst_q15 lpf_instance;       // Low pass filter biquad instance.
static q15_t lpf_state[BIQUAD_STAGES * 4];
#define SCALING 2.0                                     // Adjust this value to accommodate the largest coefficient below.
#define NORMALIZED(v) (v * 0x7FFF / SCALING)            // Scale v to a q15_t
static const q15_t lpf_coefficients[] =
{
    // https://www.earlevel.com/main/2021/09/02/biquad-calculator-v3/
    // CMSIS b10, 0, b11, b12, a11, a12. From the calculator above, swap A and B, and change the signs of the resulting As.

    // 1000 Hz sample rate, 5Hz cutoff, Q=0.7071, high pass.
    // This removes DC (ie gravity).
    NORMALIZED(0.9780302754084559),
    0,
    NORMALIZED(-1.9560605508169118),
    NORMALIZED(0.9780302754084559),
    NORMALIZED(1.9555778328194147),
    NORMALIZED(-0.9565432688144089),

    // 1000 Hz sample rate, 35Hz cutoff, Q=0.7071, low pass.
    // Use a low cutoff to get maximum slope in the region of the note.
    NORMALIZED(0.010432400000959617),
    0,
    NORMALIZED(0.020864800001919235),
    NORMALIZED(0.010432400000959617),
    NORMALIZED(1.6909942097141335),
    NORMALIZED(-0.7327238097179718)

#if 0
    // 1000 Hz sample rate, 70Hz cutoff, Q=0.7071, low pass.
    ,
    NORMALIZED(0.036574754677828704),
    0,
    NORMALIZED(0.07314950935565741),
    NORMALIZED(0.036574754677828704),
    NORMALIZED(1.3908921947801067),
    NORMALIZED(-0.5371912134914214),
#endif
};


static void add_to_debug_data(int16_t *pData, int count)
{
    // Append the data supplied to the debug data buffer, wrapping as required.

    count = MIN(count, DEBUG_DATA_LEN);     // Paranoia.
    
    // Potentially we copy in two parts, to allow for wrapping.

    int part1_count = MIN(count, DEBUG_DATA_LEN - debug_index);
    int16_t *limit = debug_data + debug_index + part1_count;
    for (int16_t *pDest = debug_data + debug_index; pDest < limit; pDest++, pData++)
        *pDest = *pData;
    debug_index += part1_count;
    if (debug_index == DEBUG_DATA_LEN)
        debug_index = 0;    // Wrap if required.

    if (count > part1_count) {
        // We only get here if we have just wrapped, so some assumptions can be made.
        int part2_count = count - part1_count;
        limit = debug_data + part2_count;
        for (int16_t *pDest = debug_data; pDest < limit; pDest++, pData++)
            *pDest = *pData;
        debug_index += part2_count;
    }

}

void processing_initialize(void)
{
    // Prepare the biquad filter:
    arm_biquad_cascade_df1_init_q15(&lpf_instance, BIQUAD_STAGES, lpf_coefficients, lpf_state, 0);

    n_negative = 0;
    n_positive = 0;
    imu_zc_count = 0;
    debug_index = 0;
}

/** 
 * Process newly arrived raw data in raw_data_buf.
*/
void processing_process(int accel_samples)
{
    // Extract the single acceleration reading we are interested in from the (x,y,z)
    // values arriving from the IMU:
    xyz_sample *pxyz = raw_data_buf;
    for (int i = 0; i < accel_samples; i++, pxyz++)
        raw_accel_data[i] = pxyz->accel_x;

    // Apply a low pass filter to the data:
    arm_biquad_cascade_df1_q15(&lpf_instance, raw_accel_data, lpf_accel_data, accel_samples);

    // Zero crossing detector:
    for (int i = 0; i < accel_samples; i++)
    {
        // TODO: we should also check that the negative portion before the position portion had at least n points.
        int16_t v = lpf_accel_data[i];
        if (v < 0) {
            n_negative++;
            n_positive = 0;
        }
        else {
            n_positive++;
            n_negative = 0;
        }
        if (n_positive >= 3) {
            // Yes, this seems to be rising edge passing through zero.
            imu_zc_count++;
        }
    }

    add_to_debug_data(lpf_accel_data, accel_samples);
}
