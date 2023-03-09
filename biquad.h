#ifndef BIQUAD_H
#define BIQUAD_H

#include <stdint.h>

/*
 * This code is from CMSIS: Copyright ARM, Apache license.
 */

typedef int16_t q15_t;
typedef int32_t q31_t;
typedef int64_t q63_t;


/**
   * @brief Instance structure for the Q15 Biquad cascade filter.
   */
  typedef struct
  {
          int8_t numStages;        /**< number of 2nd order stages in the filter.  Overall order is 2*numStages. */
          q15_t *pState;           /**< Points to the array of state coefficients.  The array is of length 4*numStages. */
    const q15_t *pCoeffs;          /**< Points to the array of coefficients.  The array is of length 5*numStages. */
          int8_t postShift;        /**< Additional shift, in bits, applied to each output sample. */
  } arm_biquad_casd_df1_inst_q15;

void arm_biquad_cascade_df1_init_q15(
        arm_biquad_casd_df1_inst_q15 * S,
        uint8_t numStages,
  const q15_t * pCoeffs,
        q15_t * pState,
        int8_t postShift);

void arm_biquad_cascade_df1_q15(
  const arm_biquad_casd_df1_inst_q15 * S,
  const q15_t * pSrc,
        q15_t * pDst,
        uint32_t blockSize);



#endif // BIQUAD_H