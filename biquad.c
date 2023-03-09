#include "biquad.h"

#include <memory.h>


void arm_biquad_cascade_df1_init_q15(
        arm_biquad_casd_df1_inst_q15 * S,
        uint8_t numStages,
  const q15_t * pCoeffs,
        q15_t * pState,
        int8_t postShift)
{
  /* Assign filter stages */
  S->numStages = numStages;

  /* Assign postShift to be applied to the output */
  S->postShift = postShift;

  /* Assign coefficient pointer */
  S->pCoeffs = pCoeffs;

  /* Clear state buffer and size is always 4 * numStages */
  memset(pState, 0, (4U * (uint32_t) numStages) * sizeof(q15_t));

  /* Assign state pointer */
  S->pState = pState;
}


static inline int32_t __SSAT(int32_t val, uint32_t sat)
  {
    if ((sat >= 1U) && (sat <= 32U))
    {
      const int32_t max = (int32_t)((1U << (sat - 1U)) - 1U);
      const int32_t min = -1 - max ;
      if (val > max)
      {
        return max;
      }
      else if (val < min)
      {
        return min;
      }
    }
    return val;
  }


void arm_biquad_cascade_df1_q15(
  const arm_biquad_casd_df1_inst_q15 * S,
  const q15_t * pSrc,
        q15_t * pDst,
        uint32_t blockSize)
{
  const q15_t *pIn = pSrc;                             /* Source pointer */
        q15_t *pOut = pDst;                            /* Destination pointer */
        q15_t b0, b1, b2, a1, a2;                      /* Filter coefficients */
        q15_t Xn1, Xn2, Yn1, Yn2;                      /* Filter state variables */
        q15_t Xn;                                      /* temporary input */
        q63_t acc;                                     /* Accumulator */
        int32_t shift = (15 - (int32_t) S->postShift); /* Post shift */
        q15_t *pState = S->pState;                     /* State pointer */
  const q15_t *pCoeffs = S->pCoeffs;                   /* Coefficient pointer */
        uint32_t sample, stage = (uint32_t) S->numStages;     /* Stage loop counter */

  do
  {
    /* Reading the coefficients */
    b0 = *pCoeffs++;
    pCoeffs++;  // skip the 0 coefficient
    b1 = *pCoeffs++;
    b2 = *pCoeffs++;
    a1 = *pCoeffs++;
    a2 = *pCoeffs++;

    /* Reading the state values */
    Xn1 = pState[0];
    Xn2 = pState[1];
    Yn1 = pState[2];
    Yn2 = pState[3];

    /* The variables acc holds the output value that is computed:
     *    acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2]
     */

    sample = blockSize;

    while (sample > 0U)
    {
      /* Read the input */
      Xn = *pIn++;

      /* acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2] */
      /* acc =  b0 * x[n] */
      acc = (q31_t) b0 *Xn;

      /* acc +=  b1 * x[n-1] */
      acc += (q31_t) b1 *Xn1;
      /* acc +=  b[2] * x[n-2] */
      acc += (q31_t) b2 *Xn2;
      /* acc +=  a1 * y[n-1] */
      acc += (q31_t) a1 *Yn1;
      /* acc +=  a2 * y[n-2] */
      acc += (q31_t) a2 *Yn2;

      /* The result is converted to 1.31  */
      acc = __SSAT((acc >> shift), 16);

      /* Every time after the output is computed state should be updated. */
      /* The states should be updated as:  */
      /* Xn2 = Xn1 */
      /* Xn1 = Xn  */
      /* Yn2 = Yn1 */
      /* Yn1 = acc */
      Xn2 = Xn1;
      Xn1 = Xn;
      Yn2 = Yn1;
      Yn1 = (q15_t) acc;

      /* Store the output in the destination buffer. */
      *pOut++ = (q15_t) acc;

      /* decrement the loop counter */
      sample--;
    }

    /*  The first stage goes from the input buffer to the output buffer. */
    /*  Subsequent stages occur in-place in the output buffer */
    pIn = pDst;

    /* Reset to destination pointer */
    pOut = pDst;

    /*  Store the updated state variables back into the pState array */
    *pState++ = Xn1;
    *pState++ = Xn2;
    *pState++ = Yn1;
    *pState++ = Yn2;

  } while (--stage);
}