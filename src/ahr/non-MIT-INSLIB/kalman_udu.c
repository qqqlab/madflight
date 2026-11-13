/** @file kalman_udu.c
 * KFCore
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * @brief UDU Kalman Filter
 *
 * Constraints
 *  - All pointer arguments shall not overlap.
 *  - R in kalman_udu() must be diagonal (call decorrelate() if needed)
 * @{ */

/******************************************************************************
 * SYSTEM INCLUDE FILES
 ******************************************************************************/

#include <math.h>
#include <assert.h>
#include <string.h> /* memcpy */

/******************************************************************************
 * PROJECT INCLUDE FILES
 ******************************************************************************/

#include "linalg.h"
#include "miniblas.h" /* strmm_ */
#include "kalman_udu.h"

/******************************************************************************
 * DEFINES
 ******************************************************************************/

#ifndef KALMAN_MAX_STATE_SIZE
#define KALMAN_MAX_STATE_SIZE 32 /* kalman filter scratchpad buf size */
#endif

#ifndef KALMAN_MAX_NOISE_SIZE
/* max. columns r of noise matrix G (n x r). */
#define KALMAN_MAX_NOISE_SIZE KALMAN_MAX_STATE_SIZE
#endif

/* Limit for division (alpha, d[i], s). */
#define KALMAN_UDU_EPS (1e-30f)

/******************************************************************************
 * TYPEDEFS
 ******************************************************************************/

/******************************************************************************
 * LOCAL DATA DEFINITIONS
 ******************************************************************************/

/******************************************************************************
 * LOCAL FUNCTION PROTOTYPES
 ******************************************************************************/

/******************************************************************************
 * FUNCTION BODIES
 ******************************************************************************/

int kalman_udu_scalar(float* restrict x, float* restrict U, float* restrict d,
                      const float dz, const float R,
                      const float* restrict H_line, int n)
{
    assert(n > 0 && n <= KALMAN_MAX_STATE_SIZE);
    assert((const float*)U != H_line && x != H_line && d != H_line);

    float a[KALMAN_MAX_STATE_SIZE];
    float b[KALMAN_MAX_STATE_SIZE];

    if (!(R > 0.0f)) /* also capture NaN */
    {
        return -1;
    }

    {
        // calculate: a = U'*H'
        int   tmpone   = 1;
        float tmpalpha = 1.0f;
        memcpy(a, H_line, sizeof(a[0]) * (size_t)n); // preload with H_line
        strmm_("L", "U", "T", "U", &n, &tmpone, &tmpalpha, U, &n, a, &n);
    }

    for (int j = 0; j < n; j++)
    {
        b[j] = d[j] * a[j]; // b = D*a = diag(d)*a
    }

    /* Health check */
    {
        float alpha_chk = R;
        for (int j = 0; j < n; j++)
        {
            alpha_chk += a[j] * b[j];
            if (!(alpha_chk > KALMAN_UDU_EPS))
            {
                return -1; /* U-D update not possible */
            }
        }
    }

    float alpha = R;
    float gamma = 1.0f / alpha;

    for (int j = 0; j < n; j++)
    {
        const float beta_j = alpha;
        alpha += a[j] * b[j];
        const float lambda = -a[j] * gamma;

        gamma = 1.0f / alpha; /* alpha > 0 due to health check */

        d[j] *= beta_j * gamma;

        const float bj = b[j];
        for (int i = 0; i < j; i++)
        {
            const float uij = MAT_ELEM(U, i, j, n, n);

            MAT_ELEM(U, i, j, n, n) = uij + b[i] * lambda;
            b[i] += bj * uij;
        }
    }

    const float k = gamma * dz;
    for (int j = 0; j < n; j++)
    {
        x[j] += k * b[j];
    }

    return 0;
}

int kalman_udu(float* restrict x, float* restrict U, float* restrict d,
               const float* restrict z, const float* restrict R,
               const float* restrict Ht, int n, int m,
               float chi2_threshold, int downweight_outlier)
{
    assert(n > 0 && n <= KALMAN_MAX_STATE_SIZE);

    int retcode = 0;

    for (int i = 0; i < m; i++, Ht += n) /* iterate over each measurement,
                                            goto next line of H after each iteration */
    {
        float Rv = MAT_ELEM(R, i, i, m, m); /* get scalar measurement variance */
        float dz = z[i];                    /* calculate residual for current scalar measurement */
        // matmul("N", "N", 1, 1, n, -1.0f, Ht, x, 1.0f, &dz); // dz = z - H(i,:)*x
        // inline for-loop dot product to avoid matmul call
        for (int j = 0; j < n; j++) { dz -= Ht[j] * x[j]; }

        // <robust>
        if (chi2_threshold > 0.0f)
        {
            float tmp[KALMAN_MAX_STATE_SIZE];
            float s; // for chi2 test: s = H*U*diag(d)*U'*H' + R
                     // Chang, G. (2014). Robust Kalman filtering based on
                     // Mahalanobis distance as outlier judging criterion.
                     // Journal of Geodesy, 88(4), 391-401.

            float HPHT = 0.0f; // calc. scalar result of H_line*U*diag(d)*U'*H_line'
            // matmul("N", "N", 1, n, n, 1.0f, Ht, U, 0.0f, tmp);
            // tmp = H(i,:) * U
            // nxn product (at this size) is cheaper as a direct loop
            // with column-major than a SGEMM call.
            for (int k = 0; k < n; k++)
            {
                float acc = 0.0f;
                for (int j = 0; j < n; j++) { acc += Ht[j] * MAT_ELEM(U, j, k, n, n); }
                tmp[k] = acc;
            }
            for (int j = 0; j < n; j++)
            {
                HPHT += tmp[j] * tmp[j] * d[j];
            }
            s = HPHT + Rv;

            if (!(s > KALMAN_UDU_EPS)) /* is innovation broken? */
            {
                retcode = -1;
                continue; /* skip measurement */
            }

            const float mahalanobis_dist_sq = dz * dz / s;
            if (mahalanobis_dist_sq > chi2_threshold) // potential outlier?
            {
                if (!downweight_outlier)
                {
                    continue; /* skip measurement */
                }
                /* process this measurement, but reduce the measurement precision */
                const float f = mahalanobis_dist_sq / chi2_threshold;
                Rv            = (f - 1.0f) * HPHT + f * Rv;
            }
        }
        // </robust>

        int status = kalman_udu_scalar(x, U, d, dz, Rv, Ht, n);
        if (status != 0)
        {
            retcode = -1; /* still process rest of the measurement vector */
        }
    }
    return retcode;
}

int decorrelate(float* restrict z, float* restrict Ht, float* restrict R,
                int n, int m)
{
    /* Basic decorrelation in MATLAB
    [G] = chol(R); % G'*G = R
    zdecorr = (G')\z;
    Hdecorr = (G')\H;
    Rdecorr = eye(length(z)); */

    // in-place cholesky so that L*L' = R:
    int result = cholesky(R, m, 0 /* 0 means: fill upper part with zeros */);
    if (result != 0)
    {
        return -1;
    }
    // L*H_decorr = H
    // (L*H_decorr)' = H'
    // H_decorr'*L' = H' solve for H_decorr
    trisolveright(R /*L*/, Ht, m, n, "T");
    trisolve(R /*L*/, z, m, 1, "N");

    return 0;
}

int kalman_udu_predict(float* restrict x, float* restrict U, float* restrict d,
                       const float* restrict Phi, const float* restrict G,
                       const float* restrict Q, int n, int r)
{
    assert(n > 0 && n <= KALMAN_MAX_STATE_SIZE);
    assert(r >= 0 && r <= KALMAN_MAX_NOISE_SIZE);
    /* restrict-contract: check for some aliasing violations */
    assert(Phi != (const float*)U && G != (const float*)U);
    assert(Q != (const float*)d);

    int retcode = 0;

    if (x) //  if prediction of state vector is requested: x = Phi*x;
    {
        float tmp[KALMAN_MAX_STATE_SIZE];
        memcpy(tmp, x, sizeof(x[0]) * (size_t)n);
        matmul("N", "N", n, 1, n, 1.0f, Phi, tmp, 0.0f, x);
    }

    // G_tmp = G; // move to internal array for destructive updates
    float G_tmp[KALMAN_MAX_STATE_SIZE * KALMAN_MAX_NOISE_SIZE];
    memcpy(G_tmp, G, sizeof(G_tmp[0]) * (size_t)n * (size_t)r);

    // PhiU  = Phi*U; // rows of [PhiU,G] are to be orthogonalized
    float PhiU[KALMAN_MAX_STATE_SIZE * KALMAN_MAX_STATE_SIZE];
    float tmpalpha = 1.0f;
    memcpy(PhiU, Phi, sizeof(Phi[0]) * (size_t)n * (size_t)n);
    strmm_("R", "U", "N", "U", &n, &n, &tmpalpha, U, &n, PhiU, &n);

    mateye(U, n); // U = eye(n)

    // save origin input d vector
    float din[KALMAN_MAX_STATE_SIZE];
    memcpy(din, d, sizeof(d[0]) * (size_t)n); // din = d

    for (int i = n - 1; i >= 0; i--)
    {
        // d[i] is the weighted norm of row i of [PhiU, G]: BOTH sums must
        // run over the full column count of their matrix (n for PhiU, r
        // for G).
        float sigma = 0.0f;
        for (int j = 0; j < n; j++)
        {
            sigma += MAT_ELEM(PhiU, i, j, n, n) *
                     MAT_ELEM(PhiU, i, j, n, n) * din[j];
        }
        for (int j = 0; j < r; j++)
        {
            sigma += MAT_ELEM(G_tmp, i, j, n, r) *
                     MAT_ELEM(G_tmp, i, j, n, r) * Q[j];
        }
        d[i] = sigma;

        if (!(d[i] > KALMAN_UDU_EPS))
        {
            d[i] = 0.0f;
            for (int j = 0; j < i; j++)
            {
                MAT_ELEM(U, j, i, n, n) = 0.0f;
            }
            retcode = -1; /* P is singular */
            continue;
        }

        const float dinv = 1.0f / d[i];

        for (int j = 0; j < i; j++)
        {
            sigma = 0.0f;
            for (int k = 0; k < n; k++)
            {
                sigma += MAT_ELEM(PhiU, i, k, n, n) * din[k] *
                         MAT_ELEM(PhiU, j, k, n, n);
            }
            for (int k = 0; k < r; k++)
            {
                sigma += MAT_ELEM(G_tmp, i, k, n, r) *
                         Q[k] *
                         MAT_ELEM(G_tmp, j, k, n, r);
            }

            const float uji = sigma * dinv;
            MAT_ELEM(U, j, i, n, n) = uji;

            for (int k = 0; k < n; k++)
            {
                MAT_ELEM(PhiU, j, k, n, n) -= uji * MAT_ELEM(PhiU, i, k, n, n);
            }
            for (int k = 0; k < r; k++)
            {
                MAT_ELEM(G_tmp, j, k, n, r) -= uji * MAT_ELEM(G_tmp, i, k, n, r);
            }
        }
    }

    return retcode;
}

/* @} */
