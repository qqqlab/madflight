/** @file linalg.c
 * KFCore
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * @brief Math functions
 * @{ */

/******************************************************************************
 * SYSTEM INCLUDE FILES
 ******************************************************************************/

#include <assert.h>
#include <math.h>
#include <string.h> /* memset */

/******************************************************************************
 * PROJECT INCLUDE FILES
 ******************************************************************************/

#include "linalg.h"
#include "miniblas.h"

/******************************************************************************
 * DEFINES
 ******************************************************************************/

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

void matmul(const char* ta, const char* tb, int n, int k, int m, float alpha, const float* A,
            const float* B, float beta, float* C)
{
    int lda    = lsame_(ta, "T") ? m : n;
    int ldb    = lsame_(tb, "T") ? k : m;
    int result = sgemm_(ta, tb, &n, &k, &m, &alpha, (float*)A, &lda, (float*)B, &ldb,
                       &beta, C, &n);
    assert(result == 0);
}

void matmulsym(const float* A_sym, const float* B, int n, int m, float* C)
{
    float alpha  = 1.0f;
    float beta   = 0.0f;
    int   result = ssymm_("L" /* calculate C = A*B not C = B*A */,
                         "U" /* reference upper triangular part of A */, &n, /* rows of B/C */
                         &m,                                                 /* cols of B / C */
                         &alpha, (float*)A_sym, &n, (float*)B, &n, &beta, C, &n);
    assert(result == 0);
}

void mateye(float* A, int n)
{
    memset(A, 0, sizeof(float) * (size_t)n * (size_t)n);
    for (int i = 0; i < n; i++)
    {
        MAT_ELEM(A, i, i, n, n) = 1.0f;
    }
}

int cholesky(float* A, const int n, int onlyWriteLowerPart)
{
    /* in-place calculation of lower triangular matrix L*L' = A */

    /* set the upper triangular part to zero? */
    if (!onlyWriteLowerPart)
    {
        for (int i = 0; i < n - 1; i++) /* row */
        {
            for (int j = i + 1; j < n; j++) /* col */
            {
                MAT_ELEM(A, i, j, n, n) = 0.0f;
            }
        }
    }

    for (int j = 0; j < n; j++) /* main loop */
    {
        const float Ajj = MAT_ELEM(A, j, j, n, n);
        if (Ajj <= 0.0f || !isfinite(Ajj))
        {
            return -1;
        }
        MAT_ELEM(A, j, j, n, n) = SQRTF(Ajj);

        const float invLjj = 1.0f / MAT_ELEM(A, j, j, n, n);
        for (int i = j + 1; i < n; i++)
        {
            MAT_ELEM(A, i, j, n, n) *= invLjj;
        }

        for (int k = j + 1; k < n; k++)
        {
            for (int i = k; i < n; i++)
            {
                MAT_ELEM(A, i, k, n, n) -= MAT_ELEM(A, i, j, n, n) * MAT_ELEM(A, k, j, n, n);
            }
        }
    }
    return 0;
}

void trisolve(const float* A, float* B, int n, int m, const char* tp)
{
    float     alpha = 1.0f;
    const int result =
        strsm_("L" /* left hand*/, "L" /* lower triangular matrix */, tp /* transpose L? */,
              "N" /* L is not unit triangular */, &n, &m, &alpha, A, &n, B, &n);
    assert(result == 0);
    /* strsm only checks matrix dimensions, handled via assert */
}

void trisolveright(const float* L, float* A, int n, int m, const char* tp)
{
    float     alpha = 1.0f;
    const int result =
        strsm_("R" /* right hand*/, "L" /* lower triangular matrix */, tp /* transpose L? */,
              "N" /* L is not unit triangular */, &m, &n, &alpha, L, &n, A, &m);
    assert(result == 0);
    /* strsm only checks matrix dimensions, handled via assert */
}

void symmetricrankupdate(float* P, const float* E, int n, int m)
{
    float alpha = -1.0f;
    float beta  = 1.0f;

    const int result = ssyrk_("U", "N", &n, &m, &alpha, (float*)E, &n, &beta, P, &n);
    assert(result == 0);
}

int udu(const float* A, float* U, float* d, const int m)
{
    /*    A = U*diag(d)*U' decomposition
     *    Source:
     *      1. Golub, Gene H., and Charles F. Van Loan. "Matrix Computations." 4rd ed.,
     *         Johns Hopkins University Press, 2013.
     *      2. Grewal, Weill, Andrews. "Global positioning systems, inertial
     *         navigation, and integration". 1st ed. John Wiley & Sons, New York, 2001.
     *
     *    function [U, d] = udu(M)
     *      [m, ~] = size(M);
     *      U = zeros(m, m); d = zeros(m, 1);
     *
     *      for j = m:-1:1
     *        for i = j:-1:1
     *          sigma = M(i, j);
     *          for k = j + 1:m
     *              sigma = sigma - U(i, k) * d(k) * U(j, k);
     *          end
     *          if i == j
     *              d(j) = sigma;
     *              U(j, j) = 1; % U is a unit triangular matrix
     *          else
     *              U(i, j) = sigma / d(j); % off-diagonal elements of U
     *          end
     *        end
     *      end
     *    end
     */
    int   i, j, k;
    float sigma;

    memset(U, 0, sizeof(U[0]) * (size_t)m * (size_t)m);
    memset(d, 0, sizeof(d[0]) * (size_t)m);

    for (j = m - 1; j >= 0; j--) /* UDU decomposition */
    {
        for (i = j; i >= 0; i--)
        {
            sigma = MAT_ELEM(A, i, j, m, m);
            for (k = j + 1; k < m; k++)
            {
                sigma -= MAT_ELEM(U, i, k, m, m) * d[k] * MAT_ELEM(U, j, k, m, m);
            }
            if (i == j)
            {
                d[j]                    = sigma;
                MAT_ELEM(U, j, j, m, m) = 1.0f;
            }
            else
            {
                if ((d[j] <= 0.0f) || !isfinite(d[j]))
                {
                    /* matrix is not positive definite if d < 0 */
                    return -1;
                }
                MAT_ELEM(U, i, j, m, m) = sigma / d[j];
            }
        }
    }
    return 0;
}
