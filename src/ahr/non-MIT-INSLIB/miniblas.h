/** @file miniblas.h
 * KFCore
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * @brief Minimal generic BLAS implementation
 *
 * Note: all matrices are stored in column-major order.
 *
 * @{ */

/******************************************************************************
 * SYSTEM INCLUDE FILES
 ******************************************************************************/

/******************************************************************************
 * PROJECT INCLUDE FILES
 ******************************************************************************/

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
 * FUNCTION PROTOTYPES
 ******************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

    int lsame_(const char* a, const char* b);

    int strsm_(const char* side, const char* uplo, const char* transa, const char* diag, int* m,
              int* n, float* alpha, const float* a, int* lda, float* b, int* ldb);

    int sgemm_(const char* transa, const char* transb, int* m, int* n, int* k, float* alpha,
              float* a, int* lda, float* b, int* ldb, float* beta, float* c, int* ldc);

    int ssyrk_(const char* uplo, const char* trans, int* n, int* k, float* alpha, float* a,
              int* lda, float* beta, float* c, int* ldc);

    int ssymm_(const char* side, const char* uplo, int* m, int* n, float* alpha, float* a,
              int* lda, float* b, int* ldb, float* beta, float* c, int* ldc);

    int strmm_(const char* side, const char* uplo, const char* transa, const char* diag, int* m,
               int* n, float* alpha, float* a, int* lda, float* b, int* ldb);

#ifdef __cplusplus
}
#endif

/* @} */
