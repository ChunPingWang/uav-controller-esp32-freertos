/**
 * @file matrix_math.h
 * @brief Lightweight 6x6 matrix operations for EKF
 *
 * Custom implementation avoiding external dependencies.
 * Optimized for ESP32 (no dynamic allocation per Constitution III).
 */

#ifndef MATRIX_MATH_H
#define MATRIX_MATH_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Matrix Dimensions
 * ========================================================================== */

#define MAT_DIM     6       /**< Matrix dimension for EKF */
#define MAT_SIZE    36      /**< Total elements (6x6) */

/* ==========================================================================
 * Matrix Type
 *
 * Matrices are stored as 2D arrays for clarity.
 * All operations use static allocation.
 * ========================================================================== */

typedef float mat6x6_t[MAT_DIM][MAT_DIM];
typedef float vec6_t[MAT_DIM];

/* ==========================================================================
 * Basic Matrix Operations
 * ========================================================================== */

/**
 * @brief Set matrix to zero
 *
 * @param m Matrix to zero
 */
void mat6_zero(mat6x6_t m);

/**
 * @brief Set matrix to identity
 *
 * @param m Matrix to set
 */
void mat6_identity(mat6x6_t m);

/**
 * @brief Copy matrix
 *
 * @param dst Destination matrix
 * @param src Source matrix
 */
void mat6_copy(mat6x6_t dst, const mat6x6_t src);

/**
 * @brief Matrix addition: C = A + B
 *
 * @param c Result matrix
 * @param a First operand
 * @param b Second operand
 */
void mat6_add(mat6x6_t c, const mat6x6_t a, const mat6x6_t b);

/**
 * @brief Matrix subtraction: C = A - B
 *
 * @param c Result matrix
 * @param a First operand
 * @param b Second operand
 */
void mat6_sub(mat6x6_t c, const mat6x6_t a, const mat6x6_t b);

/**
 * @brief Scalar multiplication: B = s * A
 *
 * @param b Result matrix
 * @param a Input matrix
 * @param s Scalar value
 */
void mat6_scale(mat6x6_t b, const mat6x6_t a, float s);

/**
 * @brief Matrix multiplication: C = A * B
 *
 * @param c Result matrix (6x6)
 * @param a First operand (6x6)
 * @param b Second operand (6x6)
 */
void mat6_mult(mat6x6_t c, const mat6x6_t a, const mat6x6_t b);

/**
 * @brief Matrix transpose: B = A^T
 *
 * @param b Result matrix
 * @param a Input matrix
 */
void mat6_transpose(mat6x6_t b, const mat6x6_t a);

/**
 * @brief In-place transpose (for symmetric forcing)
 *
 * @param m Matrix to transpose in place
 */
void mat6_transpose_inplace(mat6x6_t m);

/* ==========================================================================
 * Matrix-Vector Operations
 * ========================================================================== */

/**
 * @brief Set vector to zero
 *
 * @param v Vector to zero
 */
void vec6_zero(vec6_t v);

/**
 * @brief Copy vector
 *
 * @param dst Destination vector
 * @param src Source vector
 */
void vec6_copy(vec6_t dst, const vec6_t src);

/**
 * @brief Vector addition: c = a + b
 *
 * @param c Result vector
 * @param a First operand
 * @param b Second operand
 */
void vec6_add(vec6_t c, const vec6_t a, const vec6_t b);

/**
 * @brief Vector subtraction: c = a - b
 *
 * @param c Result vector
 * @param a First operand
 * @param b Second operand
 */
void vec6_sub(vec6_t c, const vec6_t a, const vec6_t b);

/**
 * @brief Scalar multiplication: b = s * a
 *
 * @param b Result vector
 * @param a Input vector
 * @param s Scalar value
 */
void vec6_scale(vec6_t b, const vec6_t a, float s);

/**
 * @brief Matrix-vector multiplication: y = A * x
 *
 * @param y Result vector (6)
 * @param a Matrix (6x6)
 * @param x Input vector (6)
 */
void mat6_vec_mult(vec6_t y, const mat6x6_t a, const vec6_t x);

/* ==========================================================================
 * Advanced Operations
 * ========================================================================== */

/**
 * @brief Matrix inversion using Gauss-Jordan elimination
 *
 * Computes B = A^(-1).
 *
 * @param b Result inverse matrix
 * @param a Input matrix
 * @return 0 on success, -1 if matrix is singular
 */
int mat6_invert(mat6x6_t b, const mat6x6_t a);

/**
 * @brief Force matrix symmetry: A = (A + A^T) / 2
 *
 * Important for covariance matrix numerical stability.
 *
 * @param m Matrix to symmetrize in place
 */
void mat6_force_symmetric(mat6x6_t m);

/**
 * @brief Compute matrix trace
 *
 * @param m Input matrix
 * @return Sum of diagonal elements
 */
float mat6_trace(const mat6x6_t m);

/**
 * @brief Check if matrix contains NaN or Inf
 *
 * @param m Matrix to check
 * @return true if matrix is valid (no NaN/Inf)
 */
bool mat6_is_valid(const mat6x6_t m);

/**
 * @brief Check if matrix is positive definite
 *
 * Uses simple diagonal check (approximate).
 *
 * @param m Matrix to check
 * @return true if all diagonal elements are positive
 */
bool mat6_is_positive_definite(const mat6x6_t m);

/* ==========================================================================
 * Specialized EKF Operations
 * ========================================================================== */

/**
 * @brief Compute P = F * P * F^T + Q (covariance prediction)
 *
 * Optimized for EKF prediction step.
 *
 * @param p_new Result covariance (output)
 * @param p Current covariance
 * @param f State transition matrix
 * @param q Process noise covariance
 */
void mat6_covariance_predict(mat6x6_t p_new, const mat6x6_t p,
                              const mat6x6_t f, const mat6x6_t q);

/**
 * @brief Joseph form covariance update: P = (I-KH)P(I-KH)^T + KRK^T
 *
 * Numerically stable covariance update.
 *
 * @param p_new Result covariance (output)
 * @param p Current covariance
 * @param k Kalman gain (6xm)
 * @param h Measurement matrix (mx6)
 * @param r Measurement noise covariance (mxm)
 * @param m Measurement dimension (1 or 3)
 */
void mat6_joseph_update(mat6x6_t p_new, const mat6x6_t p,
                        const float *k, const float *h,
                        const float *r, int m);

/* ==========================================================================
 * Debug/Print Functions
 * ========================================================================== */

/**
 * @brief Print matrix to console (debug)
 *
 * @param name Matrix name for label
 * @param m Matrix to print
 */
void mat6_print(const char *name, const mat6x6_t m);

/**
 * @brief Print vector to console (debug)
 *
 * @param name Vector name for label
 * @param v Vector to print
 */
void vec6_print(const char *name, const vec6_t v);

#ifdef __cplusplus
}
#endif

#endif /* MATRIX_MATH_H */
