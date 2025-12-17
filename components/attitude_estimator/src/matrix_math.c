/**
 * @file matrix_math.c
 * @brief 6x6 matrix operations implementation
 *
 * Lightweight matrix library for EKF on ESP32.
 * No dynamic allocation (Constitution III).
 */

#include "matrix_math.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

/* ==========================================================================
 * Basic Matrix Operations
 * ========================================================================== */

void mat6_zero(mat6x6_t m)
{
    memset(m, 0, sizeof(mat6x6_t));
}

void mat6_identity(mat6x6_t m)
{
    mat6_zero(m);
    for (int i = 0; i < MAT_DIM; i++) {
        m[i][i] = 1.0f;
    }
}

void mat6_copy(mat6x6_t dst, const mat6x6_t src)
{
    memcpy(dst, src, sizeof(mat6x6_t));
}

void mat6_add(mat6x6_t c, const mat6x6_t a, const mat6x6_t b)
{
    for (int i = 0; i < MAT_DIM; i++) {
        for (int j = 0; j < MAT_DIM; j++) {
            c[i][j] = a[i][j] + b[i][j];
        }
    }
}

void mat6_sub(mat6x6_t c, const mat6x6_t a, const mat6x6_t b)
{
    for (int i = 0; i < MAT_DIM; i++) {
        for (int j = 0; j < MAT_DIM; j++) {
            c[i][j] = a[i][j] - b[i][j];
        }
    }
}

void mat6_scale(mat6x6_t b, const mat6x6_t a, float s)
{
    for (int i = 0; i < MAT_DIM; i++) {
        for (int j = 0; j < MAT_DIM; j++) {
            b[i][j] = a[i][j] * s;
        }
    }
}

void mat6_mult(mat6x6_t c, const mat6x6_t a, const mat6x6_t b)
{
    mat6x6_t temp;

    for (int i = 0; i < MAT_DIM; i++) {
        for (int j = 0; j < MAT_DIM; j++) {
            float sum = 0.0f;
            for (int k = 0; k < MAT_DIM; k++) {
                sum += a[i][k] * b[k][j];
            }
            temp[i][j] = sum;
        }
    }

    mat6_copy(c, temp);
}

void mat6_transpose(mat6x6_t b, const mat6x6_t a)
{
    for (int i = 0; i < MAT_DIM; i++) {
        for (int j = 0; j < MAT_DIM; j++) {
            b[j][i] = a[i][j];
        }
    }
}

void mat6_transpose_inplace(mat6x6_t m)
{
    for (int i = 0; i < MAT_DIM; i++) {
        for (int j = i + 1; j < MAT_DIM; j++) {
            float temp = m[i][j];
            m[i][j] = m[j][i];
            m[j][i] = temp;
        }
    }
}

/* ==========================================================================
 * Vector Operations
 * ========================================================================== */

void vec6_zero(vec6_t v)
{
    memset(v, 0, sizeof(vec6_t));
}

void vec6_copy(vec6_t dst, const vec6_t src)
{
    memcpy(dst, src, sizeof(vec6_t));
}

void vec6_add(vec6_t c, const vec6_t a, const vec6_t b)
{
    for (int i = 0; i < MAT_DIM; i++) {
        c[i] = a[i] + b[i];
    }
}

void vec6_sub(vec6_t c, const vec6_t a, const vec6_t b)
{
    for (int i = 0; i < MAT_DIM; i++) {
        c[i] = a[i] - b[i];
    }
}

void vec6_scale(vec6_t b, const vec6_t a, float s)
{
    for (int i = 0; i < MAT_DIM; i++) {
        b[i] = a[i] * s;
    }
}

void mat6_vec_mult(vec6_t y, const mat6x6_t a, const vec6_t x)
{
    vec6_t temp;

    for (int i = 0; i < MAT_DIM; i++) {
        float sum = 0.0f;
        for (int j = 0; j < MAT_DIM; j++) {
            sum += a[i][j] * x[j];
        }
        temp[i] = sum;
    }

    vec6_copy(y, temp);
}

/* ==========================================================================
 * Matrix Inversion (Gauss-Jordan)
 * ========================================================================== */

int mat6_invert(mat6x6_t b, const mat6x6_t a)
{
    mat6x6_t aug;
    mat6_copy(aug, a);

    /* Initialize result as identity */
    mat6_identity(b);

    /* Forward elimination with partial pivoting */
    for (int col = 0; col < MAT_DIM; col++) {
        /* Find pivot */
        int max_row = col;
        float max_val = fabsf(aug[col][col]);

        for (int row = col + 1; row < MAT_DIM; row++) {
            if (fabsf(aug[row][col]) > max_val) {
                max_val = fabsf(aug[row][col]);
                max_row = row;
            }
        }

        /* Check for singularity */
        if (max_val < 1e-10f) {
            return -1; /* Singular matrix */
        }

        /* Swap rows if needed */
        if (max_row != col) {
            for (int j = 0; j < MAT_DIM; j++) {
                float temp = aug[col][j];
                aug[col][j] = aug[max_row][j];
                aug[max_row][j] = temp;

                temp = b[col][j];
                b[col][j] = b[max_row][j];
                b[max_row][j] = temp;
            }
        }

        /* Scale pivot row */
        float pivot = aug[col][col];
        for (int j = 0; j < MAT_DIM; j++) {
            aug[col][j] /= pivot;
            b[col][j] /= pivot;
        }

        /* Eliminate column */
        for (int row = 0; row < MAT_DIM; row++) {
            if (row != col) {
                float factor = aug[row][col];
                for (int j = 0; j < MAT_DIM; j++) {
                    aug[row][j] -= factor * aug[col][j];
                    b[row][j] -= factor * b[col][j];
                }
            }
        }
    }

    return 0;
}

/* ==========================================================================
 * Advanced Operations
 * ========================================================================== */

void mat6_force_symmetric(mat6x6_t m)
{
    for (int i = 0; i < MAT_DIM; i++) {
        for (int j = i + 1; j < MAT_DIM; j++) {
            float avg = (m[i][j] + m[j][i]) * 0.5f;
            m[i][j] = avg;
            m[j][i] = avg;
        }
    }
}

float mat6_trace(const mat6x6_t m)
{
    float trace = 0.0f;
    for (int i = 0; i < MAT_DIM; i++) {
        trace += m[i][i];
    }
    return trace;
}

bool mat6_is_valid(const mat6x6_t m)
{
    for (int i = 0; i < MAT_DIM; i++) {
        for (int j = 0; j < MAT_DIM; j++) {
            if (isnan(m[i][j]) || isinf(m[i][j])) {
                return false;
            }
        }
    }
    return true;
}

bool mat6_is_positive_definite(const mat6x6_t m)
{
    /* Simple check: all diagonal elements positive */
    for (int i = 0; i < MAT_DIM; i++) {
        if (m[i][i] <= 0.0f) {
            return false;
        }
    }
    return true;
}

/* ==========================================================================
 * EKF Specialized Operations
 * ========================================================================== */

void mat6_covariance_predict(mat6x6_t p_new, const mat6x6_t p,
                              const mat6x6_t f, const mat6x6_t q)
{
    mat6x6_t fp, ft, fpft;

    /* FP = F * P */
    mat6_mult(fp, f, p);

    /* F^T */
    mat6_transpose(ft, f);

    /* FPF^T = FP * F^T */
    mat6_mult(fpft, fp, ft);

    /* P_new = FPF^T + Q */
    mat6_add(p_new, fpft, q);

    /* Force symmetry for numerical stability */
    mat6_force_symmetric(p_new);
}

void mat6_joseph_update(mat6x6_t p_new, const mat6x6_t p,
                        const float *k, const float *h,
                        const float *r, int m)
{
    /* Joseph form: P = (I-KH)P(I-KH)^T + KRK^T
     *
     * This is more numerically stable than the standard update.
     * For simplicity, we implement for m=1 (scalar) and m=3 cases.
     */

    mat6x6_t I_KH;
    mat6_identity(I_KH);

    if (m == 1) {
        /* Scalar measurement case */
        for (int i = 0; i < MAT_DIM; i++) {
            for (int j = 0; j < MAT_DIM; j++) {
                I_KH[i][j] -= k[i] * h[j];
            }
        }

        /* (I-KH) * P */
        mat6x6_t temp1;
        mat6_mult(temp1, I_KH, p);

        /* (I-KH)^T */
        mat6x6_t I_KH_T;
        mat6_transpose(I_KH_T, I_KH);

        /* (I-KH) * P * (I-KH)^T */
        mat6_mult(p_new, temp1, I_KH_T);

        /* Add K * R * K^T */
        float r_val = r[0];
        for (int i = 0; i < MAT_DIM; i++) {
            for (int j = 0; j < MAT_DIM; j++) {
                p_new[i][j] += k[i] * r_val * k[j];
            }
        }
    } else if (m == 3) {
        /* 3-measurement case (accelerometer) */
        /* For efficiency, we use the standard update for this case */
        /* P = (I - KH) * P */
        for (int i = 0; i < MAT_DIM; i++) {
            for (int j = 0; j < MAT_DIM; j++) {
                float kh_sum = 0.0f;
                for (int mm = 0; mm < 3; mm++) {
                    kh_sum += k[i * 3 + mm] * h[mm * MAT_DIM + j];
                }
                I_KH[i][j] -= kh_sum;
            }
        }

        mat6_mult(p_new, I_KH, p);
    }

    /* Force symmetry */
    mat6_force_symmetric(p_new);
}

/* ==========================================================================
 * Debug Functions
 * ========================================================================== */

void mat6_print(const char *name, const mat6x6_t m)
{
    printf("%s:\n", name);
    for (int i = 0; i < MAT_DIM; i++) {
        printf("  [");
        for (int j = 0; j < MAT_DIM; j++) {
            printf("%8.4f", m[i][j]);
            if (j < MAT_DIM - 1) printf(", ");
        }
        printf("]\n");
    }
}

void vec6_print(const char *name, const vec6_t v)
{
    printf("%s: [", name);
    for (int i = 0; i < MAT_DIM; i++) {
        printf("%8.4f", v[i]);
        if (i < MAT_DIM - 1) printf(", ");
    }
    printf("]\n");
}
