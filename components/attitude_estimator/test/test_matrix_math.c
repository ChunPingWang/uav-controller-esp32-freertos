/**
 * @file test_matrix_math.c
 * @brief Comprehensive matrix math unit tests
 *
 * Tests: T016a - Matrix math operations
 */

#include "unity.h"
#include "matrix_math.h"
#include <string.h>
#include <math.h>

#define N EKF_STATE_DIM

/* Helper to print matrix for debugging */
static void print_matrix(const char *name, const float *M)
{
    (void)name;
    (void)M;
    /* Uncomment for debugging:
    printf("%s:\n", name);
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            printf("%8.4f ", M[i * N + j]);
        }
        printf("\n");
    }
    */
}

/* ==========================================================================
 * Basic Operations
 * ========================================================================== */

TEST_CASE("mat6_zero clears all elements", "[matrix]")
{
    float M[N * N];
    /* Fill with non-zero values */
    for (int i = 0; i < N * N; i++) {
        M[i] = 99.0f;
    }

    mat6_zero(M);

    for (int i = 0; i < N * N; i++) {
        TEST_ASSERT_FLOAT_WITHIN(1e-10f, 0.0f, M[i]);
    }
}

TEST_CASE("mat6_identity sets diagonal to 1", "[matrix]")
{
    float M[N * N];
    mat6_identity(M);

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            float expected = (i == j) ? 1.0f : 0.0f;
            TEST_ASSERT_FLOAT_WITHIN(1e-10f, expected, M[i * N + j]);
        }
    }
}

TEST_CASE("mat6_copy duplicates matrix", "[matrix]")
{
    float src[N * N], dst[N * N];

    for (int i = 0; i < N * N; i++) {
        src[i] = (float)i * 1.5f - 10.0f;
    }
    mat6_zero(dst);

    mat6_copy(dst, src);

    for (int i = 0; i < N * N; i++) {
        TEST_ASSERT_FLOAT_WITHIN(1e-10f, src[i], dst[i]);
    }
}

TEST_CASE("mat6_scale multiplies all elements", "[matrix]")
{
    float M[N * N];

    for (int i = 0; i < N * N; i++) {
        M[i] = (float)(i + 1);
    }

    mat6_scale(M, M, 2.5f);

    for (int i = 0; i < N * N; i++) {
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, (float)(i + 1) * 2.5f, M[i]);
    }
}

TEST_CASE("mat6_add sums two matrices", "[matrix]")
{
    float A[N * N], B[N * N], C[N * N];

    for (int i = 0; i < N * N; i++) {
        A[i] = (float)i;
        B[i] = (float)(N * N - i);
    }

    mat6_add(C, A, B);

    for (int i = 0; i < N * N; i++) {
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, (float)(N * N), C[i]);
    }
}

/* ==========================================================================
 * Transpose
 * ========================================================================== */

TEST_CASE("mat6_transpose swaps rows and columns", "[matrix]")
{
    float A[N * N], At[N * N];

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            A[i * N + j] = (float)(i * 100 + j);
        }
    }

    mat6_transpose(At, A);

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            TEST_ASSERT_FLOAT_WITHIN(1e-10f, A[i * N + j], At[j * N + i]);
        }
    }
}

TEST_CASE("mat6_transpose of symmetric is unchanged", "[matrix]")
{
    float S[N * N], St[N * N];

    /* Create symmetric matrix */
    for (int i = 0; i < N; i++) {
        for (int j = 0; j <= i; j++) {
            float val = (float)(i + j + 1);
            S[i * N + j] = val;
            S[j * N + i] = val;
        }
    }

    mat6_transpose(St, S);

    for (int i = 0; i < N * N; i++) {
        TEST_ASSERT_FLOAT_WITHIN(1e-10f, S[i], St[i]);
    }
}

/* ==========================================================================
 * Multiplication
 * ========================================================================== */

TEST_CASE("mat6_mult with identity", "[matrix]")
{
    float A[N * N], I[N * N], C[N * N];

    for (int i = 0; i < N * N; i++) {
        A[i] = (float)i * 0.3f + 1.0f;
    }
    mat6_identity(I);

    mat6_mult(C, A, I);

    for (int i = 0; i < N * N; i++) {
        TEST_ASSERT_FLOAT_WITHIN(1e-5f, A[i], C[i]);
    }
}

TEST_CASE("mat6_mult associativity", "[matrix]")
{
    float A[N * N], B[N * N], C[N * N];
    float AB[N * N], BC[N * N];
    float AB_C[N * N], A_BC[N * N];

    /* Initialize matrices */
    for (int i = 0; i < N * N; i++) {
        A[i] = (float)(i % 7) * 0.5f;
        B[i] = (float)(i % 5) * 0.3f + 0.1f;
        C[i] = (float)(i % 3) * 0.2f + 0.2f;
    }

    /* (A * B) * C */
    mat6_mult(AB, A, B);
    mat6_mult(AB_C, AB, C);

    /* A * (B * C) */
    mat6_mult(BC, B, C);
    mat6_mult(A_BC, A, BC);

    for (int i = 0; i < N * N; i++) {
        TEST_ASSERT_FLOAT_WITHIN(1e-3f, AB_C[i], A_BC[i]);
    }
}

/* ==========================================================================
 * Inversion
 * ========================================================================== */

TEST_CASE("mat6_invert identity gives identity", "[matrix]")
{
    float I[N * N], I_inv[N * N];

    mat6_identity(I);
    int result = mat6_invert(I_inv, I);

    TEST_ASSERT_EQUAL(0, result);

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            float expected = (i == j) ? 1.0f : 0.0f;
            TEST_ASSERT_FLOAT_WITHIN(1e-5f, expected, I_inv[i * N + j]);
        }
    }
}

TEST_CASE("mat6_invert A * A_inv = I", "[matrix]")
{
    float A[N * N], A_inv[N * N], product[N * N];

    /* Create invertible matrix (diagonally dominant) */
    mat6_zero(A);
    for (int i = 0; i < N; i++) {
        A[i * N + i] = 10.0f + (float)i;
        if (i > 0) A[i * N + (i - 1)] = 1.0f;
        if (i < N - 1) A[i * N + (i + 1)] = 1.0f;
    }

    int result = mat6_invert(A_inv, A);
    TEST_ASSERT_EQUAL(0, result);

    mat6_mult(product, A, A_inv);

    print_matrix("A", A);
    print_matrix("A_inv", A_inv);
    print_matrix("A * A_inv", product);

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            float expected = (i == j) ? 1.0f : 0.0f;
            TEST_ASSERT_FLOAT_WITHIN(1e-3f, expected, product[i * N + j]);
        }
    }
}

TEST_CASE("mat6_invert singular matrix returns error", "[matrix]")
{
    float singular[N * N], inv[N * N];

    /* All zeros - singular */
    mat6_zero(singular);

    int result = mat6_invert(inv, singular);
    TEST_ASSERT_NOT_EQUAL(0, result);
}

TEST_CASE("mat6_invert diagonal matrix", "[matrix]")
{
    float D[N * N], D_inv[N * N];

    mat6_zero(D);
    for (int i = 0; i < N; i++) {
        D[i * N + i] = (float)(i + 2); /* 2, 3, 4, 5, 6, 7 */
    }

    int result = mat6_invert(D_inv, D);
    TEST_ASSERT_EQUAL(0, result);

    /* Inverse of diagonal is reciprocal of diagonal */
    for (int i = 0; i < N; i++) {
        float expected = 1.0f / (float)(i + 2);
        TEST_ASSERT_FLOAT_WITHIN(1e-5f, expected, D_inv[i * N + i]);
    }
}

/* ==========================================================================
 * Trace
 * ========================================================================== */

TEST_CASE("mat6_trace of identity is N", "[matrix]")
{
    float I[N * N];
    mat6_identity(I);

    float trace = mat6_trace(I);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, (float)N, trace);
}

TEST_CASE("mat6_trace sums diagonal", "[matrix]")
{
    float M[N * N];
    mat6_zero(M);

    float expected_trace = 0.0f;
    for (int i = 0; i < N; i++) {
        M[i * N + i] = (float)(i * i);
        expected_trace += (float)(i * i);
    }

    float trace = mat6_trace(M);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, expected_trace, trace);
}

/* ==========================================================================
 * Covariance Operations
 * ========================================================================== */

TEST_CASE("mat6_covariance_predict maintains symmetry", "[matrix][ekf]")
{
    float P[N * N], P_out[N * N];
    float F[N * N], Q[N * N];

    /* Start with symmetric positive definite P */
    mat6_identity(P);
    mat6_scale(P, P, 0.1f);

    /* F close to identity */
    mat6_identity(F);
    F[0 * N + 3] = 0.01f; /* Small coupling */

    /* Q diagonal */
    mat6_zero(Q);
    for (int i = 0; i < N; i++) {
        Q[i * N + i] = 0.001f;
    }

    mat6_covariance_predict(P_out, P, F, Q);

    /* Check symmetry */
    for (int i = 0; i < N; i++) {
        for (int j = i + 1; j < N; j++) {
            TEST_ASSERT_FLOAT_WITHIN(1e-6f, P_out[i * N + j], P_out[j * N + i]);
        }
    }
}

TEST_CASE("mat6_joseph_update reduces covariance", "[matrix][ekf]")
{
    float P_prior[N * N], P_post[N * N];
    float H[N];
    float R = 0.01f;

    /* Initial covariance */
    mat6_identity(P_prior);
    mat6_scale(P_prior, P_prior, 1.0f);

    /* Measurement matrix - only observes first state */
    for (int i = 0; i < N; i++) {
        H[i] = (i == 0) ? 1.0f : 0.0f;
    }

    mat6_joseph_update(P_post, P_prior, H, R);

    /* Posterior covariance should be <= prior for observed state */
    TEST_ASSERT_TRUE(P_post[0] <= P_prior[0]);
}
