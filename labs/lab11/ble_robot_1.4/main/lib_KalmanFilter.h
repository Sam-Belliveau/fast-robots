#pragma once

#include "lib_BasicLinearAlgebra.h"

template <int N> struct KalmanFilter {
    BLA::Matrix<N, 1> x; // state estimate
    BLA::Matrix<N, N> P; // error covariance

    KalmanFilter() {
        x.Fill(0);
        P.Fill(0);
    }

    // Prediction without control input
    void predict(
        const BLA::Matrix<N, N> &F, // state transition matrix
        const BLA::Matrix<N, N> &Q  // process noise covariance
    ) {
        // state prediction
        x = F * x;

        // noise prediction
        P = F * P * ~F + Q;
    }

    // Prediction with K-dimensional control input
    template <int K>
    void predict(
        const BLA::Matrix<N, N> &F, // state transition matrix
        const BLA::Matrix<N, K> &B, // control input matrix
        const BLA::Matrix<K, 1> &u, // control input
        const BLA::Matrix<N, N> &Q  // process noise covariance
    ) {
        x = F * x + B * u;
        P = F * P * ~F + Q;
    }

    // Measurement update with M-dimensional measurement
    template <int M>
    void update(
        const BLA::Matrix<M, N> &H, // measurement matrix
        const BLA::Matrix<M, M> &R, // measurement noise covariance
        const BLA::Matrix<M, 1> &z  // measurement
    ) {
        BLA::Matrix<M, 1> y = z - H * x;
        BLA::Matrix<M, M> S = H * P * ~H + R;
        BLA::Matrix<N, M> K = P * ~H * BLA::Inverse(S);
        x = x + K * y;

        BLA::Matrix<N, N> I;
        I.Fill(0);
        for (int i = 0; i < N; i++)
            I(i, i) = 1;
        P = (I - K * H) * P;
    }

    void reset() {
        x.Fill(0);
        P.Fill(0);
    }
};
