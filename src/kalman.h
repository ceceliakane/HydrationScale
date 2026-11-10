#pragma once

// Scalar (1D) Kalman filter
class KalmanFilter {
  private:
    float q, r, x, p, k;
  public:
    KalmanFilter(float processNoise, float measurementNoise,
                 float initialError, float initialValue)
      : q(processNoise), r(measurementNoise), x(initialValue), p(initialError), k(0) {}

    float update(float measurement) {
      p = p + q;
      k = p / (p + r);
      x = x + k * (measurement - x);
      p = (1.0f - k) * p;
      return x;
    }

    void reset(float value, float initialError = 1.0f) {
      x = value;
      p = initialError;
    }
};
