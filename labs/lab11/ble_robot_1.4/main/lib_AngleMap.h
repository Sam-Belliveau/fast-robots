#pragma once

// Circular angle map: N buckets evenly spaced over 360 degrees.
// Readings are fused with a Gaussian kernel over the angular axis so a
// single sample contributes to every bucket, weighted by angular distance.
// Each bucket stores a running weighted distance sum and a weight sum.

#include <math.h>

template <int N = 64> class AngleMap {
  public:
    static constexpr int NumBuckets = N;
    static constexpr float BucketWidth = 360.0f / (float)N;

    AngleMap() {
        clear();
    }

    void clear() {
        for (int i = 0; i < N; ++i) {
            _sum_dist[i] = 0.0f;
            _sum_weight[i] = 0.0f;
        }
    }

    // Add a reading at angle_deg with a Gaussian kernel of angular std
    // angle_std_deg. Every bucket is updated so the kernel tails still
    // contribute to neighbours even when std is small relative to 360.
    void update(float angle_deg, float distance, float angle_std_deg = 10.0f) {
        if (angle_std_deg <= 0.0f)
            return;

        const float inv_two_var = 0.5f / (angle_std_deg * angle_std_deg);

        for (int i = 0; i < N; ++i) {
            const float center = ((float)i + 0.5f) * BucketWidth;
            const float d = wrap180(center - angle_deg);
            const float w = expf(-d * d * inv_two_var);
            _sum_dist[i] += w * distance;
            _sum_weight[i] += w;
        }
    }

    // Weighted mean distance at bucket i. Returns -1 if no data.
    float bucket(int i) const {
        if (_sum_weight[i] <= 0.0f)
            return -1.0f;
        return _sum_dist[i] / _sum_weight[i];
    }

    // Read distance at an arbitrary angle by selecting the nearest bucket.
    // Returns -1 if that bucket has no data.
    float read(float angle_deg) const {
        return bucket(index_of(angle_deg));
    }

    static int index_of(float angle_deg) {
        float a = fmodf(angle_deg, 360.0f);
        if (a < 0.0f)
            a += 360.0f;
        int i = (int)floorf(a / BucketWidth);
        if (i >= N)
            i -= N;
        return i;
    }

    static float bucket_center(int i) {
        return ((float)i + 0.5f) * BucketWidth;
    }

  private:
    // Shortest signed angular distance in degrees, result in [-180, 180].
    static float wrap180(float d) {
        return d - 360.0f * roundf(d / 360.0f);
    }

    float _sum_dist[N];
    float _sum_weight[N];
};
