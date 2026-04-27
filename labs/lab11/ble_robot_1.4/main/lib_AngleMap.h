#pragma once

// Circular angle map: N buckets evenly spaced over 360 degrees.
// Each reading is split between the two nearest bucket centers using a
// tent (linear) kernel. A sample exactly on a bucket center contributes
// weight 1.0 to that bucket; a sample halfway between two centers
// contributes 0.5 to each. Each bucket stores a running weighted
// distance sum and a weight sum.

#include <math.h>

template <int N = 18> class AngleMap {
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

    // Add a reading at angle_deg using a tent kernel between the two
    // nearest bucket centers. The unused std parameter is kept so call
    // sites that pass a kernel width still compile.
    void update(float angle_deg, float distance, float = 0.0f) {
        float a = fmodf(angle_deg, 360.0f);
        if (a < 0.0f)
            a += 360.0f;

        const float f = a / BucketWidth;
        const int i_lo = ((int)floorf(f)) % N;
        const float frac = f - floorf(f);
        const int i_hi = (i_lo + 1) % N;

        const float w_lo = 1.0f - frac;
        const float w_hi = frac;

        _sum_dist[i_lo] += w_lo * distance;
        _sum_weight[i_lo] += w_lo;
        _sum_dist[i_hi] += w_hi * distance;
        _sum_weight[i_hi] += w_hi;
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
        return (float)i * BucketWidth;
    }

  private:
    float _sum_dist[N];
    float _sum_weight[N];
};
