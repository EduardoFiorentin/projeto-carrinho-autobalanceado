#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

/**
 * One-dimensional complementary filter for tilt angle estimation.
 *
 * Gyroscope integration gives fast short-term response, while accelerometer
 * angle provides a long-term gravity reference. The first update initializes
 * directly from the accelerometer to avoid starting from a fake zero angle.
 */
class ComplementaryFilter {
  public:
    // alpha near 1.0 trusts gyro integration more; lower alpha trusts accel more.
    explicit ComplementaryFilter(float alpha);

    // Fuse accelerometer angle and gyro rate using the real elapsed time in seconds.
    float update(float accelAngle, float gyroRate, float dt);

    // Return the latest estimated angle in degrees.
    float value() const;

    // Indicates whether update() has received the first accelerometer reference.
    bool initialized() const;

    // Clear the estimator so the next update can reinitialize from accelAngle.
    void reset(float initialAngle = 0.0f);

  private:
    float alpha;
    float angle = 0.0f;
    bool hasEstimate = false;
};

#endif
