package frc.lib.math;

public class AccelerationIntegrator {
    private double prevAccel = 0.0;
    private double velocity = 0.0;
    private double lastTimestamp = 0.0;

    private double filteredAccel = 0.0;
    private final double alpha = 0.1;  // TODO tune

    public double lowPassFilter(double newAccel) {
        filteredAccel = alpha * newAccel + (1 - alpha) * filteredAccel;
        return filteredAccel;
    }

    // Trapezoidal integration for velocity estimation
    public double integrateAccel(double accel, double currentTime) {
        double dt = currentTime - lastTimestamp;
        velocity += 0.5 * (accel + prevAccel) * dt;

        // Update prev
        prevAccel = accel;
        lastTimestamp = currentTime;

        return velocity;
    }

    public double update(double rawAccel, double currentTime) {
        double filteredAccel = lowPassFilter(rawAccel);  // Filter noisy input
        return integrateAccel(filteredAccel, currentTime);  // Integrate for velocity
    }
}