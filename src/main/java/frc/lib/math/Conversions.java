package frc.lib.math;

public class Conversions {
    
    /**
     * @param rotations CANCoder Rotations
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double CANcoderToDegrees(double rotations, double gearRatio) {
        return rotations * (360.0 / gearRatio);
    }

    /**
     * @param degrees   Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return CANCoder Rotations
     */
    public static double degreesToCANcoder(double degrees, double gearRatio) {
        return degrees / (360.0 / gearRatio);
    }

    
    /**
     * @param revolutions Motor Revolutions
     * @param gearRatio   Gear Ratio between Motor and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double motorToDegrees(double revolutions, double gearRatio) {
        return revolutions * (360 / gearRatio);
    }

    /**
     * @param degrees   Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @return Motor rotations
     */
    public static double degreesToMotor(double degrees, double gearRatio) {
        return degrees / (360 / gearRatio);
    }

    /**
     * @param motorRPM  Motor RPM
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @return RPM of Mechanism
     */
    public static double motorToWheelRPM(double motorRPM, double gearRatio) {
        return motorRPM / gearRatio;
    }

    /**
     * @param RPM       RPM of mechanism
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @return RPM of Motor
     */
    public static double wheelRPMToMotor(double RPM, double gearRatio) {
        return RPM * gearRatio;
    }

    /**
     * @param motorRPM      Motor RPM
     * @param circumference Circumference of Wheel
     * @param gearRatio     Gear Ratio between Motor and Mechanism 
     * @return Meters Per Second
     */
    public static double motorToMPS(double motorRPM, double circumference, double gearRatio) {
        double wheelRPM = motorToWheelRPM(motorRPM, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * @param velocity      Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio     Gear Ratio between Motor and Mechanism (set to 1 for
     *                      Motor MPS)
     * @return Motor RPM
     */
    public static double MPSToMotor(double velocity, double circumference, double gearRatio){
        double wheelRPM = (velocity * 60) / circumference;
        return wheelRPMToMotor(wheelRPM, gearRatio);
    }

    /**
     * @param rotations     Motor rotations
     * @param circumference Circumference of Wheel
     * @param gearRatio     Gear Ratio between Motor and Wheel
     * @return Meters
     */
    public static double motorToMeters(double rotations, double circumference, double gearRatio) {
        return rotations * (circumference / gearRatio);
    }

    /**
     * @param meters        Meters
     * @param circumference Circumference of Wheel
     * @param gearRatio     Gear Ratio between Motor and Wheel
     * @return Motor Revolutions
     */
    public static double metersToMotor(double meters, double circumference, double gearRatio) {
        return meters / (circumference / gearRatio);
    }
}
