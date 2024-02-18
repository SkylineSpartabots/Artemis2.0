// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    private static Shooter instance;


    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private String stateName; // to be used for periodic display - set everytime shooter motor is changed
    private CANSparkMax shooterTopM;
    private CANSparkMax shooterBottomM;

    private double currentTopSpeed = 0;
    private double currentBottomSpeed = 0;

    public Shooter() {
        shooterTopM = new CANSparkMax(Constants.HardwarePorts.shooterTopM, MotorType.kBrushless);
        shooterBottomM = new CANSparkMax(Constants.HardwarePorts.shooterBottomM, MotorType.kBrushless);
        shooterBottomM.setInverted(true);
    }

    public enum ShooterStates {
        MAX(1),
        OFF(0);
        private double speed;

        public double getValue() {
            return speed;
        }

        ShooterStates(double speed) {
            this.speed = speed;
        }

    }

    public enum ShooterMotors {
        BOTTOM(1),
        TOP(2),
        BOTH(0);
        private int motor;

        public int getMotor() {
            return motor;
        }

        ShooterMotors(int motor) {
            this.motor = motor;
        }
    }

    /**
     * @param speeds
     * @param MotorLocation: 1 is top motor, 2 is bottom motor, 0 is both
     */
    public void setSpeed(double[] speeds, ShooterMotors MotorLocation) { //sets speed of motors using specific speed values
        var motor = MotorLocation.getMotor();
        if (MotorLocation == Shooter.ShooterMotors.BOTTOM) {
            shooterBottomM.set(speeds[motor]);
            currentBottomSpeed = speeds[motor];
        } else if (MotorLocation == Shooter.ShooterMotors.TOP) {
            shooterTopM.set(speeds[motor]);
            currentTopSpeed = speeds[motor];
        } else if (MotorLocation == Shooter.ShooterMotors.BOTH) {
            shooterTopM.set(speeds[motor]);
            shooterBottomM.set(speeds[motor]);
            currentTopSpeed = speeds[motor];
            currentBottomSpeed = speeds[motor];
        }
    }

    public void setSpeed(double rpm) {
        double[] speeds = new double[3];
        for (int i = 0; i < 3; i++) {
            speeds[i] = rpm;
        }
        setSpeed(speeds, ShooterMotors.BOTH);
    } //what does do 🦅🦅🍔🍔🌭 ??

    public double getSpeed(Shooter.ShooterMotors motor) {
        if (motor == Shooter.ShooterMotors.BOTTOM) {
            return currentBottomSpeed;
        } else if (motor == Shooter.ShooterMotors.TOP) { //could just have this be else but it would look weird
            return currentTopSpeed;
        } else {
            return -1;
        }
    }

    public double getTopSpeed() { //gets specific Speed (i hope)
        return currentTopSpeed;
    }

    public double getBottomSpeed() { //gets specific Speed (i hope)
        return currentBottomSpeed;
    }

    public double[] getBothSpeeds() {
        return new double[]{currentBottomSpeed, currentTopSpeed};
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Top Speed", String.valueOf(currentTopSpeed));
        SmartDashboard.putString("Bottom Speed", String.valueOf(currentBottomSpeed));
    }

    @Override
    public void simulationPeriodic() {
    }
}
