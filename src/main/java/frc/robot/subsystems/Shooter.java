// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkFlex;
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
    private CANSparkFlex shooterTopM;
    private CANSparkFlex shooterBottomM;

    private double currentTopSpeed = 0;
    private double currentBottomSpeed = 0;

    public Shooter() {
        shooterTopM = new CANSparkFlex(Constants.HardwarePorts.shooterTopM, MotorType.kBrushless);
        shooterBottomM = new CANSparkFlex(Constants.HardwarePorts.shooterBottomM, MotorType.kBrushless);
        shooterBottomM.setInverted(true);
        configMotors();
    }

    private void configMotors() {
        shooterTopM.setSmartCurrentLimit(Constants.shooterPeakCurrentLimit);

        shooterTopM.getPIDController().setFF(0.0078);
        shooterTopM.getPIDController().setP(0.3);
        shooterTopM.getPIDController().setI(0.017);
        shooterTopM.getPIDController().setD(0.005);

        // shooterBottomM.getPIDController().setFF(0.0);
        // shooterBottomM.getPIDController().setP(0.0);
        // shooterBottomM.getPIDController().setI(0.0);
        // shooterBottomM.getPIDController().setD(0.0);
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
     * @param speeds:        Array containing speeds for Bottom and Top motors (bottom, top)
     * @param MotorLocation: Choose motors to set, 1 is Bottom motor, 2 is Top motor, 0 is Both
     */
    public void setSpeed(double[] speeds, ShooterMotors MotorLocation) { //sets speed of motors using specific speed values

        if (MotorLocation == Shooter.ShooterMotors.BOTTOM) {
            shooterBottomM.set(speeds[0]);
            currentBottomSpeed = speeds[0];
        } else if (MotorLocation == Shooter.ShooterMotors.TOP) {
            shooterTopM.set(speeds[1]);
            currentTopSpeed = speeds[1];
        } else if (MotorLocation == Shooter.ShooterMotors.BOTH) {
            shooterTopM.set(speeds[0]);
            currentTopSpeed = speeds[0];
            shooterBottomM.set(speeds[1]);
            currentBottomSpeed = speeds[1];
        }
    }

    public void setSpeed(double rpm) {
        double[] speeds = new double[3];
        for (int i = 0; i < 3; i++) {
            speeds[i] = rpm;
        }
        setSpeed(speeds, ShooterMotors.BOTH);
    } //what does do 🦅🦅🍔🍔🌭 ??

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
        Logger.recordOutput("Shooter/TopSetpoints", currentTopSpeed);
        Logger.recordOutput("Shooter/BottomSetpoints", currentBottomSpeed);
        Logger.recordOutput("Shooter/topSpeed", shooterTopM.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/botSpeed", shooterBottomM.getEncoder().getVelocity());

        // SmartDashboard.putString("Top Speed", String.valueOf(currentTopSpeed));
        // SmartDashboard.putString("Bottom Speed", String.valueOf(currentBottomSpeed));
    }

    @Override
    public void simulationPeriodic() {
    }
}
