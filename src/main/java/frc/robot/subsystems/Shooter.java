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

    private CANSparkMax shooterTopM;
    private CANSparkMax shooterBottomM;

    private double currentTopSpeed = 0;
    private double currentBottomSpeed = 0;

    public Shooter() {
        shooterTopM = new CANSparkMax(Constants.HardwarePorts.shooterLeaderM, MotorType.kBrushless);
        shooterBottomM = new CANSparkMax(Constants.HardwarePorts.shooterFollowerM, MotorType.kBrushless);
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

    public void setSpeed(double speed, boolean MotorLocation) {
        if(MotorLocation) {
            shooterTopM.set(speed);
            currentTopSpeed = speed;
        } else if (MotorLocation) {
            shooterBottomM.set(speed);
            currentBottomSpeed = speed;
        }
    }

    /**
     * @param MotorLocation
     * true = top motor
     * false = bottom motor
     */
    public double getSpeed(boolean MotorLocation) { //gets specific Speed (i hope)
        if(MotorLocation) {return currentTopSpeed;} else{ return currentBottomSpeed;}
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
