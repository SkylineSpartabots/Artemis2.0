// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
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
    private String stateName; // to be used for periodic display - should be set everytime shooter is set
    // issue is that shooter can be any random number
    private CANSparkFlex m_shooterTop; // Top Roller
    private CANSparkFlex m_shooterBottom; // Bottom Roller

    private double currentTopSpeed = 0;
    private double currentBottomSpeed = 0;

    public Shooter() {
        m_shooterTop = new CANSparkFlex(Constants.HardwarePorts.m_shooterTop, MotorType.kBrushless);
        m_shooterBottom = new CANSparkFlex(Constants.HardwarePorts.m_shooterBottom, MotorType.kBrushless);
        m_shooterBottom.setInverted(true);
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
    public enum ShooterMotors{
        BOTTOM(0),
        TOP(1),
        BOTH(2);
        private int motor; // can i just use an int idk but it didnt like a motor like CANSparxflex or whatevr
        public int getValue() {
            return motor;
        }

        ShooterMotors(int motor) {
            this.motor = motor;
        }

    }

    public void setSpeed(double[] speeds, int MotorLocation) {
        if (MotorLocation == 0) {
            m_shooterBottom.set(speeds[MotorLocation]);
            currentBottomSpeed = speeds[MotorLocation];
        } else if(MotorLocation == 1) {
            m_shooterTop.set(speeds[MotorLocation]);
            currentTopSpeed = speeds[MotorLocation];
        } else if (MotorLocation == 2) {
            m_shooterTop.set(speeds[MotorLocation]);
            currentTopSpeed = speeds[MotorLocation];
            m_shooterBottom.set(speeds[MotorLocation]);
            currentBottomSpeed = speeds[MotorLocation];
        }
//        this.stateName = state.name();

    }
    public double getSpeed(Shooter.ShooterMotors motor) {
        if(motor == Shooter.ShooterMotors.BOTTOM) {
            return currentBottomSpeed;
        } else if (motor == Shooter.ShooterMotors.TOP){
            return currentTopSpeed;
        } else {
            return 0; // how do i handle this - it wants a return but like there is no default case?
        }
    }
    public double getTopSpeed() { //gets specific Speed (i hope)
        return currentTopSpeed;
    }
    public double getBottomSpeed() { //gets specific Speed (i hope)
        return currentBottomSpeed;
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
