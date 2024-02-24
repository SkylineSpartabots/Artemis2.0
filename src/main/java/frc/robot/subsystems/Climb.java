// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkBase.IdleMode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  
    private static Climb instance;
    public static Climb getInstance() {
        if(instance == null){
            instance = new Climb();
        }
        return instance;
    }

    private CANSparkFlex climbLeaderM;
    private CANSparkFlex climbFollowerM;
    
    public Climb() {
        climbLeaderM = new CANSparkFlex(Constants.HardwarePorts.climbLeaderMotor, MotorType.kBrushless);
        configLeaderMotor(climbLeaderM);

        climbFollowerM = new CANSparkFlex(Constants.HardwarePorts.climbFollowerMotor, MotorType.kBrushless);
        configFollowerMotor(climbFollowerM, climbLeaderM);
    }
    
    private void configLeaderMotor(CANSparkFlex leaderMotor) {
        leaderMotor.setSmartCurrentLimit(Constants.climbPeakCurrentLimit);
        leaderMotor.setIdleMode(IdleMode.kBrake);
        leaderMotor.enableVoltageCompensation(12);
    }

    private void configFollowerMotor(CANSparkFlex followerMotor, CANSparkFlex leaderMotor) {
        followerMotor.setSmartCurrentLimit(Constants.climbPeakCurrentLimit);
        followerMotor.setIdleMode(IdleMode.kBrake);
        followerMotor.enableVoltageCompensation(12);

        followerMotor.follow(leaderMotor);
    } 


    private double speed;
    private double currentOutput;

    public void setSpeed(double speed){
        climbLeaderM.set(speed); //speed should be -1.0 to 1.0
        this.speed = speed;
    }

    public double getSpeed() {
        return speed;
    }


    private boolean isPeaked = false;
    private double spikeThreshold = 10;

    public boolean getIsPeaked() {
        return isPeaked;
    }
    
    public void setIsPeaked(boolean isPeaked) {
        this.isPeaked = isPeaked;
    }
    
    
    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Climb Rotations", climbLeaderM.getEncoder().getPosition());
        isPeaked = (climbLeaderM.getOutputCurrent() >= spikeThreshold);
        
        SmartDashboard.putNumber("Climb Output Current", climbLeaderM.getOutputCurrent());
        SmartDashboard.putNumber("Climb Speed", getSpeed());
        SmartDashboard.putBoolean("Climb Peaked", getIsPeaked());
    }
}
