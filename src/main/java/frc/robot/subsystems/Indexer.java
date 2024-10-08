// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.ColorSensorV3;


import com.ctre.phoenix6.controls.Follower;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {

    private static Indexer instance;

    public static Indexer getInstance() {
        if (instance == null) {
            instance = new Indexer();
        }
        return instance;
    }

    private TalonFX indexerLeaderM;
    private TalonFX indexerFollowerM;

    private Follower follow = new Follower(Constants.HardwarePorts.indexerTopM, false );

    private ColorSensorV3 colorSensor;
    private DigitalInput limitSwitch = new DigitalInput(0);
    private static final I2C.Port onboardI2C = I2C.Port.kOnboard;

    public Indexer() {
        indexerLeaderM = new TalonFX(Constants.HardwarePorts.indexerTopM);
        indexerFollowerM = new TalonFX(Constants.HardwarePorts.indexerBottomM);
        indexerLeaderM.setInverted(true);
        indexerFollowerM.setControl(follow);
        colorSensor = new ColorSensorV3(onboardI2C);
    }

    public enum IndexerStates {
        ON(0.6),
        SHOOTING(0.8),
        AMP(0.7),
        ONAUTO(0.32),
        OFF(0),
        REV(-0.8);
        
        private double speed;

        public double getValue() {
            return speed;
        }

        IndexerStates(double speed) {
            this.speed = speed;
        }
    }

    public void setSpeed(double percentageOutput) {
        indexerLeaderM.set(percentageOutput);
    }


    public void setState(IndexerStates state){
        indexerLeaderM.set(state.speed);
        indexerFollowerM.setControl(follow);
    }

    public int getColorSensorResult() {
        return colorSensor.getProximity();
    }

    // public boolean getLimitSwitchResult() {
    //     return limitSwitch.get();
    // }

    public double getMotorVoltage() {
        return indexerLeaderM.getMotorVoltage().getValueAsDouble();
    }

    public double getMotorCurrent() {
        return indexerLeaderM.getTorqueCurrent().getValueAsDouble();
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Indexer/Color Sensor Proximity", getColorSensorResult());
        SmartDashboard.putNumber("Indexer/Indexer Motor Current", getMotorCurrent());
        // SmartDashboard.putBoolean("Limit Switch State", getLimitSwitchResult());
    }

    @Override
    public void simulationPeriodic() {
    }
}
