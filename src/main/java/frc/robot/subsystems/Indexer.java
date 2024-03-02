// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.ColorSensorV3;

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
    private CANSparkFlex indexerLeaderM;
    private CANSparkFlex indexerFollowerM;
    private ColorSensorV3 colorSensor;
    private static final I2C.Port onboardI2C = I2C.Port.kOnboard;

    public Indexer() {
        indexerLeaderM = new CANSparkFlex(Constants.HardwarePorts.indexerTopM, MotorType.kBrushless);
        indexerFollowerM = new CANSparkFlex(Constants.HardwarePorts.indexerBottomM, MotorType.kBrushless);
        indexerLeaderM.setInverted(true);
        indexerFollowerM.follow(indexerLeaderM, false);

        colorSensor = new ColorSensorV3(onboardI2C);
    }

    public enum IndexerStates {
        ON(0.8),
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

    }

    public int getColorSensorResult() {
        return colorSensor.getProximity();
    }

    public double getMotorVoltage() {
        return indexerLeaderM.getBusVoltage();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Color Sensor Proximity", getColorSensorResult());
    }

    @Override
    public void simulationPeriodic() {
    }
}
