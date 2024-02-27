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
import frc.robot.subsystems.Intake.IntakeStates;

public class Indexer extends SubsystemBase {

    private static Indexer instance;

    public static Indexer getInstance() {
        if (instance == null) {
            instance = new Indexer();
        }
        return instance;
    }

    private double currentTopSpeed = 0;
    private double currentBottomSpeed = 0;
    private CANSparkFlex indexerTopM;
    private CANSparkFlex indexerBottomM;
    private ColorSensorV3 colorSensor;
    private static final I2C.Port onboardI2C = I2C.Port.kOnboard;

    public Indexer() {
        indexerTopM = new CANSparkFlex(Constants.HardwarePorts.indexerTopM, MotorType.kBrushless);
        indexerBottomM = new CANSparkFlex(Constants.HardwarePorts.indexerBottomM, MotorType.kBrushless);
        indexerTopM.setInverted(true);
        indexerBottomM.setInverted(true);

        colorSensor = new ColorSensorV3(onboardI2C);
    }

    public enum IndexerStates {
        ON(0.4),
        OFF(0),
        REV(-0.3);
        
        private double speed;

        public double getValue() {
            return speed;
        }

        IndexerStates(double speed) {
            this.speed = speed;
        }
    }

    public enum IndexerMotors {
        BOTTOM(1),
        TOP(2),
        BOTH(0);
        private int motor;

        public int getMotor() {
            return motor;
        }

        IndexerMotors(int motor) {
            this.motor = motor;
        }

    }
    /**
     * @param speeds: Array containing speeds for Bottom and Top motors (bottom, top)
     * @param MotorLocation: Choose motors to set, 1 is Bottom motor, 2 is Top motor, 0 is Both
     */
    public void setSpeed(double[] speeds, IndexerMotors MotorLocation) {

        // if this all doesnt work to test just use the last else if and put that as all that is in this method

        if (MotorLocation == IndexerMotors.BOTTOM) {
            indexerBottomM.set(speeds[0]);
            currentBottomSpeed = speeds[0];
        } else if (MotorLocation == IndexerMotors.TOP) {
            indexerTopM.set(speeds[1]);
            currentTopSpeed = speeds[1];
        } else if (MotorLocation == IndexerMotors.BOTH) {
            indexerTopM.set(speeds[1]);
            currentTopSpeed = speeds[1];
            indexerBottomM.set(speeds[0]);
            currentBottomSpeed = speeds[0];
        }
//        this.stateName = state.name();
    }
    public double getTopSpeed() {
        return currentTopSpeed;
    }

    public double getBottomSpeed() {
        return currentBottomSpeed;
    }

    public void setState(IndexerStates state){
        indexerTopM.set(state.speed);
        indexerBottomM.set(state.speed);
    }

    public double[] getBothSpeeds() {
        return new double[]{currentBottomSpeed, currentTopSpeed};
    }

    public double getTopMotorVoltage() {
        return indexerTopM.getBusVoltage();
    }

    public double getBotMotorVoltage() {
        return indexerBottomM.getBusVoltage();
    }

    public int getColorSensorResult() {
        return colorSensor.getProximity();
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Top Speed", String.valueOf(currentTopSpeed));
        SmartDashboard.putString("Bottom Speed", String.valueOf(currentBottomSpeed));
        SmartDashboard.putNumber("Color Sensor Proximity", getColorSensorResult());
    }

    @Override
    public void simulationPeriodic() {
    }
}
