// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
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

    private String stateName; // to be used for periodic display - should be set everytime indexer is set
    // issue is that indexer can be any random number
    private double currentTopSpeed = 0;
    private double currentBottomSpeed = 0;
    private CANSparkMax indexerTopM;
    private CANSparkMax indexerBottomM;

    public Indexer() {
        indexerTopM = new CANSparkMax(Constants.HardwarePorts.indexerTopM, MotorType.kBrushless);
        indexerBottomM = new CANSparkMax(Constants.HardwarePorts.indexerBottomM, MotorType.kBrushless);
        indexerBottomM.setInverted(true);
    }

    public enum IndexerStates {
        ON(1),
        OFF(0);
        private double speed;
        public double getValue() {
            return speed;
        }

        IndexerStates(double speed) {
            this.speed = speed;
        }
    }

    public enum IndexerMotors{
        BOTTOM(1),
        TOP(2),
        BOTH(0);
        private int motor;
        public int getValue() {
            return motor;
        }

        IndexerMotors(int motor) {
            this.motor = motor;
        }

    }
    public void setSpeed(double[] speeds, IndexerMotors MotorLocation) {
        var motor = MotorLocation.getValue();
        if (MotorLocation == IndexerMotors.BOTTOM) {
            indexerBottomM.set(speeds[motor]);
            currentBottomSpeed = speeds[motor];
        } else if(MotorLocation == IndexerMotors.TOP) {
            indexerTopM.set(speeds[motor]);
            currentTopSpeed = speeds[motor];
        } else if (MotorLocation == IndexerMotors.BOTH) {
            indexerTopM.set(speeds[motor]);
            indexerBottomM.set(speeds[motor]);
            currentTopSpeed = speeds[motor];
            currentBottomSpeed = speeds[motor];
        }
//        this.stateName = state.name();

    }
    public double getSpeed(IndexerMotors motor) {
        if(motor == IndexerMotors.BOTTOM) {
            return currentBottomSpeed;
        } else if (motor == IndexerMotors.TOP){
            return currentTopSpeed;
        } else {
            return -1;
        }
    }

    public double getTopSpeed() {
        return currentTopSpeed;
    }
    public double getBottomSpeed() { 
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
