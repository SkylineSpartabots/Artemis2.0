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
        indexerTopM = new CANSparkMax(Constants.HardwarePorts.indexerM, MotorType.kBrushless);
        indexerBottomM = new CANSparkMax(Constants.HardwarePorts.indexerM, MotorType.kBrushless);
        indexerBottomM.setInverted(true);
    }

    public enum IndexerStates {
        ON(1),
        OFF(0);
        private double speed;
        public double getValue() {
            return speed;
        }

        IndexerStates(double    speed) {
            this.speed = speed;
        }
    }

    public enum IndexerMotors{
        BOTTOM(0),
        TOP(1),
        BOTH(2);
        private int motor; //
        public int getValue() {
            return motor;
        }

        IndexerMotors(int motor) {
            this.motor = motor;
        }

    }
    public void setSpeed(double[] speeds, int MotorLocation) {
        if (MotorLocation == 0) {
            indexerBottomM.set(speeds[MotorLocation]);
            currentBottomSpeed = speeds[MotorLocation];
        } else if(MotorLocation == 1) {
            indexerTopM.set(speeds[MotorLocation]);
            currentTopSpeed = speeds[MotorLocation];
        } else if (MotorLocation == 2) {
            indexerTopM.set(speeds[MotorLocation]);
            currentTopSpeed = speeds[MotorLocation];
            indexerBottomM.set(speeds[MotorLocation]);
            currentBottomSpeed = speeds[MotorLocation];
        }
//        this.stateName = state.name();

    }
    public double getSpeed(IndexerMotors motor) {
        if(motor == IndexerMotors.BOTTOM) {
            return currentBottomSpeed;
        } else if (motor == IndexerMotors.TOP){
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
