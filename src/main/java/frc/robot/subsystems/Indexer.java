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

    private String state;   // hi

    private CANSparkMax indexerM;

    public Indexer() {
        indexerM = new CANSparkMax(Constants.HardwarePorts.indexerM, MotorType.kBrushless);
    }

    public enum IndexerStates {
        ON(1),
        OFF(0);
        private double speed;

        IndexerStates(double speed) {
            this.speed = speed;
        }
    }

    public void setSpeed(IndexerStates state) {
        indexerM.set(state.speed);
        this.state = state.name();
    }

    public void setSpeed(double speed) {
        indexerM.set(speed);
    }

    @Override
    public void periodic() {
        if (state != null) {
            SmartDashboard.putString("indexer state", state);
        }
    }

    @Override
    public void simulationPeriodic() {
    }
}
