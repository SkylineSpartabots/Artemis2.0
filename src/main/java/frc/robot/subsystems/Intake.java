// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private static Intake instance;


    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private String state;

    private TalonFX intakeFollowerM;
    private CANSparkMax intakeLeaderM;

    public Intake() {
        intakeLeaderM = new CANSparkMax(Constants.HardwarePorts.intakeLeaderM, MotorType.kBrushless);
        intakeFollowerM = new TalonFX(Constants.HardwarePorts.intakeFollowerM);
        intakeFollowerM.setInverted(true);
        intakeFollowerM.setControl(new StrictFollower(Constants.HardwarePorts.intakeLeaderM));
    }

    public enum IntakeStates {
        ON(1),
        OFF(0),
        REV(-1);
        private double speed;

        IntakeStates(double speed) {
            this.speed = speed;
        }
    }

    public void setSpeed(IntakeStates state) {
        intakeLeaderM.set(state.speed);
        this.state = state.name();
    }

    @Override
    public void periodic() {
        if (state != null) {
            SmartDashboard.putString("Intake state", state);
        }
    }

    @Override
    public void simulationPeriodic() {
    }
}
