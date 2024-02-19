// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;

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

    private String stateName;

    private CANSparkFlex intakeLeaderM;
    private CANSparkFlex intakeFollowerM;
    private TalonSRX serializationM; // Someone told me this will control both

    //TODO configure motor methods for motors, pid???

    public Intake() {
        intakeLeaderM = new CANSparkFlex(Constants.HardwarePorts.intakeLeaderM, MotorType.kBrushless);
        configMotor(intakeLeaderM);
        intakeFollowerM = new CANSparkFlex(Constants.HardwarePorts.intakeFollowerM, MotorType.kBrushless);
        configMotor(intakeFollowerM);

        intakeFollowerM.follow(intakeLeaderM, false);
        serializationM = new TalonSRX(Constants.HardwarePorts.serializationM);
        configMotor(serializationM, false);
    }

    private void configMotor(CANSparkFlex motor) {
        motor.setSmartCurrentLimit(Constants.intakePeakCurrentLimit);
        motor.setIdleMode(IdleMode.kCoast);
    }

    private void configMotor(TalonSRX motor, boolean inverted) {
        motor.setInverted(true);
        motor.configPeakCurrentLimit(Constants.serializationPeakCurrentLimit);
        motor.configContinuousCurrentLimit(Constants.serializationContinuousCurrentLimit);
    }

    public enum IntakeStates {
        ON(.8),
        OFF(0),
        REV(-1);
        private double speed;
        public double getValue() {
            return speed;
        }

        IntakeStates(double speed) {
            this.speed = speed;
        }
    }

    public void setSpeed(IntakeStates state) {
        intakeLeaderM.set(state.speed);
        // I mean we could just make a second enum value right? but this works and theres really no reason to do so
        if (state == IntakeStates.ON) {
            serializationM.set(ControlMode.PercentOutput, 0.5);
        } else {
            serializationM.set(ControlMode.PercentOutput, 0);
        }
        
        this.stateName = state.name();

    }


    @Override
    public void periodic() {
        if (stateName != null) {
            SmartDashboard.putString("Intake State", stateName);
        }
    }

    @Override
    public void simulationPeriodic() {
    }
}
