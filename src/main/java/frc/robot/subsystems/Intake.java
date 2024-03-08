// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Intake extends SubsystemBase {

    private static Intake instance;
    private Follower follow = new Follower(Constants.HardwarePorts.intakeLeaderM, false );
    private DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private IntakeStates currentState = IntakeStates.OFF;

    private TalonFX intakeLeaderM;
    private TalonFX intakeFollowerM;

    // private CANSparkFlex intakeLeaderM;
    // private CANSparkFlex intakeFollowerM;
    private TalonFX serialM; // Someone told me this will control both

    //TODO configure motor methods for motors, pid??? // DONE?

    public Intake() {
        // Rollers
        intakeLeaderM = new TalonFX(Constants.HardwarePorts.intakeLeaderM);
        intakeFollowerM = new TalonFX(Constants.HardwarePorts.intakeFollowerM);
        configMotor(intakeLeaderM);
        configMotor(intakeFollowerM);
        //intakeFollowerM.setControl(follow);
        
        
        // intakeLeaderM = new CANSparkFlex(Constants.HardwarePorts.intakeLeaderM, MotorType.kBrushless);
        // configMotor(intakeLeaderM);
        // intakeFollowerM = new CANSparkFlex(Constants.HardwarePorts.intakeFollowerM, MotorType.kBrushless);
        // configMotor(intakeFollowerM);
        // intakeFollowerM.follow(intakeLeaderM, false);

        // Serial
        serialM = new TalonFX(Constants.HardwarePorts.serialM);
        configMotor(serialM, false);
    }

    private void configMotor(TalonFX motor) {
        // motor.setSmartCurrentLimit(Constants.intakePeakCurrentLimit); for testing
        // motor.setIdleMode(IdleMode.kCoast);
        // motor.setInverted(false);

        TalonFXConfiguration config = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();

        currentLimitsConfigs.SupplyCurrentLimit = Constants.intakeContinuousCurrentLimit;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentThreshold = Constants.intakePeakCurrentLimit;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.7;
        // config.OpenLoopRamps.

        Slot0Configs slot0Configs = new Slot0Configs();
        // slot0Configs.kP = Constants.SwerveConstants.driveKP;
        // slot0Configs.kI = Constants.SwerveConstants.driveKI;
        // slot0Configs.kD = Constants.SwerveConstants.driveKD;
        config.CurrentLimits = currentLimitsConfigs;
        motor.getConfigurator().apply(config);
    }

    private void configMotor(TalonFX motor, boolean inverted) {
        motor.setInverted(false);
        //motor.configPeakCurrentLimit(Constants.serializationPeakCurrentLimit); for testing
        // motor.configContinuousCurrentLimit(Constants.serializationContinuousCurrentLimit); for testing
    }

    public double getMotorVoltage() {
        return intakeLeaderM.getMotorVoltage().getValueAsDouble();
    }

    public enum IntakeStates {
        ON(0.3, 0.8),
        INDEX(0.5, 0.5),
        OFF(0, 0),
        REV(-0.8, -0.8);

        private double speed;
        private double serialSpeed;

        public double getValue() {
            return speed;
        }

        IntakeStates(double speed, double serialSpeed) {
            this.speed = speed;
            this.serialSpeed = serialSpeed;
        }
    }

    public void setSpeed(IntakeStates state) {
        intakeLeaderM.setControl(dutyCycleRequest.withOutput(state.speed));
        
        intakeFollowerM.setControl(follow);
        serialM.set(state.serialSpeed);
        currentState = state;
    }


    /**
     * Testing purposes only, should not be used during any comps
     */
    private double currentIntakePercentage = Integer.MAX_VALUE;

    public void incPower() {
        if (currentIntakePercentage == Integer.MAX_VALUE) {
            if (currentState != null) {
                currentIntakePercentage = currentState.speed;
            }
        }
        currentIntakePercentage += 0.05;
        intakeLeaderM.set(currentIntakePercentage);
    }

    public void decPower() {
        if (currentIntakePercentage == Integer.MAX_VALUE) {
            if (currentState != null) {
                currentIntakePercentage = currentState.speed;
            }
        }
        currentIntakePercentage -= 0.05;
        intakeLeaderM.set(currentIntakePercentage);
    }


    @Override
    public void periodic() {
        // if (stateName != null) {
        //     SmartDashboard.putString("Intake State", stateName);
        // }
        SmartDashboard.putBoolean("Intake On", intakeLeaderM.getMotorVoltage().getValueAsDouble() > 2);
    }

    @Override
    public void simulationPeriodic() {
    }
}
