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

    private IntakeStates currentState = IntakeStates.OFF;

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
        ON(0.8, 0.8),
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
        intakeLeaderM.set(state.speed);
        serializationM.set(ControlMode.PercentOutput, state.serialSpeed);
       currentState = state;    
    }

    /**
     * Testing purposes only, should not be used during any comps
     */
    private double currentIntakePercentage = Integer.MAX_VALUE;

    public void incPower(){
        if(currentIntakePercentage == Integer.MAX_VALUE){
            if(currentState != null){
                currentIntakePercentage = currentState.speed;
            }
        }
        currentIntakePercentage += 0.05;
        intakeLeaderM.set(currentIntakePercentage);
    }

    public void decPower(){
        if(currentIntakePercentage == Integer.MAX_VALUE){
            if(currentState != null){
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
    }

    @Override
    public void simulationPeriodic() {
    }
}
