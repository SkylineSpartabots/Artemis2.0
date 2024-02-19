// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
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
    private TalonSRX SerializationM; // Someone told me this will control both

    //TODO configure motor methods for motors, pid???

    public Intake() {
        intakeLeaderM = new CANSparkFlex(Constants.HardwarePorts.intakeLeaderM, MotorType.kBrushless);
        intakeFollowerM = new CANSparkFlex(Constants.HardwarePorts.intakeFollowerM, MotorType.kBrushless);
        intakeFollowerM.setInverted(true);
        intakeFollowerM.follow(intakeLeaderM);
        SerializationM = new TalonSRX(Constants.HardwarePorts.serializationM);
        SerializationM.setInverted(true);
    }

    public enum IntakeStates {
        ON(1),
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
        if (state == IntakeStates.ON) {
            SerializationM.set(ControlMode.PercentOutput, 1);
        } else {
            SerializationM.set(ControlMode.PercentOutput, 0);
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
