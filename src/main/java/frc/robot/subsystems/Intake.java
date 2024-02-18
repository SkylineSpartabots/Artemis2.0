// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
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

        private CANSparkFlex m_intakeLeader;
    private CANSparkFlex m_intakeFollower;

    private TalonSRX m_serialization; // Someone told me this will control both

    public Intake() {
        m_intakeLeader = new CANSparkFlex(Constants.HardwarePorts.m_intakeLeft, MotorType.kBrushless);
        m_intakeFollower = new CANSparkFlex(Constants.HardwarePorts.m_intakeRight, MotorType.kBrushless);
        m_intakeFollower.setInverted(true);
        m_intakeFollower.follow(m_intakeLeader);

        //! check if this is correct
        m_serialization = new TalonSRX(Constants.HardwarePorts.m_serialization); //! this spins at a different speed than intake motors
        // SerializationM.follow((IMotorController) intakeLeaderM); // this seems illegal but idk
    }

    public enum IntakeStates {
        ON(1),
        OFF(0),
        REV(-1);
        private double speed;
        public double getValue() {
            return speed;
        } // how use

        IntakeStates(double speed) {
            this.speed = speed;
        }
    }

    public void setSpeed(IntakeStates state) {
        m_intakeLeader.set(state.speed);
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
