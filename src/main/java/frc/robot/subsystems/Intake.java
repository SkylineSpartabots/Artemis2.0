// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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

    private String stateName;

    private CANSparkMax intakeFollowerM;
    private CANSparkMax intakeLeaderM;

    private TalonSRX SerializationM; // Someone told me this will control both

    public Intake() {
        intakeLeaderM = new CANSparkMax(Constants.HardwarePorts.intakeLeaderM, MotorType.kBrushless);
        intakeFollowerM = new CANSparkMax(Constants.HardwarePorts.intakeFollowerM, MotorType.kBrushless);
        intakeFollowerM.setInverted(true);
        intakeFollowerM.follow(intakeLeaderM);
        SerializationM.follow((IMotorController) intakeLeaderM); // this seems illegal but idk
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
        intakeLeaderM.set(state.speed);
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
