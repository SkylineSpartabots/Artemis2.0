// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class Intake extends SubsystemBase {
  /** Creates a new Intake. */


// sorry i was rushed and i did this rlly quickly lol mb

  private CANSparkFlex intakeLeaderMotor;
  private CANSparkFlex intakeFollowerMotor;
  private TalonSRX intakeCerealMotor;
  private double speed;

  private static Intake instance;
  public static Intake getInstance() {
    if (instance == null)
        instance = new Intake();
    return instance;
  }
  
  public Intake() {
      intakeLeaderMotor = new CANSparkFlex(Constants.HardwarePorts.intakeLeaderMotor, MotorType.kBrushless);
      configureIntakeLeaderMotor();
      intakeFollowerMotor = new CANSparkFlex(Constants.HardwarePorts.intakeFollowerMotor, MotorType.kBrushless); 
      configureIntakeFollowerMotor();
      intakeCerealMotor = new TalonSRX(Constants.HardwarePorts.intakeCerealMotor);
      configureCerealMotor();
  }

  public void configureIntakeLeaderMotor() {
    intakeLeaderMotor.setSmartCurrentLimit(Constants.intakePeakCurrentLimit);
    intakeLeaderMotor.setIdleMode(Constants.intakeNeutralMode);
    intakeLeaderMotor.setOpenLoopRampRate(Constants.openLoopRamp);
    
  }
 
  public void configureIntakeFollowerMotor() {
    intakeFollowerMotor.follow(intakeLeaderMotor,false);
    intakeFollowerMotor.setIdleMode(Constants.intakeNeutralMode);
  }

  public void configureCerealMotor() {
  }

  public double getSpeed() {
    return speed;
  }

  public void setSpeed(double val) {
      this.speed = val;
      intakeLeaderMotor.set(val);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
