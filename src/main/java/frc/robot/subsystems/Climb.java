// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  private static Climb instance;
  
  public static Climb getInstance() {
    if(instance == null){
      instance = new Climb();
  }
    return instance;
  }

  private CANSparkFlex climbLeaderM;
  private CANSparkFlex climbFollowerM;

  private RelativeEncoder leaderEncoder;
  private RelativeEncoder followEncoder;

  // Measured in motor rotations
  private int maxHeight = 300;
  private int setState;

  public Climb() {
    climbLeaderM = new CANSparkFlex(Constants.HardwarePorts.climbLeaderMotor, MotorType.kBrushless);
    configMotor(climbLeaderM, true);

    climbFollowerM = new CANSparkFlex(Constants.HardwarePorts.climbFollowerMotor, MotorType.kBrushless);
    configMotor(climbFollowerM, false);
    climbFollowerM.follow(climbLeaderM, true);
    setState = 2;
    leaderEncoder = climbLeaderM.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 7168);
    followEncoder = climbFollowerM.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 7168);

    resetMotorEncoders();
  }


  private void configMotor(CANSparkFlex motor, boolean inverted) {
    motor.setSmartCurrentLimit(Constants.climbPeakCurrentLimit);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(inverted);
    motor.enableVoltageCompensation(12);
  }

  public void resetMotorEncoders() {
    leaderEncoder.setPosition(2);
    followEncoder.setPosition(2);
  }

  public void setClimbSpeed(double speed){
    climbLeaderM.set(speed); //speed should be -1.0 to 1.0
  }

  public void setState(int state) {
    setState = state;
  }

  public double getPosition() {
    return leaderEncoder.getPosition();
  }

  /**
   * Sets the desired voltage to the climb leader motor. 
   * @param voltage The desired voltage. 
   */
  public void setVoltage(double voltage) {
    climbLeaderM.setVoltage(voltage);
  }
  
  public double getSetPoint() {
    return setState;
  }

  public double getMotorCurrent() {
    return (climbLeaderM.getOutputCurrent() + climbFollowerM.getOutputCurrent()) / 2;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climb Leader motor position", leaderEncoder.getPosition());
    SmartDashboard.putNumber("Climb follower position", followEncoder.getPosition());
    // This method will be called once per scheduler run
  }
}
