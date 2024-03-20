// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//lol this is so dumb 

public class Music extends SubsystemBase {
  public static Music instance;
    
  public static Music getInstance() {
      if (instance == null) {
          instance = new Music();
      }
      return instance;
  }

  private String song = "no song in queue :)";

  private TalonFX shooterBottomM;
  private TalonFX shooterTopM;

  private TalonFX indexerTopM;
  private TalonFX indexerBottomM;

  private TalonFX intakeLeaderM;
  private TalonFX intakeFollowerM;
  private TalonFX serialM;

  // private TalonFX climbLeaderMotor;
  // private TalonFX climbFollowerMotor;

  private TalonFX pivotLeaderM;
  private TalonFX pivotFollowerM;

  // private TalonSRX ampMotor;
  
  private TalonFX FrontLeftDriveMotor;
  private TalonFX FrontLeftSteerMotor;

  private TalonFX FrontRightDriveMotor; 
  private TalonFX FrontRightSteerMotor;

  private TalonFX BackLeftDriveMotor;
  private TalonFX BackLeftSteerMotor;

  private TalonFX BackRightDriveMotor;
  private TalonFX BackRightSteerMotor;

  private com.ctre.phoenix6.Orchestra m_Orchestra;


  /** Creates a new Orchestra. */
  public Music() {
    m_Orchestra = new com.ctre.phoenix6.Orchestra();

    shooterBottomM = new TalonFX(Constants.HardwarePorts.shooterBottomM);
    shooterTopM = new TalonFX(Constants.HardwarePorts.shooterTopM);

    indexerTopM = new TalonFX(Constants.HardwarePorts.indexerTopM);
    indexerBottomM = new TalonFX(Constants.HardwarePorts.indexerBottomM);

    intakeLeaderM = new TalonFX(Constants.HardwarePorts.intakeLeaderM);
    intakeFollowerM = new TalonFX(Constants.HardwarePorts.intakeFollowerM);
    serialM = new TalonFX(Constants.HardwarePorts.serialM);

    // climbLeaderMotor = new TalonFX(Constants.HardwarePorts.climbLeaderMotor);
    // climbFollowerMotor = new TalonFX(Constants.HardwarePorts.climbFollowerMotor);

    pivotLeaderM = new TalonFX(Constants.HardwarePorts.pivotLeaderM);
    pivotFollowerM = new TalonFX(Constants.HardwarePorts.pivotFollowerM);

    // ampMotor = new TalonSRX(Constants.HardwarePorts.ampMotor);

    //lol the tuner constants were private and i didnt want to cahgne them to public
    FrontLeftDriveMotor = new TalonFX(10);
    FrontLeftSteerMotor = new TalonFX(11);

    FrontRightDriveMotor = new TalonFX(12);
    FrontRightSteerMotor = new TalonFX(13);

    BackLeftDriveMotor = new TalonFX(14);
    BackLeftSteerMotor = new TalonFX(15);

    BackRightDriveMotor = new TalonFX(16);
    BackRightSteerMotor = new TalonFX(17);
     
    m_Orchestra.addInstrument(shooterBottomM);
    m_Orchestra.addInstrument(shooterTopM);

    m_Orchestra.addInstrument(indexerTopM);
    m_Orchestra.addInstrument(indexerBottomM);

    m_Orchestra.addInstrument(intakeLeaderM);
    m_Orchestra.addInstrument(intakeFollowerM);
    m_Orchestra.addInstrument(serialM);

    // m_Orchestra.addInstrument(climbLeaderMotor);
    // m_Orchestra.addInstrument(climbFollowerMotor);

    m_Orchestra.addInstrument(pivotLeaderM);
    m_Orchestra.addInstrument(pivotFollowerM);

    // m_Orchestra.addInstrument(ampMotor);

    m_Orchestra.addInstrument(FrontLeftDriveMotor);
    m_Orchestra.addInstrument(FrontLeftSteerMotor);

    m_Orchestra.addInstrument(FrontRightDriveMotor);
    m_Orchestra.addInstrument(FrontRightSteerMotor);
    
    m_Orchestra.addInstrument(BackLeftDriveMotor);
    m_Orchestra.addInstrument(BackLeftSteerMotor);

    m_Orchestra.addInstrument(BackRightDriveMotor);
    m_Orchestra.addInstrument(BackRightSteerMotor);
  }

  public void loadMusic(String song) {
    this.song = song;
    m_Orchestra.loadMusic(song);
  }

  public void playMusic() {
    m_Orchestra.play();
  }

  public void stopMusic() {
    m_Orchestra.stop();
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Song", song);
    SmartDashboard.putBoolean("isPlaying", m_Orchestra.isPlaying());
    // This method will be called once per scheduler run
  }
}
