// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Vision;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  SendableChooser<Autos.AutoPath> autoChooser = new SendableChooser<Autos.AutoPath>();

  private RobotContainer m_robotContainer;

  private final Shooter s_Shooter;
  private final Indexer s_Indexer;
  private final Intake s_Intake;
  private final Pivot s_Pivot;
   private final Vision s_Vision;
  private final Climb s_Climb;
  private final Amp s_Amp;

  public Robot() {
    super();
    s_Shooter = Shooter.getInstance();
    s_Indexer = Indexer.getInstance();
    s_Intake = Intake.getInstance();
    s_Pivot = Pivot.getInstance();
     s_Vision = Vision.getInstance();
    s_Climb = Climb.getInstance();
    s_Amp = Amp.getInstance();
  }

  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }
    // if (isReal()) {
    //     Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs") do we have a USB stick?
    //     Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    //     new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    // } else {
    //     setUseTiming(false); // Run as fast as possible
    //     String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
    //     Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
    //     Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    // }

    // Logger.disableDeterministicTimestamps(); // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    // Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
    
    //freaks
    // switch (Constants.currentMode) {
    //   case REAL:
    //     System.out.println("REPLAY");

    //     // Running on a real robot, log to a USB stick ("/U/logs")
    //     Logger.addDataReceiver(new WPILOGWriter());
    //     Logger.addDataReceiver(new NT4Publisher());
    //     break;

    //   case SIM:
    //     System.out.println("REPLAY");

    //     // Running a physics simulator, log to NT
    //     Logger.addDataReceiver(new NT4Publisher());
    //     break;

    //   case REPLAY:
    //     System.out.println("REPLAY");
    //     // Replaying a log, set up replay source
    //     setUseTiming(false); // Run as fast as possible
    //     String logPath = LogFileUtil.findReplayLog();
    //     Logger.setReplaySource(new WPILOGReader(logPath));
    //     Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    //     break;
    // }

    // Logger.start();


  if(isReal()){
      Logger.addDataReceiver(new WPILOGWriter()); // should be savig to usb
      Logger.addDataReceiver(new NT4Publisher());
    }
    else {
      Logger.addDataReceiver(new NT4Publisher());
    }
    Logger.start();


    autoChooser.setDefaultOption("ThreeNoteFarSide", Autos.AutoPath.ThreeNoteFarSide);
    autoChooser.addOption("FourNoteSubWoofer", Autos.AutoPath.FourNoteSubwoofer);
    autoChooser.addOption("FourNoteCloseSide", Autos.AutoPath.FourNoteCloseSide);
    autoChooser.addOption("Horizontal", Autos.AutoPath.Horizontal);
    autoChooser.addOption("Straight", Autos.AutoPath.Straight);
    autoChooser.addOption("Rotation", Autos.AutoPath.Rotation);
    autoChooser.addOption("FourNoteFromTop", Autos.AutoPath.FourNoteFromTop);
    autoChooser.addOption("TwoNoteSubwoofer", Autos.AutoPath.TwoNoteSubwoofer);
    autoChooser.addOption("ThreeNoteSubwooferMidTop", Autos.AutoPath.ThreeNoteSubwooferMidTop);
    autoChooser.addOption("ThreeNoteSubwooferMidBot", Autos.AutoPath.ThreeNoteSubwooferMidBot);
    autoChooser.addOption("FourNoteSubwooferNew", Autos.AutoPath.FourNoteSubwooferNew);
    //autoChooser.addOption("FirstLine", Autos.AutoPath.FirstLine);
    autoChooser.addOption("TwoNoteFarSide", Autos.AutoPath.TwoNoteFarSide);
    /*autoChooser.addOption("Straight and turn 180", Autos.AutoPath.StraightAndTurn180Testing);
    autoChooser.addOption("Angled drive", Autos.AutoPath.AngledDrivingTesting);
    autoChooser.addOption("Turn in place", AutoPath.NOTHINGTEST);*/
    SmartDashboard.putData("Auto choices", autoChooser);
    m_robotContainer = RobotContainer.getInstance();
    PortForwarder.add(5800, "photonvision.local", 5800);

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = Autos.getAutoCommand(autoChooser.getSelected());

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
