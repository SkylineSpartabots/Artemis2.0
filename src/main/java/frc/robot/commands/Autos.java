// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.Optional;

import com.choreo.lib.*;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;



//pathplanner imports
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.CommandUtil;
import com.pathplanner.lib.auto.AutoBuilder;

public final class Autos {

  ChoreoTrajectory traj;
  private static CommandSwerveDrivetrain s_Swerve = CommandSwerveDrivetrain.getInstance();


  public static Command Robot1_3Note() {

    // Use the PathPlannerAuto class to get a path group from an auto
    List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("Example Auto");

    // You can also get the starting pose from the auto. Only call this if the auto actually has a starting pose.
    Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile("Example Auto");

    //use this to load choreo trajectory into a pathplannerpath
    PathPlannerPath exampleChoreoTraj = PathPlannerPath.fromChoreoTrajectory("Example Choreo Traj");


    //the line below WILL NOT WORK without configuring AutoBuilder: see https://pathplanner.dev/pplib-build-an-auto.html#load-an-auto
    //I don't know how to integrate it right now bc it needs to be configured in a drive subsystem which extends SubsystemBase
    //also it needs getPose(), resetPose(), getRobotRelativeSpeeds() or getCurrentSpeeds(), driveRobotRelative() or drive().

    return new PathPlannerAuto("r1_3note"); //according to api should work (allegedly)

    
  }
  // Return auto selected in Shuffleboard
  /**
   * Runs an auto command depending on the AutoType enum variable. Works by
   * assembling all commands
   * that will be executed in the autopath into one SequentialCommandGroup and
   * then scheduling that
   * command group to the command scheduler.
   * 
   * @param auto AutoType enum representing the auto path that is to be run.
   */
  public static Command getAutoCommand(AutoPath auto) {
    SwerveRequest.ApplyChassisSpeeds drive = new SwerveRequest.ApplyChassisSpeeds();
    PIDController thetaController = new PIDController(0.013, 0, 0); // TODO: tune
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    ChoreoTrajectory traj = Choreo.getTrajectory(auto.name);
    thetaController.reset();

    //s_Swerve.setAutoStartPose(traj.getPoses()[0]); //this isn't working for some reason, maybe this code isn't up to date
    SmartDashboard.putString("auto path", auto.name);
    s_Swerve.resetOdo(traj.getInitialPose());
    Command swerveCommand = Choreo.choreoSwerveCommand(
        traj,
        s_Swerve::getPose,
        new PIDController(0.57, 0.2, 0),
        new PIDController(0.57, 0.2, 0),
        thetaController,
        (ChassisSpeeds speeds) -> s_Swerve.setControl(drive.withSpeeds(speeds)),
        // (ChassisSpeeds speeds) -> s_Swerve.applyRequest(() ->
        // drive.withSpeeds(speeds)),
        () -> {
          Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
          return alliance.isPresent() && alliance.get() == Alliance.Red;
        },
        s_Swerve);

    return swerveCommand;
  }

  /*
   * Enum for the different autos. Contains a name and a mechCommands array. The
   * mechCommands array contains
   * all the commands that the mechanisms will use (stuff that is unrelated to the
   * drivetrain). These commands
   * will be executed in the order they are in the array during the auto path.
   * Refer to runAutoCommand(AutoType auto).
   */
  public enum AutoPath {

    // when writing enums, if you want multiple mechCommands to run before the next
    // path, put them in a sequential command group
    // if you want those mechCommands to run in parallel, put them in a
    // parallelCommandGroup
    // if you want to run a mechCommand or mechCommandGroup in parallel with a path,
    // create a boolean array with true values corresponding to the mechCommands you
    // want to run in parallel.
    StraightPathTesting("StraightPathTesting", new Command[] {}),
    AngledDrivingTesting("AngledDrivingTesting", new Command[] {}),
    StraightAndTurn180Testing("StraightAndTurn180Testing", new Command[] {}),
    TESTPATH("TestPath", new Command[] { new InstantCommand() }),
    NOTHINGTEST("NothingTesting", new Command[] {}),
    FENDER("FenderAuto", new Command[] {});

    String name;
    Command[] mechCommands;
    boolean[] parallelToPath;

    private AutoPath(String a, Command[] mechCommands, boolean[] parallelToPath) {
      name = a;
      this.mechCommands = mechCommands;
      this.parallelToPath = parallelToPath;

    }

    private AutoPath(String a, Command[] mechCommands) {
      name = a;
      this.mechCommands = mechCommands;
      parallelToPath = new boolean[mechCommands.length];
    }
  }
}