// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.ArrayList;
import java.util.Optional;

import com.choreo.lib.*;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Pivot.SetPivot;
import frc.robot.commands.Shooter.SetShooterVelocity;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer.IndexerMotors;
import frc.robot.subsystems.Indexer.IndexerStates;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Pivot.PivotState;
import frc.robot.commands.SetIndexer;
import frc.robot.commands.SetIntake;

public final class Autos {  
  private static CommandSwerveDrivetrain s_Swerve = CommandSwerveDrivetrain.getInstance();

  public static Command getAutoCommand(AutoPath autoPath){
    return autoPath.autoCommand;
  }
  public static Command FollowChoreoTrajectory(ChoreoTrajectory path) {
    SwerveRequest.ApplyChassisSpeeds drive = new SwerveRequest.ApplyChassisSpeeds();
    PIDController thetaController = new PIDController(0.013, 0, 0); //TODO: tune
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    ChoreoTrajectory traj = path;
    thetaController.reset();

    s_Swerve.setAutoStartPose(traj.getPoses()[0]);
    s_Swerve.resetOdo(traj.getInitialPose());
    Command swerveCommand = Choreo.choreoSwerveCommand(
      traj,
        s_Swerve::getPose,
        new PIDController(1, 0.5, 0),
        new PIDController(1, 0.5, 0),                                                           
        thetaController,
        (ChassisSpeeds speeds) -> s_Swerve.setControl(drive.withSpeeds(speeds)),
        // (ChassisSpeeds speeds) -> s_Swerve.applyRequest(() -> drive.withSpeeds(speeds)),
            () -> { Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
              return alliance.isPresent() && alliance.get() == Alliance.Red;},
              s_Swerve);
      
      return swerveCommand;
  }

  public static Command FourNoteSubwoofer(){
    ArrayList<ChoreoTrajectory> trajectory = Choreo.getTrajectoryGroup("FourNoteSubwoofer");
    return new SequentialCommandGroup(
      new SetPivot(PivotState.SUBWOOFER),
      new SetShooterVelocity(2500),
      Commands.waitSeconds(0.8),
      new SetShooterVelocity(0),

      new ParallelCommandGroup(
        FollowChoreoTrajectory(trajectory.get(0)),
        
      )
    )
  }

  public static Command FourNoteCloseSide(){
    ArrayList<ChoreoTrajectory> trajectory = Choreo.getTrajectoryGroup("FourNoteCloseSide");
    return new SequentialCommandGroup(
    new SetPivot(PivotState.SUBWOOFER),
    new SetShooterVelocity(2500),
    Commands.waitSeconds(0.8),
    new SetShooterVelocity(0),

    FollowChoreoTrajectory(trajectory.get(0)),

    new ParallelCommandGroup(
      new SetIntake(IntakeStates.ON),
      new SetIndexer(IndexerStates.ON, IndexerMotors.BOTH),
      FollowChoreoTrajectory(trajectory.get(1))
    ),

    new SetIntake(IntakeStates.OFF),
    new SetIndexer(IndexerStates.OFF, IndexerMotors.BOTH),
    new SetShooterVelocity(2500),
    Commands.waitSeconds(0.8),
    new SetShooterVelocity(0),

    FollowChoreoTrajectory(trajectory.get(2)),

    new ParallelCommandGroup(
    new 
    )
    );
  }

  public static Command ThreeNoteFarSide(){
    ArrayList<ChoreoTrajectory> trajectory = Choreo.getTrajectoryGroup("ThreeNoteFarSide");
    return new SequentialCommandGroup(
    new SetPivot(PivotState.SUBWOOFER),
    new SetShooterVelocity(2500), //TODO: add color sensor so we can make shooter commands end when the note leaves the robot instead of waiting a set amount of time
    Commands.waitSeconds(0.8),
    new SetShooterVelocity(0),

    FollowChoreoTrajectory(trajectory.get(0)),

    new ParallelCommandGroup(
      new SetIntake(IntakeStates.ON),
      new SetIndexer(IndexerStates.ON, IndexerMotors.BOTH),
      FollowChoreoTrajectory(trajectory.get(1))
    ),

    new ParallelCommandGroup(
      new SetIntake(IntakeStates.OFF),
      new SetIndexer(IndexerStates.OFF, IndexerMotors.BOTH), //might have to turn indexer off later if it takes too long to index
      FollowChoreoTrajectory(trajectory.get(2))
    ),

    new SetShooterVelocity(2500), //TODO: add setpivot command before this so that it actually aims at the speaker
    Commands.waitSeconds(0.8),
    new SetShooterVelocity(0),

    FollowChoreoTrajectory(trajectory.get(2)),

    new ParallelCommandGroup(
      new SetIntake(IntakeStates.ON),
      new SetIndexer(IndexerStates.ON, IndexerMotors.BOTH),
      FollowChoreoTrajectory(trajectory.get(3))
    ),

    new ParallelCommandGroup(
      new SetIntake(IntakeStates.OFF),
      new SetIndexer(IndexerStates.OFF, IndexerMotors.BOTH), //might have to turn index off later if it takes too long to index
      FollowChoreoTrajectory(trajectory.get(4))
    ),

    FollowChoreoTrajectory(trajectory.get(5)),

    new SetShooterVelocity(2500),
    Commands.waitSeconds(0.8),
    new SetShooterVelocity(0)
    );
  }

  /*
   * Enum for the different autos. Contains a name and a mechCommands array. The mechCommands array contains 
   * all the commands that the mechanisms will use (stuff that is unrelated to the drivetrain). These commands 
   * will be executed in the order they are in the array during the auto path. Refer to runAutoCommand(AutoType auto).
   */
  public enum AutoPath {

    //when writing enums, if you want multiple mechCommands to run before the next path, put them in a sequential command group
    //if you want those mechCommands to run in parallel, put them in a parallelCommandGroup
    //if you want to run a mechCommand or mechCommandGroup in parallel with a path, create a boolean array with true values corresponding to the mechCommands you want to run in parallel.
      ThreeNoteFarSide("ThreeNoteFarSide", ThreeNoteFarSide()),
      FourNoteCloseSide("FourNoteCloseSide", FourNoteCloseSide()),
      FourNoteSubwoofer("FourNoteSubwoofer", FourNoteSubwoofer());

      String name;
      Command autoCommand;

      private AutoPath(String a, Command autoCommand){
        name = a;
        this.autoCommand = autoCommand;
      }
  }
}