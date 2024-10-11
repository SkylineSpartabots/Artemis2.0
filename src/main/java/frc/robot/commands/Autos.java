// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.Optional;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Amp.ZeroAmp;
import frc.robot.commands.Intake.SetIntake;
import frc.robot.commands.Pivot.AlignPivot;
import frc.robot.commands.Pivot.ZeroPivot;
import frc.robot.commands.Shooter.SetShooterCommand;
import frc.robot.commands.Shooter.ZeroShooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indexer.IndexerStates;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Pivot.PivotState;
import frc.robot.subsystems.Shooter;



public final class Autos {
    private static CommandSwerveDrivetrain s_Swerve = CommandSwerveDrivetrain.getInstance();
    private static Shooter s_Shooter = Shooter.getInstance();

    private static final PIDController thetaController = new PIDController(3, 1.4, 0); //tune?
    private static final PIDController xController = new PIDController(5, 1, 0);
    private static final PIDController yController = new PIDController(5, 1, 0);

    public static Command getAutoCommand(AutoPath autoPath) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                  new ZeroPivot(),
                  new ZeroAmp()      
                ), 
                autoPath.autoCommand
                );
    }

    public static Command FollowChoreoTrajectory(ChoreoTrajectory path) {
        SwerveRequest.ApplyChassisSpeeds drive = new SwerveRequest.ApplyChassisSpeeds();
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        ChoreoTrajectory traj = path;
        thetaController.reset();
        xController.reset();
        yController.reset();

        s_Swerve.setAutoStartPose(traj.getInitialPose());
        SmartDashboard.putNumber("Start pose x", traj.getInitialPose().getX());
        Command swerveCommand = Choreo.choreoSwerveCommand(
                traj,
                s_Swerve::getPose,
                xController,
                yController,
                thetaController,
                (ChassisSpeeds speeds) -> s_Swerve.setControl(drive.withSpeeds(speeds)),
                // () -> {return false;},
                // (ChassisSpeeds speeds) -> s_Swerve.applyRequest(() -> drive.withSpeeds(speeds)),
                () -> {
                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == Alliance.Red;
                },
                s_Swerve);

        return swerveCommand;
    }

    public static Command FourNoteSubwoofer() {
        ArrayList<ChoreoTrajectory> trajectory = Choreo.getTrajectoryGroup("FourPieceSubwoofer");
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Pose2d initialPose;
                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    initialPose = alliance.isPresent() && alliance.get() != Alliance.Red ? trajectory.get(0).getInitialPose() : trajectory.get(0).flipped().getInitialPose();
                    s_Swerve.resetOdo(initialPose);
                    System.out.println(initialPose.getX() + " " + initialPose.getY());
                }),

                new ParallelCommandGroup(
                        new AlignPivot(PivotState.SUBWOOFER),
                        new SetShooterCommand(40)//, new WaitCommand(0.2)
                ),

                new InstantCommand(() -> Indexer.getInstance().setSpeed(0.8)),
                // new SetIndexer(IndexerStates.ON, false), 
                Commands.waitSeconds(0.2),

                new ParallelCommandGroup(
                        new ZeroShooter(),
                        new SetIndexer(IndexerStates.ON, true, 5),
                        new SetIntake(IntakeStates.ON),
                        new AlignPivot(PivotState.INTAKE),
                        new SequentialCommandGroup(new WaitCommand(0.3), FollowChoreoTrajectory(trajectory.get(0)))
                ),

                Commands.waitSeconds(0.3),

                new ParallelCommandGroup(
                        // new AlignPivot(PivotState.SUBWOOFER), after pivot stops moving it drops a bit
                        CommandFactory.eject(),
                        FollowChoreoTrajectory(trajectory.get(1))
                ),
                CommandFactory.eject(),
                new AlignPivot(PivotState.SUBWOOFER),
                new SetShooterCommand(40),
                new SetIndexer(IndexerStates.ON),
                Commands.waitSeconds(0.2),

                new ParallelCommandGroup(
                        FollowChoreoTrajectory(trajectory.get(2)),
                        new ZeroShooter(),
                        new AlignPivot(PivotState.INTAKE),
                        new SetIndexer(IndexerStates.ON, true),
                        new SetIntake(IntakeStates.ON)
                ),

                Commands.waitSeconds(0.3),

                new ParallelCommandGroup(
                        FollowChoreoTrajectory(trajectory.get(3)),
                        CommandFactory.eject()
                        // new AlignPivot(PivotState.SUBWOOFER)
                ),
                CommandFactory.eject(),

                new ParallelCommandGroup(new SetShooterCommand(40), new AlignPivot(PivotState.SUBWOOFER)),
                new SetIndexer(IndexerStates.ON),
                Commands.waitSeconds(0.5),

                new ParallelCommandGroup(
                        FollowChoreoTrajectory(trajectory.get(4)),
                        new ZeroShooter(),
                        new AlignPivot(PivotState.INTAKE),
                        new SetIndexer(IndexerStates.ON, true),
                        new SetIntake(IntakeStates.ON)
                ),

                Commands.waitSeconds(0.3),

                new ParallelCommandGroup(
                        FollowChoreoTrajectory(trajectory.get(5)),
                        CommandFactory.eject(),
                        new AlignPivot(PivotState.SUBWOOFER)
                ),
                CommandFactory.eject(),

                new ParallelCommandGroup(new SetShooterCommand(40), new AlignPivot(PivotState.SUBWOOFER)),
                new SetIndexer(IndexerStates.ON),
                Commands.waitSeconds(0.5),
                new SetIndexer(IndexerStates.OFF),
                new ParallelCommandGroup(new AlignPivot(PivotState.GROUND), new ZeroShooter()));
    }

    public static Command FourNoteSubwooferNew() {
        ArrayList<ChoreoTrajectory> trajectory = Choreo.getTrajectoryGroup("FourPieceSubwooferNew");
        return new SequentialCommandGroup(
               new InstantCommand(() -> {
                    Pose2d initialPose;
                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    initialPose = alliance.isPresent() && alliance.get() != Alliance.Red ? trajectory.get(0).getInitialPose() : trajectory.get(0).flipped().getInitialPose();
                    s_Swerve.resetOdo(initialPose);
                    System.out.println(initialPose.getX() + " " + initialPose.getY());
                }),
                FollowChoreoTrajectory(trajectory.get(0)),
                Commands.waitSeconds(1),
                FollowChoreoTrajectory(trajectory.get(1)),
                Commands.waitSeconds(1),
                FollowChoreoTrajectory(trajectory.get(2))
        );
    }


    public static Command ThreeNoteSubwooferMidTop(){
      ArrayList<ChoreoTrajectory> trajectory = Choreo.getTrajectoryGroup("FourPieceSubwoofer");
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Pose2d initialPose;
                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    initialPose = alliance.isPresent() && alliance.get() != Alliance.Red ? trajectory.get(0).getInitialPose() : trajectory.get(0).flipped().getInitialPose();
                    s_Swerve.resetOdo(initialPose);
                    System.out.println(initialPose.getX() + " " + initialPose.getY());
                }),

                new ParallelCommandGroup(
                        new AlignPivot(PivotState.SUBWOOFER),
                        new SetShooterCommand(40), new WaitCommand(0.2)
                ),

                new InstantCommand(() -> Indexer.getInstance().setSpeed(0.8)),
                // new SetIndexer(IndexerStates.ON, false), 
                Commands.waitSeconds(0.3),

                new ParallelCommandGroup(
                        new SetShooterCommand(0),
                        new SetIndexer(IndexerStates.ON, true),
                        new SetIntake(IntakeStates.ON),
                        new AlignPivot(PivotState.INTAKE),
                        new SequentialCommandGroup(new WaitCommand(0.3), FollowChoreoTrajectory(trajectory.get(0)))
                ),

                Commands.waitSeconds(0.5),

                new ParallelCommandGroup(
                        new AlignPivot(PivotState.SUBWOOFER),
                        CommandFactory.eject(),
                        FollowChoreoTrajectory(trajectory.get(1))
                ),

                new SetShooterCommand(40),
                new SetIndexer(IndexerStates.ON),
                Commands.waitSeconds(0.5),

                new ParallelCommandGroup(
                        FollowChoreoTrajectory(trajectory.get(2)),
                        new SetShooterCommand(0),
                        new AlignPivot(PivotState.INTAKE),
                        new SetIndexer(IndexerStates.ON, true),
                        new SetIntake(IntakeStates.ON)
                ),

                Commands.waitSeconds(0.3),

                new ParallelCommandGroup(
                        FollowChoreoTrajectory(trajectory.get(3)),
                        CommandFactory.eject(),
                        new AlignPivot(PivotState.SUBWOOFER)
                ),

                new ParallelCommandGroup(new SetShooterCommand(40), new AlignPivot(PivotState.SUBWOOFER)),
                new SetIndexer(IndexerStates.ON),
                Commands.waitSeconds(0.5),

                new SetIndexer(IndexerStates.OFF),
                new ParallelCommandGroup(new AlignPivot(PivotState.GROUND), new SetShooterCommand(0))
                );
    }

    public static Command TwoNoteSubwoofer() {
      ArrayList<ChoreoTrajectory> trajectory = Choreo.getTrajectoryGroup("FourPieceSubwoofer");
      return new SequentialCommandGroup(
              new InstantCommand(() -> {
                  Pose2d initialPose;
                  Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                  initialPose = alliance.isPresent() && alliance.get() != Alliance.Red ? trajectory.get(0).getInitialPose() : trajectory.get(0).flipped().getInitialPose();
                  s_Swerve.resetOdo(initialPose);
                  System.out.println(initialPose.getX() + " " + initialPose.getY());
              }),

              new ParallelCommandGroup(
                      new AlignPivot(PivotState.SUBWOOFER),
                      new SetShooterCommand(40), new WaitCommand(0.2)
              ),

              new InstantCommand(() -> Indexer.getInstance().setSpeed(0.8)),
              // new SetIndexer(IndexerStates.ON, false), 
              Commands.waitSeconds(0.3),

              new ParallelCommandGroup(
                      new SetShooterCommand(0),
                      new SetIndexer(IndexerStates.ON, true),
                      new SetIntake(IntakeStates.ON),
                      new AlignPivot(PivotState.INTAKE),
                      new SequentialCommandGroup(new WaitCommand(0.3), FollowChoreoTrajectory(trajectory.get(0)))
              ),

              Commands.waitSeconds(0.5),

              new ParallelCommandGroup(
                      new AlignPivot(PivotState.SUBWOOFER),
                      CommandFactory.eject(),
                      FollowChoreoTrajectory(trajectory.get(1))
              ),

              new SetShooterCommand(40),
              new SetIndexer(IndexerStates.ON),
              Commands.waitSeconds(0.5),

              new SetIndexer(IndexerStates.OFF),
              new ParallelCommandGroup(new AlignPivot(PivotState.GROUND), new SetShooterCommand(0)));
  }

    public static Command FourNoteFromTop() {
      ArrayList<ChoreoTrajectory> trajectory = Choreo.getTrajectoryGroup("FourNoteMinTranslationTop");
      return new SequentialCommandGroup(
        new InstantCommand(() -> {
            Pose2d initialPose;
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            initialPose = alliance.isPresent() && alliance.get() != Alliance.Red ? trajectory.get(0).getInitialPose() : trajectory.get(0).flipped().getInitialPose();
            s_Swerve.resetOdo(initialPose);
            System.out.println(initialPose.getX() + " " + initialPose.getY());
        }),

        new ParallelCommandGroup(
          new AlignPivot(PivotState.SUBWOOFER),
          new SetShooterCommand(40)
        ),

        new InstantCommand(() -> Indexer.getInstance().setSpeed(0.8)),
        Commands.waitSeconds(0.1),
        
        new ParallelCommandGroup(
          new SetShooterCommand(0),
          new SetIndexer(IndexerStates.ON, true),
          new SetIntake(IntakeStates.ON),
          new AlignPivot(PivotState.INTAKE),
          new SequentialCommandGroup(
            new WaitCommand(0.3), 
            FollowChoreoTrajectory(trajectory.get(0))
            )
        ),

        new ParallelCommandGroup(
          new AlignPivot(35),
          new SetShooterCommand(40)
        ),

        new SetIndexer(IndexerStates.ON),
        Commands.waitSeconds(0.1),

        new ParallelCommandGroup(
          new SetShooterCommand(0),
          new SetIndexer(IndexerStates.ON, true),
          new SetIntake(IntakeStates.ON),
          new AlignPivot(PivotState.INTAKE),
          new SequentialCommandGroup(
            new WaitCommand(0.3), 
            FollowChoreoTrajectory(trajectory.get(1))
            )
        ),

        new ParallelCommandGroup(
          new AlignPivot(35),
          new SetShooterCommand(40)
        ),

        new SetIndexer(IndexerStates.ON),
        Commands.waitSeconds(0.1),

        new ParallelCommandGroup(
          new SetShooterCommand(0),
          new SetIndexer(IndexerStates.ON, true),
          new SetIntake(IntakeStates.ON),
          new AlignPivot(PivotState.INTAKE),
          new SequentialCommandGroup(
            new WaitCommand(0.3), 
            FollowChoreoTrajectory(trajectory.get(2))
          )
        ),

        new ParallelCommandGroup(
          new AlignPivot(35),
          new SetShooterCommand(40)
        ),

        new SetIndexer(IndexerStates.ON),
        Commands.waitSeconds(0.5),

        new ParallelCommandGroup(
          new SetShooterCommand(0),
          new SetIndexer(IndexerStates.OFF),
          new AlignPivot(PivotState.GROUND)
        )
      );
    }

    /*public static Command FirstLine() {
        ArrayList<ChoreoTrajectory> trajectory = Choreo.getTrajectoryGroup("FirstLine");
        return new SequentialCommandGroup(

                new InstantCommand(() -> {
                        Pose2d initialPose;
                        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                        initialPose = alliance.isPresent() && alliance.get() != Alliance.Red ? trajectory.get(0).getInitialPose() : trajectory.get(0).flipped().getInitialPose();
                        s_Swerve.resetOdo(initialPose);
                        System.out.println(initialPose.getX() + " " + initialPose.getY());
                }),

                FollowIntakePath(FollowChoreoTrajectory(trajectory.get(0)), 1.5, 0.3),

                new ParallelCommandGroup(
                        new SetShooterCommand(45),
                        new AlignPivot(30)
                ),

                new SetIndexer(IndexerStates.SHOOTING, false),
                Commands.waitSeconds(0.2),

                FollowAutoCycle(FollowChoreoTrajectory(trajectory.get(1)), 1.5, 0.3, 35, 45),

                FollowIntakePath(FollowChoreoTrajectory(trajectory.get(2)), 2.5, 0),
                Commands.waitSeconds(0.3),

                FollowShootPath(FollowChoreoTrajectory(trajectory.get(3)), 35, 45)

        );
    }
        
*/
    public static Command FourNoteCloseSide() {
        ArrayList<ChoreoTrajectory> trajectory = Choreo.getTrajectoryGroup("FourNoteCloseSide");
        return new SequentialCommandGroup(
          new InstantCommand(() -> {
            Pose2d initialPose;
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            initialPose = alliance.isPresent() && alliance.get() != Alliance.Red ? trajectory.get(0).getInitialPose() : trajectory.get(0).flipped().getInitialPose();
            s_Swerve.resetOdo(initialPose);
            System.out.println(initialPose.getX() + " " + initialPose.getY());
        }),

          new ParallelCommandGroup(
            new AlignPivot(PivotState.SUBWOOFER),
            new SetShooterCommand(45)
          ),

          new SetIndexer(IndexerStates.ON, false),
          Commands.waitSeconds(0.5),

          new ParallelCommandGroup(
            new SetShooterCommand(15),
            FollowChoreoTrajectory(trajectory.get(0)),
            new SetIndexer(IndexerStates.ON, true),
            new SetIntake(IntakeStates.ON)
          ),

          new ParallelCommandGroup(
            new SetShooterCommand(30),
            new AlignPivot(20) //TODO: tune this
          )

        );
    }

    public static Command TwoNoteFarSide() {
        ArrayList<ChoreoTrajectory> trajectory = Choreo.getTrajectoryGroup("TwoNoteFarSide");
        return new SequentialCommandGroup(
             new InstantCommand(() -> {
                    Pose2d initialPose;
                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    initialPose = alliance.isPresent() && alliance.get() != Alliance.Red ? trajectory.get(0).getInitialPose() : trajectory.get(0).flipped().getInitialPose();
                    s_Swerve.resetOdo(initialPose);
                    System.out.println(initialPose.getX() + " " + initialPose.getY());
                }),
                
                new ParallelCommandGroup(
                        new AlignPivot(PivotState.SUBWOOFER),
                        // RobotContainer.getInstance().eject(),
                        new SetShooterCommand(40)
                ),
                Commands.waitSeconds(0.2),
                
                new InstantCommand(() -> Indexer.getInstance().setSpeed(0.8)),
                Commands.waitSeconds(0.2),

                new ParallelCommandGroup(
                        FollowChoreoTrajectory(trajectory.get(0)),
                        new SetShooterCommand(0),
                        new SetIndexer(IndexerStates.ON, true, 4.75),
                        new SetIntake(IntakeStates.ON, 4.5),
                        new AlignPivot(PivotState.INTAKE)
                ),

                Commands.waitSeconds(0.5),
                CommandFactory.eject(),

                new ParallelCommandGroup(
                        FollowChoreoTrajectory(trajectory.get(1)),
                        new AlignPivot(60.5),
                        new SetShooterCommand(40)
                ),

                Commands.waitSeconds(1),
                new SetIndexer(IndexerStates.ON, false),
                Commands.waitSeconds(0.3),

                new ParallelCommandGroup(
                        FollowChoreoTrajectory(trajectory.get(1)),
                        new SetShooterCommand(0),
                        new SetIndexer(IndexerStates.ON, true, 5),
                        new SetIntake(IntakeStates.ON, 4.75),
                        new AlignPivot(PivotState.INTAKE)
                )
        );
    }

    public static Command ThreeNoteFarSide() {
        ArrayList<ChoreoTrajectory> trajectory = Choreo.getTrajectoryGroup("ThreeNoteFarSide");
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Pose2d initialPose;
                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    initialPose = alliance.isPresent() && alliance.get() != Alliance.Red ? trajectory.get(0).getInitialPose() : trajectory.get(0).flipped().getInitialPose();
                    s_Swerve.resetOdo(initialPose);
                    System.out.println(initialPose.getX() + " " + initialPose.getY());
                }),

                new ParallelCommandGroup(
                        new AlignPivot(PivotState.SUBWOOFER),
                        // RobotContainer.getInstance().eject(),
                        new SetShooterCommand(40)
                ),
                Commands.waitSeconds(1.0),
                
                new InstantCommand(() -> Indexer.getInstance().setSpeed(0.8)),
                Commands.waitSeconds(0.2),

                new ParallelCommandGroup(
                        FollowChoreoTrajectory(trajectory.get(0)),
                        new SetShooterCommand(0),
                        new SetIndexer(IndexerStates.ON, true, 4.25),
                        new SetIntake(IntakeStates.ON, 4),
                        new AlignPivot(PivotState.INTAKE)
                ),

                Commands.waitSeconds(0.5),
                CommandFactory.eject(),

                new ParallelCommandGroup(
                        FollowChoreoTrajectory(trajectory.get(1)),
                        new AlignPivot(Constants.getAngleForDistance(3.9)),
                        new SetShooterCommand(40)
                ),


                new SetIndexer(IndexerStates.ON, false),
                Commands.waitSeconds(5),

                new ParallelCommandGroup(
                        FollowChoreoTrajectory(trajectory.get(2)),
                        new AlignPivot(PivotState.INTAKE),
                        new SetShooterCommand(0),
                        new SetIndexer(IndexerStates.ON, true, 3.75),
                        new SetIntake(IntakeStates.ON, 3.5)
                ),

                Commands.waitSeconds(0.5),
                CommandFactory.eject(),

                new ParallelCommandGroup(
                  FollowChoreoTrajectory(trajectory.get(3)),
                  new AlignPivot(35),
                  new SetShooterCommand(30),
                  CommandFactory.eject()


                ),

                new SetIndexer(IndexerStates.ON, false),
                Commands.waitSeconds(0.5),
                new SetIndexer(IndexerStates.OFF, false),
                new SetShooterCommand(0)
        );
    }

    public static Command TwoNote() {
        ArrayList<ChoreoTrajectory> trajectory = Choreo.getTrajectoryGroup("TwoNote");

        // Pose2d initialPose;
        // Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        // initialPose = alliance.isPresent() && alliance.get() == Alliance.Red ? trajectory.get(0).getInitialPose() : trajectory.get(0).flipped().getInitialPose();
        // s_Swerve.resetOdo(initialPose);
        // System.out.println(initialPose.getX() + " " + initialPose.getY());

        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Pose2d initialPose;
                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    initialPose = alliance.isPresent() && alliance.get() != Alliance.Red ? trajectory.get(0).getInitialPose() : trajectory.get(0).flipped().getInitialPose();
                    s_Swerve.resetOdo(initialPose);
                    System.out.println(initialPose.getX() + " " + initialPose.getY());
                }),
                new ParallelCommandGroup(new AlignPivot(PivotState.SUBWOOFER),
                        new SetShooterCommand(40)),
                Commands.waitSeconds(0.8),
                new SetIndexer(IndexerStates.ON, false),
                Commands.waitSeconds(0.8),

                new ParallelCommandGroup(new SetShooterCommand(40), new SetIndexer(IndexerStates.OFF, false)),

                //new ParallelCommandGroup(FollowChoreoTrajectory(trajectory.get(0)), new SetIntake(IntakeStates.ON), new SetIndexer(IndexerStates.ON, true)),
                new ParallelCommandGroup(FollowChoreoTrajectory(trajectory.get(0)), new SetIndexer(IndexerStates.ON, false)),

                new ParallelCommandGroup(FollowChoreoTrajectory(trajectory.get(1)), new SetIntake(IntakeStates.OFF), new SetIndexer(IndexerStates.OFF, false))

                //new ParallelCommandGroup(FollowChoreoTrajectory(trajectory.get(1)), new SetIndexer(IndexerStates.OFF, false)),

                // shootSequence()
        );
    }

    private static Command ThreeNoteSubwooferMidBot() {
      ArrayList<ChoreoTrajectory> trajectory = Choreo.getTrajectoryGroup("ThreeNoteSubwooferMidBot");
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Pose2d initialPose;
                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    initialPose = alliance.isPresent() && alliance.get() != Alliance.Red ? trajectory.get(0).getInitialPose() : trajectory.get(0).flipped().getInitialPose();
                    s_Swerve.resetOdo(initialPose);
                    System.out.println(initialPose.getX() + " " + initialPose.getY());
                }),

                new ParallelCommandGroup(
                        new AlignPivot(PivotState.SUBWOOFER),
                        new SetShooterCommand(40), new WaitCommand(0.2)
                ),

                new InstantCommand(() -> Indexer.getInstance().setSpeed(0.8)),
                // new SetIndexer(IndexerStates.ON, false), 
                Commands.waitSeconds(0.3),

                new ParallelCommandGroup(
                        new SetShooterCommand(0),
                        new SetIndexer(IndexerStates.ON, true),
                        new SetIntake(IntakeStates.ON),
                        new AlignPivot(PivotState.INTAKE),
                        new SequentialCommandGroup(new WaitCommand(0.3), FollowChoreoTrajectory(trajectory.get(0)))
                ),

                Commands.waitSeconds(0.5),

                new ParallelCommandGroup(
                        new AlignPivot(PivotState.SUBWOOFER),
                        CommandFactory.eject(),
                        FollowChoreoTrajectory(trajectory.get(1))
                ),

                new SetShooterCommand(40),
                new SetIndexer(IndexerStates.ON, false),
                Commands.waitSeconds(0.5),

                new ParallelCommandGroup(
                        FollowChoreoTrajectory(trajectory.get(2)),
                        new SetShooterCommand(0),
                        new AlignPivot(PivotState.INTAKE),
                        new SetIndexer(IndexerStates.ON, true),
                        new SetIntake(IntakeStates.ON)
                ),

                Commands.waitSeconds(0.3),

                new ParallelCommandGroup(
                        FollowChoreoTrajectory(trajectory.get(3)),
                        CommandFactory.eject(),
                        new AlignPivot(PivotState.SUBWOOFER)
                ),

                new ParallelCommandGroup(new SetShooterCommand(40), new AlignPivot(PivotState.SUBWOOFER)),
                new SetIndexer(IndexerStates.ON, false),
                Commands.waitSeconds(0.5),

                new SetIndexer(IndexerStates.OFF, false),
                new ParallelCommandGroup(new AlignPivot(PivotState.GROUND), new SetShooterCommand(0)));
    }

    public static Command Horizontal() {
        ArrayList<ChoreoTrajectory> trajectory = Choreo.getTrajectoryGroup("Horizontal Test");

        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Pose2d initialPose;
                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    initialPose = alliance.isPresent() && alliance.get() != Alliance.Red ? trajectory.get(0).getInitialPose() : trajectory.get(0).flipped().getInitialPose();
                    s_Swerve.resetOdo(initialPose);
                    System.out.println(initialPose.getX() + " " + initialPose.getY());
                }),
                FollowChoreoTrajectory(trajectory.get(0))
        );
    }

    public static Command Straight() {
        ArrayList<ChoreoTrajectory> trajectory = Choreo.getTrajectoryGroup("Straight");

        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Pose2d initialPose;
                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    initialPose = alliance.isPresent() && alliance.get() != Alliance.Red ? trajectory.get(0).getInitialPose() : trajectory.get(0).flipped().getInitialPose();
                    s_Swerve.resetOdo(initialPose);
                    System.out.println(initialPose.getX() + " " + initialPose.getY());
                }),
                FollowChoreoTrajectory(trajectory.get(0)),
                Commands.waitSeconds(0.3),
                FollowChoreoTrajectory(trajectory.get(1))
        );
    }

    public static Command Rotation() {
        ArrayList<ChoreoTrajectory> trajectory = Choreo.getTrajectoryGroup("Rotation");

        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Pose2d initialPose;
                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    initialPose = alliance.isPresent() && alliance.get() != Alliance.Red ? trajectory.get(0).getInitialPose() : trajectory.get(0).flipped().getInitialPose();
                    s_Swerve.resetOdo(initialPose);
                    System.out.println(initialPose.getX() + " " + initialPose.getY());
                }),
                FollowChoreoTrajectory(trajectory.get(0))
        );
    }

    //FIXME: IF THE AUTO MOVEMENTS ARE ONE MOVEMENT EARLY, THEN JUST INCREASE TRAJECTORY #s BY ONE OR CHANGE IN CHOREO
    //public static Command FourNoteStraightBot() {
        // ArrayList<ChoreoTrajectory> trajectory = Choreo.getTrajectoryGroup("FourNoteMinTranslationBot");
        // return new SequentialCommandGroup(
                // new InstantCommand(() -> {
                //     Pose2d initialPose;
                //     Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                //     initialPose = alliance.isPresent() && alliance.get() != Alliance.Red ? trajectory.get(0).getInitialPose() : trajectory.get(0).flipped().getInitialPose();
                //     s_Swerve.resetOdo(initialPose);
                //     System.out.println(initialPose.getX() + " " + initialPose.getY());
                // }),

                // new ParallelCommandGroup(
                //         new AlignPivot(PivotState.SUBWOOFER),
                //         new SetShooterCommand(40)
                // ),

                // new InstantCommand(() -> s_Indexer.setSpeed(0.8)),
                // Commands.waitSeconds(0.1),

                // new ParallelCommandGroup(
                //         new SetShooterCommand(0), // standby elocity later???
                //         new SetIndexer(IndexerStates.ON, true),
                //         new SetIntake(IntakeStates.ON, 1),
                //         new AlignPivot(PivotState.INTAKE),
                //         new SequentialCommandGroup(new WaitCommand(0.3), FollowChoreoTrajectory(trajectory.get(2)))
                // ),
                

                // Commands.waitSeconds(0.3),

                // new ParallelCommandGroup(
                //         FollowChoreoTrajectory(trajectory.get(1)),
                //         //TODO: make sure pivot position is right and tune velocitiesf
                //         new AlignPivot(PivotState.MIDDLE), //whatever the correct pivot is
                //         new SetShooterCommand(50) //whatever the correct velocity is
                // ),

                // new SetIndexer(IndexerStates.ON, false),
                // Commands.waitSeconds(0.5),

                // new ParallelCommandGroup(
                //         new SetShooterCommand(0),
                //         new AlignPivot(PivotState.INTAKE),
                //         new SetIndexer(IndexerStates.ON, true),
                //         new SetIntake(IntakeStates.ON, 1)
                //         new SequentialCommandGroup(new WaitCommand(0))
                // ),

                // Commands.waitSeconds(0.3),

                // new ParallelCommandGroup(new SetShooterCommand(50), new AlignPivot(PivotState.MIDDLE)),
                // new SetIndexer(IndexerStates.ON, false),
                // Commands.waitSeconds(0.5),

                // new ParallelCommandGroup(
                //         FollowChoreoTrajectory(trajectory.get(3)),

                // ),
                




                // new ParallelCommandGroup(
                //         FollowChoreoTrajectory(trajectory.get(2))

                
                // )
//         );
      //}

    /*public static Command FourNoteFarSide() {
      ArrayList<ChoreoTrajectory> trajectory = Choreo.getTrajectoryGroup("FourNoteFarSide");
        return new SequentialCommandGroup(
          new InstantCommand(() -> {
              Pose2d initialPose;
              Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
              initialPose = alliance.isPresent() && alliance.get() != Alliance.Red ? trajectory.get(0).getInitialPose() : trajectory.get(0).flipped().getInitialPose();
              s_Swerve.resetOdo(initialPose);
              System.out.println(initialPose.getX() + " " + initialPose.getY());
          }),

          new ParallelCommandGroup(
                  new AlignPivot(PivotState.SUBWOOFER),
                  new SetShooterCommand(40)
          ),

          new InstantCommand(() -> Indexer.getInstance().setSpeed(0.8)),
          // new SetIndexer(IndexerStates.ON, false), 
          Commands.waitSeconds(0.3),

          new ParallelCommandGroup(
            new SetShooterCommand(0),
            new SetIndexer(IndexerStates.ON, true),
            new SequentialCommandGroup(
              new SetIntake(IntakeStates.ON),
              new AlignPivot(30),
              RobotContainer.getInstance().eject()
            ),
            new AlignPivot(PivotState.INTAKE),
            new SequentialCommandGroup(new WaitCommand(0.3), FollowChoreoTrajectory(trajectory.get(0)))
          ),

          new SetShooterCommand(45),
          new SetIndexer(IndexerStates.ON, false),
          Commands.waitSeconds(0.3),

          new ParallelCommandGroup(
            new SetShooterCommand(0),
            new SetIndexer(IndexerStates.ON, true),
            new SequentialCommandGroup(
              new SetIntake(IntakeStates.ON, 3.5),
              new AlignPivot(30),
              RobotContainer.getInstance().eject()
            ),
            new AlignPivot(PivotState.INTAKE),
            FollowChoreoTrajectory(trajectory.get(1))
          ),

          new SetShooterCommand(45),
          new SetIndexer(IndexerStates.ON, false),
          Commands.waitSeconds(0.3),

          new ParallelCommandGroup(
            new SetShooterCommand(0),
            new SetIndexer(IndexerStates.ON, true),
            new SequentialCommandGroup(
              new SetIntake(IntakeStates.ON, 4),
              new AlignPivot(30),
              RobotContainer.getInstance().eject()
            ),
            new AlignPivot(PivotState.INTAKE),
            FollowChoreoTrajectory(trajectory.get(2))
          ),

          new SetShooterCommand(45),
          new SetIndexer(IndexerStates.ON, false),
          Commands.waitSeconds(0.3),

          new SetShooterCommand(0),
          new SetIndexer(IndexerStates.OFF, false)
        );
    }*/
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
        FourNoteSubwoofer("FourNoteSubwoofer", FourNoteSubwoofer()),
        Horizontal("Horizontal", Horizontal()),
        Straight("Straight", Straight()),
        Rotation("Rotation", Rotation()),
        //FourNoteMinTranslationMiddle("FourNoteMinTranslationMiddle", FourNoteStraightBot()),
        FourNoteFromTop("FourNoteFromTop", FourNoteFromTop()),
        TwoNoteSubwoofer("TwoNoteSubwoofer", TwoNoteSubwoofer()),
        ThreeNoteSubwooferMidTop("ThreeNoteSubwooferMidTop", ThreeNoteSubwooferMidTop()),
        ThreeNoteSubwooferMidBot("ThreeNoteSubwooferMidBot", ThreeNoteSubwooferMidBot()),
        //FirstLine("FirstLine", FirstLine()),
        TwoNoteFarSide("TwoNoteFarSide", TwoNoteFarSide()),
        FourNoteSubwooferNew("FourNoteSubwooferNew", FourNoteSubwooferNew());
        //FourNoteFarSide("FourNoteFarSide", FourNoteFarSide());




        String name;
        Command autoCommand;

        private AutoPath(String a, Command autoCommand) {
            name = a;
            this.autoCommand = autoCommand;
        }
    }

   /* public static Command FollowIntakePath(Command ChoreoDriveCommand, double pathTimeLength, double intakeRevTime) {
        return new ParallelCommandGroup(
                new SetShooterCommand(0),
                new SetIndexer(IndexerStates.ON, true, pathTimeLength + 0.5),
                new SetIntake(IntakeStates.ON, pathTimeLength),
                new AlignPivot(PivotState.INTAKE),
                new SequentialCommandGroup(new WaitCommand(intakeRevTime), ChoreoDriveCommand) //intakeRevTime = 0 for longer paths, max of around 0.3 seconds for really short paths
        );
}

    public static Command FollowShootPath(Command ChoreoDriveCommand, double shootAngle, double shooterSpeed){
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        ChoreoDriveCommand,
                        CommandFactory.eject()     
                ),

                new ParallelCommandGroup(
                        new AlignPivot(shootAngle),
                        new SetShooterCommand(shooterSpeed)
                ),

                new SetIndexer(IndexerStates.ON, false),
                Commands.waitSeconds(0.2),

                new ParallelCommandGroup(
                        new SetShooterCommand(0),
                        new SetIndexer(IndexerStates.OFF, false)
                )
        );
    }
    

    //for getting rid of stop points at intake so we dont have to waste time with wait commands
    public static Command FollowAutoCycle(Command ChoreoDriveCommand, double intakeTimeLength, double intakeRevTime, double shootAngle, double shooterSpeed){
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SetShooterCommand(0),
                        new SetIndexer(IndexerStates.ON, true, intakeTimeLength + 0.5),
                        new SetIntake(IntakeStates.ON, intakeTimeLength),
                        new AlignPivot(PivotState.INTAKE),
                        new SequentialCommandGroup(new WaitCommand(intakeRevTime), ChoreoDriveCommand )
                ),
                CommandFactory.eject(),
                new ParallelCommandGroup(
                        new AlignPivot(shootAngle),
                        new SetShooterCommand(shooterSpeed)
                ),

                new SetIndexer(IndexerStates.ON, false),
                Commands.waitSeconds(0.2),
                
                new ParallelCommandGroup(
                        new SetShooterCommand(0),
                        new SetIndexer(IndexerStates.OFF, false)
                )
        );
    }
*/


}

