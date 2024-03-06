// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


// import frc.robot.commands.SetLightz;
import frc.robot.subsystems.*;
import org.opencv.core.Point;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Indexer.IndexerStates;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Pivot.PivotState;
import frc.robot.commands.FixIndexer;
import frc.robot.commands.ManualIndexForShooting;
import frc.robot.commands.ReverseIndexer;
import frc.robot.commands.SetIndexer;
import frc.robot.commands.SmartIntake;
import frc.robot.commands.TeleopFactory;
import frc.robot.commands.Pivot.SetPivot;
import frc.robot.commands.Pivot.ZeroPivot;
import frc.robot.commands.Shooter.SetShooterCommand;
import frc.robot.commands.Shooter.ShootIntoAmp;
import frc.robot.commands.Shooter.Swing;
import frc.robot.commands.TeleopAutomation.IndexForShooting;
import frc.robot.commands.AutoAlignDrive.PIDAlign;
import frc.robot.commands.Climb.ManualClimb;
import frc.robot.commands.Intake.SetIntake;

public class RobotContainer {
    //robot testing states
    private boolean shooterSysIDTuned = false;

    //private final Vision s_Vision = Vision.getInstance();
    private final Shooter s_Shooter = Shooter.getInstance();
    private final Indexer s_Indexer = Indexer.getInstance();
    private final Intake s_Intake = Intake.getInstance();
    private final Pivot s_Pivot = Pivot.getInstance();
    private final Climb s_Climb = Climb.getInstance();
    private final Amp s_Amp = Amp.getInstance();
    // private final Lightz s_lightz = Lightz.getInstance();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final CommandXboxController driver = new CommandXboxController(0); // My joystick
    private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance(); // My drivetrain

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.MaxSpeed * 0.1).withRotationalDeadband(Constants.MaxAngularRate * 0.1) // Add a 20% deadband, tune to driver preference
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
    // driving in open loop
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(Constants.MaxSpeed);

    /* Driver Buttons */
    private final Trigger driverBack = driver.back();
    private final Trigger driverStart = driver.start();
    private final Trigger driverA = driver.a();
    private final Trigger driverB = driver.b();
    private final Trigger driverX = driver.x();
    private final Trigger driverY = driver.y();
    private final Trigger driverRightBumper = driver.rightBumper();
    private final Trigger driverLeftBumper = driver.rightBumper();
    private final Trigger driverLeftTrigger = driver.leftTrigger();
    private final Trigger driverRightTrigger = driver.rightTrigger();
    private final Trigger driverDpadUp = driver.povUp();
    private final Trigger driverDpadDown = driver.povDown();
    private final Trigger driverDpadLeft = driver.povLeft();
    private final Trigger driverDpadRight = driver.povRight();

    private void configureBindings() {

        /*
         * Mechanism bindings
         */

        driver.a().onTrue(offEverything());
        driver.x().onTrue(new SmartIntake());
        // driver.b().whileTrue(eject());
        driver.b().onTrue(new ShootIntoAmp());
        driver.y().whileTrue(new ManualIndexForShooting());

        driver.rightTrigger().onTrue(shootSubwoofer());

        driver.rightBumper().whileTrue(new ManualClimb(true));
        driver.leftBumper().whileTrue(new ManualClimb(false));

        driverDpadDown.onTrue(new SetPivot(PivotState.GROUND));
        driverDpadUp.onTrue(new SetPivot(PivotState.FARWING));
        driverDpadLeft.onTrue(new SetPivot(PivotState.AMP));
        driverDpadRight.onTrue(new ZeroPivot());

        /*
         * Drivetrain bindings
         */
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * Constants.MaxSpeed) // Drive forward with
                        // negative Y (forward)
                        .withVelocityY(-driver.getLeftX() * Constants.MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driver.getRightX() * Constants.MaxAngularRate) // Drive counterclockwise with negative X (left)
                ));

        // reset the field-centric heading. AKA reset odometry
        driverBack.onTrue(new InstantCommand(() -> drivetrain.resetOdo(new Pose2d(0, 0, new Rotation2d()))));

        /*
         * simulation bindings
         */
        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public RobotContainer() {

        configureBindings();
    }

    // public Command setLEDs(){
    //     return new SetLightz(Lightz.ledModes.BLUE);
    // }

    // private boolean shooterOn() {
    //     return s_Shooter.getBotMotorVoltage() > 0 || s_Shooter.getTopMotorVoltage() > 0;
    // }

    private Command shooterOff() {
        return new SetShooterCommand(0);
    } 

    private Command shooterOn() {
        return new SetShooterCommand(2000);
    }

    private boolean indexerOn() {
        return s_Indexer.getMotorVoltage() > 0;
    }

    private boolean intakeOn() {
        return s_Intake.getMotorVoltage() > 0;
    }

    public Command setShooterVelocity(double velocity){
        return new InstantCommand(() -> s_Shooter.setVelocity(velocity));
    }

    public Command offEverything(){
        return new SequentialCommandGroup(new ParallelCommandGroup(setShooterVelocity(0), offIndexer(), offIntake()), new SetPivot(PivotState.GROUND));
    }

    public Command aligntoCordinate(Point point) {
        return new PIDAlign(point);
    }

    public Command onIntake() {
        return new SetIntake(IntakeStates.ON);
    }

    public Command offIntake() {
        return new SetIntake(IntakeStates.OFF);
    }

    public Command indexToShooter(){
        return new SetIndexer(IndexerStates.ON, false);
    }

    //shooter
    public Command onIndexer() {
        return new SequentialCommandGroup(new SetIndexer(IndexerStates.ON, true), new FixIndexer());
    }

    public Command shootAmp(){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SetPivot(PivotState.AMP),
                new SetShooterCommand(2000, 1200))
        );
    }

    public Command shootSubwoofer(){
        if(shooterSysIDTuned){
            return new ParallelCommandGroup(new SetPivot(PivotState.SUBWOOFER), new SetShooterCommand(33));
        } else {
            return new ParallelCommandGroup(new SetPivot(PivotState.SUBWOOFER), new InstantCommand(() -> s_Shooter.setVoltage(8)));
        }
    }

    public Command eject(){
        return new SequentialCommandGroup(new SetPivot(PivotState.GROUND), new WaitCommand(0.3), new ReverseIndexer());
    }
    
    public Command offIndexer() {
        return new SetIndexer(IndexerStates.OFF, false);
    }


}