// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


// import frc.robot.commands.SetLightz;
import frc.robot.subsystems.*;

import java.time.Instant;

import org.opencv.core.Point;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Indexer.IndexerStates;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Pivot.PivotState;
import frc.robot.commands.SetIndexer;
import frc.robot.commands.SmartIntake;
import frc.robot.commands.Drive.SlowDrive;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.Pivot.AlignPivot;
import frc.robot.commands.Pivot.ZeroPivot;
import frc.robot.commands.Shooter.SetShooterCommand;
import frc.robot.commands.Intake.SetIntake;


public class RobotContainer {

    private static RobotContainer container;

    public static RobotContainer getInstance(){//so i can grab controller values lol
        if(container == null){
            container = new RobotContainer();
        }
        return container;
    }
    /* Setting up bindings for necessary control of the swerve drive platform */
    public final CommandXboxController driver = new CommandXboxController(0); // Driver joystick
    private final CommandXboxController operator = new CommandXboxController(1); //Operator joystick

    private final Amp s_Amp = Amp.getInstance();
    private final Climb s_Climb = Climb.getInstance();
    private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance(); // Drivetrain
    private final Indexer s_Indexer = Indexer.getInstance();
    private final Intake s_Intake = Intake.getInstance();
    private final Pivot s_Pivot = Pivot.getInstance();
    private final Shooter s_Shooter = Shooter.getInstance();
    private final Vision s_Vision = Vision.getInstance();

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.MaxSpeed * translationDeadband).withRotationalDeadband(Constants.MaxAngularRate * rotDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

    public static final double translationDeadband = 0.1;
    public static final double rotDeadband = 0.1;

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

    public CommandXboxController getDriverController(){
        return driver;
    }

    private void configureBindings() {

        /*
         * Mechanism bindings
         */

        driver.a().onTrue(CommandFactory.offEverything()); //FINAL
        driver.x().onTrue(new SmartIntake()); //FINAL
        driver.b().onTrue(CommandFactory.eject()); //FINAL
        driver.y().whileTrue(new SetIndexer(IndexerStates.SHOOTING)); //FINAL

        // driver.a().onTrue(new SetIndexer(IndexerStates.ON, false));
        // driver.b().onTrue(new SetIndexer(IndexerStates.OFF, false));
            

        // driver.rightTrigger().onTrue(shootSubwoofer()); //FINAL
        // driver.leftTrigger().onTrue(CommandFactory.autoShootSequence()); //automatic shooting, includes alignment
        driver.leftTrigger().whileTrue(new SlowDrive());
        // driver.leftTrigger().onTrue(new PureAlignment());
        // driver.leftTrigger().onTrue(new VisionAlign());

        driver.rightBumper().onTrue(CommandFactory.shootSubwooferPrep());
        driver.rightTrigger().onTrue(CommandFactory.SubwooferShootSequence());
        driver.leftBumper().onTrue(new SetShooterCommand(45));
        // driver.leftBumper().onTrue(onIntake());

        driverDpadDown.onTrue(new AlignPivot(PivotState.GROUND)); //FINAL
        driverDpadUp.onTrue(new AlignPivot(PivotState.SUBWOOFER)); //FINAL
        driverDpadLeft.onTrue(CommandFactory.ampShootSequence()); 
        driverDpadRight.onTrue(new ZeroPivot()); //FINAL

        
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
         * Operator bindings
         */
        // operator.b().onTrue(offEverything());
        operator.a().onTrue(CommandFactory.offEverything());
        operator.x().onTrue(new SetIntake(IntakeStates.OFF)); //FINAL
        operator.b().onTrue(CommandFactory.eject()); //FINAL
        operator.y().whileTrue(new SetIndexer(IndexerStates.OFF));

        operator.povLeft().onTrue(new ZeroPivot());

        // operator.rightTrigger().whileTrue(new ManualClimb(true));
        // operator.leftTrigger().whileTrue(new ManualClimb(false));

        operator.rightBumper().onTrue(CommandFactory.Diagnostic());

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
}