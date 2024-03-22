// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PIDAlign extends Command {

  private final CommandSwerveDrivetrain s_Swerve;
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
  // PIDController alignPID = new PIDController(1.7, 0.95, 0.05); // TODO: tune this MORE
  PIDController alignPID = new PIDController(0.5, 0, 0);

  private CommandXboxController driver = RobotContainer.getInstance().getDriverController();

  // private double currentYaw;
  private Rotation2d desiredYaw;

  private double lastXVel;
  private double lastYVel;

  // private boolean isClockwise;
  Point desiredPoint;

  public PIDAlign(Point desiredPoint) {
    s_Swerve = CommandSwerveDrivetrain.getInstance();

    this.desiredPoint = desiredPoint;

    addRequirements(s_Swerve);
  }

  public PIDAlign(){
    s_Swerve = CommandSwerveDrivetrain.getInstance();
    addRequirements(s_Swerve);
  }

  @Override
  public void initialize() {
    alignPID.reset();
    lastXVel = 0.0;
    lastYVel = 0.0;
  }

  @Override
  public void execute() { // at some point, make it so we can call this while moving

    Pose2d pose = s_Swerve.getPose();
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

    double xVel = -driver.getLeftY() * Constants.MaxSpeed;
    double yVel = -driver.getLeftX() * Constants.MaxSpeed;


    // try {
    //   // s_Swerve.updateOdometryByVision(); can someon fix vison please thanks
    //   predictedPose = pose;
    //   Transform2d predictionTransform = new Transform2d();
    //   double refreshLoopPeriod = 0.02; //seconds
    //   double realXVel = 0.5 * (lastXVel + xVel); //an attempt to account for acceleration by accounting for last vel vs. current desired vel
    //   double realYVel = 0.5 * (lastYVel + yVel); 
    //   if(Math.sqrt(Math.pow(realXVel, 2) + Math.pow(realYVel, 2)) < 6){
    //     if(realXVel > RobotContainer.translationDeadband * Constants.MaxSpeed){
    //       predictionTransform.plus(new Transform2d(realXVel*refreshLoopPeriod, 0.0, new Rotation2d()));
    //     }
    //     if(realYVel > RobotContainer.translationDeadband * Constants.MaxSpeed){
    //       predictionTransform.plus(new Transform2d(0.0, realYVel*refreshLoopPeriod, new Rotation2d()));
    //     }
    //     predictedPose.transformBy(predictionTransform);
    //   }
    desiredYaw = PhotonUtils.getYawToPose(pose, Vision.aprilTagFieldLayout.getTagPose(alliance.isPresent() && alliance.get() != Alliance.Red ? 7 : 4).get().toPose2d());
    SmartDashboard.putNumber("desiredYaw", desiredYaw.getRotations());

    double rotationSpeed = alignPID.calculate(desiredYaw.getRotations(), 0);
    s_Swerve.setControl(drive.withRotationalRate(rotationSpeed).withVelocityX(xVel).withVelocityY(yVel));
    lastXVel = xVel;
    lastYVel = yVel;

  }

  public void updateDesiredYaw() {
    Pose2d pose = s_Swerve.getPose();
    Point currentLocation = new Point(pose.getTranslation().getX(), pose.getTranslation().getY());
    Point translatedPoint = new Point(desiredPoint.x - currentLocation.x, desiredPoint.y - currentLocation.y);
    // desiredYaw = Math.atan2(translatedPoint.y, translatedPoint.x);
  }

  // public double checkRoute() {
  //   double rawError = desiredYaw - currentYaw;
  //   if (Math.abs(rawError) > Math.PI) {
  //     desiredYaw -= Math.signum(desiredYaw) * Math.PI * 2;
  //   }
  //   // 🍔 im going insane :(
  //   return desiredYaw;
  // }

  @Override
  public void end(boolean interrupted) {
    s_Swerve.setControl(drive.withRotationalRate(0));
    SmartDashboard.putBoolean("Align Running", false);
  }

  private double minimumToleranceAngle = 3;

  @Override
  public boolean isFinished() {
    return false;
    // return Math.abs(desiredYaw.getRotations() - s_Swerve.getPose().getRotation().getRotations()) < (minimumToleranceAngle/360) * 100;
  }
}