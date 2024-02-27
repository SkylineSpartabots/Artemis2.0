// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoAlignDrive;

import java.beans.Visibility;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

import org.opencv.core.Point;
import org.photonvision.targeting.PhotonTrackedTarget;


public class PIDAlign extends Command {

  private final CommandSwerveDrivetrain s_Swerve;
  private final Vision s_Vision;
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
  PIDController alignPID = new PIDController(0, 0, 0); //TODO: tune this
  

  private double currentYaw;
  private double desiredYaw;
  Point desiredPoint;
  double offsetYaw;


  public PIDAlign(Point desiredPoint) {
    s_Vision = Vision.getInstance();
    s_Swerve = CommandSwerveDrivetrain.getInstance();

    this.desiredPoint = desiredPoint;

    addRequirements(s_Vision);
    addRequirements(s_Swerve);

  }
  
  @Override
  public void initialize() {
    alignPID.reset();

    Pose2d pose = s_Swerve.getPoseByOdometry();
    Point currentLocation = new Point(pose.getTranslation().getX() , pose.getTranslation().getY());
    Point translatedPoint = new Point(desiredPoint.x - currentLocation.x , desiredPoint.y - currentLocation.y); // thanks Dave gloobert

    offsetYaw = Math.toRadians(Math.tanh(translatedPoint.y/translatedPoint.x));
  }

  @Override
  public void execute() {

    Pose2d pose = s_Swerve.getPoseByOdometry();

    try {
      s_Swerve.updateOdometryByVision(); //since you're supposed to have vision target, reset odometry using kalman first
      currentYaw = pose.getRotation().getRadians(); //grab the "accurate" odometry
    } catch (Exception e) {}

    
    desiredYaw = offsetYaw + currentYaw;

    double rotationSpeed = alignPID.calculate(currentYaw, desiredYaw);

    s_Swerve.applyRequest(() -> drive.withRotationalRate(rotationSpeed));
  }

  @Override
  public void end(boolean interrupted) {
    s_Swerve.applyRequest(() -> drive.withRotationalRate(0));
  }

  @Override
  public boolean isFinished() {
    return Math.abs(desiredYaw - currentYaw) < 3; // error of three degrees for now, set later
  }
}
