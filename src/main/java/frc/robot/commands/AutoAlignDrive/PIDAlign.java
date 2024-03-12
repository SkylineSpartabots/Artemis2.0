// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoAlignDrive;


import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.photonvision.targeting.PhotonTrackedTarget;


public class PIDAlign extends Command {

  private final CommandSwerveDrivetrain s_Swerve;
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
  PIDController alignPID = new PIDController(1.7, 0.95, 0.05); //TODO: tune this MORE
  
  private double currentYaw;
  private double desiredYaw;

  // private boolean isClockwise;
  Point desiredPoint;
  double offsetYaw;

  public PIDAlign(Point desiredPoint) {
    s_Swerve = CommandSwerveDrivetrain.getInstance();

    this.desiredPoint = desiredPoint;

    addRequirements(s_Swerve);
  }
  
  @Override
  public void initialize() {
    alignPID.reset();
    updateDesiredYaw();
  }
    

  @Override
  public void execute() { //at some point, make it so we can call this while moving

    Pose2d pose = s_Swerve.getPose();

    try {
      // s_Swerve.updateOdometryByVision(); can someon fix vison please thanks
      currentYaw = pose.getRotation().getRadians();
      if (currentYaw < 0) {  currentYaw = Math.PI + Math.abs(currentYaw); };
    } catch (Exception e) {}

    SmartDashboard.putNumber("Current yaw", currentYaw);

    desiredYaw = Normalize();

    double rotationSpeed = alignPID.calculate(currentYaw, desiredYaw);

    s_Swerve.setControl(drive.withRotationalRate(rotationSpeed));
    SmartDashboard.putNumber("final yaw", desiredYaw);
    SmartDashboard.putNumber("rotation speed", rotationSpeed);
  }

  public void updateDesiredYaw(){
    Pose2d pose = s_Swerve.getPose();
    Point currentLocation = new Point(pose.getTranslation().getX() , pose.getTranslation().getY());
    Point translatedPoint = new Point(desiredPoint.x - currentLocation.x , desiredPoint.y - currentLocation.y);

    desiredYaw = Math.atan2(translatedPoint.y,translatedPoint.x);
    if (desiredYaw < 0) {  desiredYaw = Math.PI + Math.abs(desiredYaw); }
  }

  public double Normalize() {
    double error = desiredYaw - currentYaw;
    if (Math.abs(error) > Math.PI) {
      desiredYaw += (desiredYaw > currentYaw) ? -2 * Math.PI : 2 * Math.PI;
    }

    if (desiredYaw >= Math.PI) {
      desiredYaw -= 2 * Math.PI;
  }

  return desiredYaw;
  }
  @Override
  public void end(boolean interrupted) {
    s_Swerve.setControl(drive.withRotationalRate(0));
  }

  @Override
  public boolean isFinished() {
    return Math.abs(desiredYaw - currentYaw) < (Math.PI/(180/2));
  }

}