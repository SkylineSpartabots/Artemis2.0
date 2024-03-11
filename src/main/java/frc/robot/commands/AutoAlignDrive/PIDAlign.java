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
      // s_Swerve.updateOdometryByVision(); //since you're supposed to have vision target, reset odometry using kalman first
      currentYaw = pose.getRotation().getRadians(); // OMG I FOUND THE PROBLEM I THINK!!
      if (currentYaw < 0) {  currentYaw = Math.PI + Math.abs(currentYaw); };
    } catch (Exception e) {}
// 
    SmartDashboard.putNumber("Current yaw", currentYaw);

    desiredYaw = Normalize();
    // if (Math.abs(desiredYaw - (Math.PI*2)) < desiredYaw) { desiredYaw = desiredYaw - (Math.PI*2);};
    // desiredYaw = Math.min(desiredYaw, Math.PI*2 - desiredYaw);

  //disregards current and reference, direction is already accounted for so it is always optimal turning
    double rotationSpeed = alignPID.calculate(currentYaw, desiredYaw);

    s_Swerve.setControl(drive.withRotationalRate(rotationSpeed));
    SmartDashboard.putNumber("final yaw", desiredYaw);
    SmartDashboard.putNumber("rotation speed", rotationSpeed);
  }

  public void updateDesiredYaw(){
    Pose2d pose = s_Swerve.getPose();
    Point currentLocation = new Point(pose.getTranslation().getX() , pose.getTranslation().getY());
    Point translatedPoint = new Point(desiredPoint.x - currentLocation.x , desiredPoint.y - currentLocation.y); // thanks Dave Yoon gloobert

    desiredYaw = Math.atan2(translatedPoint.y,translatedPoint.x) + Math.PI;
  }

  public double Normalize(){ //counterclockwise is positive
    double error = desiredYaw - currentYaw;
    if (Math.abs(error) > Math.PI) { // flip desiredYaw to its corresponding angle on the opposite side if raw error takes the longer path
      desiredYaw += (desiredYaw > currentYaw) ? -2 * Math.PI : 2 * Math.PI; //if true do first one if false do other
    }

    //go back to -pi to pi
    if (desiredYaw >= Math.PI) {
      desiredYaw -= 2 * Math.PI;
  }

  return desiredYaw;
  }

  @Override // rah rah rahh
  public void end(boolean interrupted) {
    s_Swerve.setControl(drive.withRotationalRate(0));
  }

  @Override
  public boolean isFinished() {
    return Math.abs(desiredYaw - currentYaw) < (Math.PI/(180/2));
  }

}

// robot's forward is posotive x, right is negetive y, circle is 180 to -180, 3, -3
// offset yaw, current yaw, desired yaw, rotation speed

// if(currentYaw < 0 && desiredYaw < 0){
    //   return currentYaw - desiredYaw;
    // } else if (currentYaw > 0 && desiredYaw > 0){
    //   return desiredYaw - currentYaw;
    // } else if (currentYaw > 0 && desiredYaw < 0){
    //   if(2*Math.PI - currentYaw - Math.abs(desiredYaw) < currentYaw + Math.abs(desiredYaw)){
    //     return 2*Math.PI - currentYaw - Math.abs(desiredYaw);
    //   } else {
    //     return -(currentYaw + Math.abs(desiredYaw));
    //   }
    // } else {
    //   if(2*Math.PI - desiredYaw - Math.abs(currentYaw) < desiredYaw + Math.abs(currentYaw)){
    //     return -(2*Math.PI - desiredYaw - Math.abs(currentYaw));
    //   } else {
    //     return desiredYaw + Math.abs(currentYaw);
    //   }
    // }

    