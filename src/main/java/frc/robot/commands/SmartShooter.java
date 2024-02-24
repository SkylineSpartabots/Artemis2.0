// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Pivot;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SmartShooter extends Command {
  private final Vision s_Vision;
  private final Shooter s_Shooter;
  private final Pivot s_Pivot;
  private final CommandSwerveDrivetrain s_Swerve;
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
  PIDController alignPID = new PIDController(0, 0, 0); //needs to be tuned
  PIDController CANController = new PIDController(50, 12, 0);
  private PhotonTrackedTarget targetTag;
  private double desiredYaw;
  private double currentYaw;
  private double distance;
  public SmartShooter() {
    s_Vision = Vision.getInstance();
    s_Shooter = Shooter.getInstance();
    s_Pivot = Pivot.getInstance();
    s_Swerve = CommandSwerveDrivetrain.getInstance();
    addRequirements(s_Shooter, s_Vision, s_Pivot, s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    alignPID.reset();
    targetTag = s_Vision.getSpeakerTarget();
    desiredYaw = targetTag.getYaw();
    distance = PhotonUtils.calculateDistanceToTargetMeters(
    Constants.Vision.cameraHeight, 
    Constants.Vision.aprilTagHeight, 
    Constants.Vision.cameraPitchOffset, 
    Units.degreesToRadians(targetTag.getPitch()));
    s_Pivot.setCalculatedSetpoint(Constants.getAngleForDistance(distance));
    CANController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((Math.abs(desiredYaw - currentYaw) < 3) != true){
    try {
      s_Swerve.updateOdometryByVision();
      currentYaw = s_Swerve.getPoseByOdometry().getRotation().getRadians();
    } catch (Exception e) {}

    double rotationSpeed = alignPID.calculate(currentYaw, desiredYaw);

    if (targetTag.getFiducialId() == Constants.Vision.AprilTags.redSpeakerCenter || targetTag.getFiducialId() == Constants.Vision.AprilTags.blueSpeakerCenter){
      s_Swerve.applyRequest(() -> drive.withRotationalRate(rotationSpeed));
    }
  } else{
    s_Swerve.applyRequest(() -> drive.withRotationalRate(0));
  }

    if((Math.abs(s_Pivot.getCalculatedSetPoint() - s_Pivot.getCANcoderAbsolutePosition()) < 0.005) != true){
    double voltage;
    if (s_Pivot.CANcoderWorking()) {
      voltage = CANController.calculate(s_Pivot.getCANcoderAbsolutePosition(), s_Pivot.getCalculatedSetPoint());
    }
    else {
      voltage = 0;
    }
        // if (Math.abs(s_Pivot.getCANcoderPosition() - s_Pivot.getSetPoint()) < 15) {
    // 	voltage = 0.7;
    // }
    s_Pivot.setVoltage(voltage);
        } else{
          s_Pivot.stopMotor();
        }

    if((Math.abs(s_Pivot.getCalculatedSetPoint() - s_Pivot.getCANcoderAbsolutePosition()) < 0.005) && (Math.abs(desiredYaw - currentYaw) < 3)){
      s_Shooter.setVelocity(Constants.getVelocityForDistance(distance));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Swerve.applyRequest(() -> drive.withRotationalRate(0));
    s_Pivot.stopMotor();
    s_Shooter.setVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //put boolean for if our robot is still holding a note
  }
}
