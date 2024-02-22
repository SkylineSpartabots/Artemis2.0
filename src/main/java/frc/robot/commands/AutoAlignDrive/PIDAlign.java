// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoAlignDrive;

import java.beans.Visibility;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;


public class PIDAlign extends Command {

  private final CommandSwerveDrivetrain s_Swerve;
  private final Vision s_Vision;
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
  PIDController alignPID = new PIDController(0, 0, 0);
  private double currentYaw;
  private double desiredYaw;

  public PIDAlign() {
    s_Vision = Vision.getInstance();
    s_Swerve = CommandSwerveDrivetrain.getInstance();

    desiredYaw = s_Vision.getBestTarget().getYaw(); //This must be radians

    addRequirements(s_Vision);
    addRequirements(s_Swerve);
  }

  @Override
  public void initialize() {
    alignPID.reset();
  }

  @Override
  public void execute() {

    try {
      currentYaw = s_Vision.calculatePoseFromVision().getRotation().toRotation2d().getRadians();
    } catch (Exception e) {} //IF WE DONT SEE NUFFIN WE DONT DO NUFFIN!!!! 🦅🦅🦅

    double errorYaw = currentYaw - desiredYaw;

    double rotationSpeed = alignPID.calculate(errorYaw, desiredYaw);


    s_Swerve.applyRequest(() -> drive.withRotationalRate(rotationSpeed));
  }

  @Override
  public void end(boolean interrupted) {
    s_Swerve.applyRequest(() -> drive.withRotationalRate(0));
  }

  @Override
  public boolean isFinished() {
    return Math.abs(desiredYaw - currentYaw) < 2; // eor of too degwees fo now
  }
}
