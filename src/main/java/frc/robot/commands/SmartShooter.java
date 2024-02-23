// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SmartShooter extends Command {
  private final Vision s_Vision;
  private final Shooter s_Shooter;
  private final Pivot s_Pivot;
  private final CommandSwerveDrivetrain s_CommandSwerveDrivetrain;

  private double desiredYaw;
  private Pose2d initPose;
  public SmartShooter() {
    s_Vision = Vision.getInstance();
    s_Shooter = Shooter.getInstance();
    s_Pivot = Pivot.getInstance();
    s_CommandSwerveDrivetrain = CommandSwerveDrivetrain.getInstance();
    addRequirements(s_Shooter, s_Vision, s_Pivot, s_CommandSwerveDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
