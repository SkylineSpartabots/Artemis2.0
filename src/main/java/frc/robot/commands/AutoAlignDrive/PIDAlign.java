// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoAlignDrive;

import java.beans.Visibility;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;


public class PIDAlign extends Command {

  private final CommandSwerveDrivetrain s_Drive;
  private final Vision s_Vision;

  PIDController alignPID = new PIDController(0, 0, 0);
  //TODO: get real pid values

  /** Creates a new PIDAlign. */
  public PIDAlign() {
    s_Vision = Vision.getInstance();
    s_Drive = CommandSwerveDrivetrain.getInstance();
    addRequirements(s_Vision);
    addRequirements(s_Drive);
    // Use addRequirements() here to declare subsystem dependencies.
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
