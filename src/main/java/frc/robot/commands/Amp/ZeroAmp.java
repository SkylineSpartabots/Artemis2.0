// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Amp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Amp;

public class ZeroAmp extends Command {
  Amp s_Amp;
  private double currentThreshold = 7;
  public ZeroAmp() {
    s_Amp = Amp.getInstance();

    addRequirements(s_Amp);
  }

  @Override
  public void initialize() {
    s_Amp.setSpeed(-0.04);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Amp.setSpeed(0);
    s_Amp.resetMotorEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_Amp.getMotorCurrent() > currentThreshold;
  }
}
