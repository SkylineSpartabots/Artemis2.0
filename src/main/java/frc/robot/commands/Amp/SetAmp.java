// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Amp;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Amp;
import frc.robot.subsystems.Amp.AmpState;

public class SetAmp extends Command {
  Amp s_Amp;

  double desiredPosition;
  AmpState ampState;

  PIDController AmpController = new PIDController(0.4, 0, 0);
  public SetAmp(AmpState state) {
    s_Amp = Amp.getInstance();
    this.ampState = state;
    addRequirements(s_Amp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    desiredPosition = ampState.getPos();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double voltage;
    voltage = AmpController.calculate(s_Amp.getPosition(), desiredPosition);
    s_Amp.setVoltage(voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Amp.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(desiredPosition - s_Amp.getPosition()) < 0.03;
  }
}
