// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ZeroShooter extends Command {
    Shooter s_Shooter;
    private Timer timer;

  public ZeroShooter() {
    s_Shooter = Shooter.getInstance();
    addRequirements(s_Shooter);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Shooter.setTopVelocity(-0.1);
    s_Shooter.setBotVelocity(-0.1);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Shooter.setTopVelocity(0);
    s_Shooter.setBotVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //double speeds[] = s_Shooter.getBothSpeeds();
    //return speeds[0] < 3 && speeds[1] < 3;
    return timer.hasElapsed(1);
  }
}
