// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class SetClimb extends Command {
    
    private final Climb s_Climb;
    private double direction = 1; //1 is up, -1 is down

    public SetClimb() {
        s_Climb = Climb.getInstance();
        addRequirements(s_Climb);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        s_Climb.setClimbSpeed(0.5 * direction);
        if (s_Climb.isPeaked() == true) {
            direction = -1 * direction;
            s_Climb.setClimbSpeed(s_Climb.getClimbSpeed() * direction);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (s_Climb.isPeaked() && (direction == -1));
    }
}