// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class SetClimb extends Command {
    
    private final Climb s_Climb;
    private double direction;
    private double speed = 0.5;

    public SetClimb(double direction) {
        s_Climb = Climb.getInstance();
        this.direction = direction;
        addRequirements(s_Climb);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        Timer failsafe = new Timer();
        failsafe.start();
        
        s_Climb.setIsPeaked(false);
        while ((s_Climb.getIsPeaked() != true) && failsafe.get() <= 10) {
            s_Climb.setSpeed(speed * direction);
        }

        s_Climb.setSpeed(0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        s_Climb.setSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}