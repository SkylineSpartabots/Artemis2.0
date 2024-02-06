package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeShooter;

public class SmartIntake extends Command{

    private final IntakeShooter s_IntakeShooter;

    public SmartIntake(){
        s_IntakeShooter = IntakeShooter.getInstance();
        addRequirements(s_IntakeShooter);
    }

    @Override
    public void initialize() {
        s_IntakeShooter.setIntake(0.7);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        s_IntakeShooter.setIntake(0);
    }

    @Override
    public boolean isFinished() {
        return s_IntakeShooter.hasNote();
    }
}
