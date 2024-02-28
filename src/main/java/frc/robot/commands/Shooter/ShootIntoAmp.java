package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.TeleopAutomation.IndexForShooting;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootIntoAmp extends Command {

    private final Shooter s_Shooter;
    Intake.IntakeStates state;

    public ShootIntoAmp(Intake.IntakeStates state) {
        s_Shooter = Shooter.getInstance();
        this.state = state;
        addRequirements(s_Shooter);
    }

    @Override
    public void initialize() {
        s_Shooter.setTopVelocity(500);
        s_Shooter.setBotVelocity(1500);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        Command cmd = new IndexForShooting();
        cmd.schedule();
    }

    @Override
    public boolean isFinished() {
        return s_Shooter.velocitiesWithinError(75);
    }
}