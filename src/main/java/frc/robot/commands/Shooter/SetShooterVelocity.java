package frc.robot.commands.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SetShooterVelocity extends Command {
    Shooter s_Shooter;
    Double desiredVelocity;

    PIDController topShooterController = new PIDController(0.05, 0.0, 0.0);
    PIDController botShooterController = new PIDController(0.05, 0.0, 0.0);

    public SetShooterVelocity(double desiredVelocity) {
        s_Shooter = Shooter.getInstance();
        this.desiredVelocity = desiredVelocity;

        addRequirements(s_Shooter);
    }

    @Override
    public void initialize() {
        // s_Shooter.setTopSetpoint(desiredVelocity);
        // s_Shooter.setBotSetpoint(desiredVelocity);
    }

    @Override
    public void execute() {
        s_Shooter.setTopVoltage(desiredVelocity);
        s_Shooter.setBotVoltage(desiredVelocity);
    }


    @Override
	public boolean isFinished() {
        return false;
	}
		
	@Override
	public void end(boolean interrupted) {
        s_Shooter.setVoltage(0.0);
	}
}
