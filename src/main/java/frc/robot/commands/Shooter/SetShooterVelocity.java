package frc.robot.commands.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SetShooterVelocity extends Command {
    Shooter s_Shooter;
    Double desiredVelocity;

    PIDController topShooterController = new PIDController(0.25, 0.05, 0.0);
    PIDController botShooterController = new PIDController(0.25, 0.05, 0.0);

    public SetShooterVelocity(double desiredVelocity) {
        s_Shooter = Shooter.getInstance();
        this.desiredVelocity = desiredVelocity;

        addRequirements(s_Shooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        double topVoltage = topShooterController.calculate(s_Shooter.getBothSpeeds()[0], desiredVelocity);
        double botVoltage = botShooterController.calculate(s_Shooter.getBothSpeeds()[1], desiredVelocity);
        s_Shooter.setTopVoltage(topVoltage);
        s_Shooter.setBotVoltage(botVoltage);
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
