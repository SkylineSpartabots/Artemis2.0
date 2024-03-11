package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SetShooterCommand extends Command {
    Shooter s_Shooter;

    private double velBot;
    private double velTop;

    public SetShooterCommand(double velocity) {
        s_Shooter = Shooter.getInstance();
        velBot = velocity;
        velTop = velocity;

        addRequirements(s_Shooter);
    }

    public SetShooterCommand(double velocityTop, double velocityBot){
        s_Shooter = Shooter.getInstance();
        velTop = velocityTop;
        velBot = velocityBot;

        addRequirements(s_Shooter);
    }

    @Override
    public void initialize() {
        System.out.println("SHOOT");
        s_Shooter.setTopVelocity(velTop);
        s_Shooter.setBotVelocity(velBot);
    }

    @Override
    public void execute() {

    }

    @Override
	public boolean isFinished() {
        double[] speeds = s_Shooter.getBothSpeeds();
        return Math.abs(speeds[0] - velTop) < 3 && Math.abs(speeds[1] - velBot) < 3;
	}
		
	@Override
	public void end(boolean interrupted) {
	}
}
