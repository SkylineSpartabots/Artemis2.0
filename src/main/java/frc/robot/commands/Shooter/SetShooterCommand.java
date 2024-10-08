package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Lights.ledModes;
import frc.robot.subsystems.Lights;

public class SetShooterCommand extends Command {
    Shooter s_Shooter;
    Lights s_Lights;

    private double velBot;
    private double velTop;

    public SetShooterCommand(double velocity) {
        s_Shooter = Shooter.getInstance();
        s_Lights = Lights.getInstance();

        velBot = velocity;
        velTop = velocity;

        addRequirements(s_Shooter, s_Lights);
    }

    public SetShooterCommand(double velocityTop, double velocityBot){
        s_Shooter = Shooter.getInstance();
        s_Lights = Lights.getInstance();

        velTop = velocityTop;
        velBot = velocityBot;

        addRequirements(s_Shooter, s_Lights);
    }

    @Override
    public void initialize() {
        System.out.println("SHOOT");
        s_Shooter.setTopVelocity(velTop);
        s_Shooter.setBottomVelocity(velBot);
        
        s_Lights.setLEDs(ledModes.ShooterRamping);
    }

    @Override
    public void execute() {

    }

    @Override
	public boolean isFinished() {
        double[] speeds = s_Shooter.getBothVelocities();
        return Math.abs(speeds[0] - velTop) < 3 && Math.abs(speeds[1] - velBot) < 3;
	}
		
	@Override
	public void end(boolean interrupted) {
        s_Lights.setLEDs(ledModes.ShooterAtSpeed);
	}
}
