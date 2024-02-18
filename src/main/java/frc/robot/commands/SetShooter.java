
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SetShooter extends Command {
    private final Shooter s_Shooter;
    double finalSpeed;

    public SetShooter(Shooter.ShooterStates state) { //change from off or max speed
        s_Shooter = Shooter.getInstance();
        finalSpeed = state.getValue();
        addRequirements(s_Shooter);
    }

    public SetShooter(double difference) { //increase or decrease speed. Makes sure not to increase above max or decrease below 0
        s_Shooter = Shooter.getInstance();

        double addedSpeed = s_Shooter.getSpeed() + difference;

        if (addedSpeed <= 1 && addedSpeed >= 0) {
            finalSpeed = addedSpeed;
        }

        addRequirements(s_Shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        s_Shooter.setSpeed(finalSpeed);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}