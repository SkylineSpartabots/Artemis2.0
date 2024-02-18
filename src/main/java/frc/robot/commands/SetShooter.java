
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SetShooter extends Command {
    private final Shooter s_Shooter;
    double finalSpeed;
    Boolean motorLoc = true;

    /**
     * @param MotorLocation
     * true = top motor
     * false = bottom motor
     * null = both
     */
    public SetShooter(Shooter.ShooterStates state, Boolean MotorLocation) { //change from off or max speed
        s_Shooter = Shooter.getInstance();
        finalSpeed = state.getValue();
        motorLoc = MotorLocation;
        addRequirements(s_Shooter);
    }

    /**
     * @param MotorLocation
     * true = top motor
     * false = bottom motor
     * null = both
     */
    public SetShooter(double difference, Boolean MotorLocation) { //increase or decrease speed. Makes sure not to increase above max or decrease below 0
        s_Shooter = Shooter.getInstance();

        double addedSpeed = s_Shooter.getSpeed(MotorLocation) + difference;

        if (addedSpeed <= 1 && addedSpeed >= 0) {
            finalSpeed = addedSpeed;
            motorLoc = MotorLocation;
        }

        addRequirements(s_Shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        s_Shooter.setSpeed(finalSpeed, motorLoc);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}