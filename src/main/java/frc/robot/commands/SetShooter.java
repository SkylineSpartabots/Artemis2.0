
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SetShooter extends Command {
    private final Shooter s_Shooter;
    double[] addedSpeeds = {0,0}; // top, bottom
    double[] finalSpeeds = {0,0}; // top, bottom
    int motorLoc;

    public SetShooter(Shooter.ShooterStates state, Shooter.ShooterMotors motor) { //change from off or max speed
        s_Shooter = Shooter.getInstance();
        motorLoc = motor.getValue();
        finalSpeeds[motorLoc] = state.getValue();
        addRequirements(s_Shooter);
    }

    public SetShooter(double difference, Shooter.ShooterMotors motor) { //increase or decrease speed. Makes sure not to increase above max or decrease below 0
        s_Shooter = Shooter.getInstance();
        motorLoc = motor.getValue();

        if (motor == Shooter.ShooterMotors.BOTH){
            // cause if they are diff vals then that difference must be preserved by the both increase
            addedSpeeds[0] = s_Shooter.getBottomSpeed() + difference;
            addedSpeeds[1] = s_Shooter.getTopSpeed() + difference;
        } else {
            addedSpeeds [motorLoc] = s_Shooter.getSpeed(motor) + difference;
        }

        // could do a for loop ig - but like okay loop for 2 values, cuts down line count
        if (addedSpeeds[0] <= 1 && addedSpeeds[0] >= 0) {
            finalSpeeds[0] = addedSpeeds[0];
        } else if (addedSpeeds[0] > 1){
            finalSpeeds[0] = 1;
        } else if (addedSpeeds[0] < 0){
            finalSpeeds[0] = 0;
        }
        if (addedSpeeds[1] <= 1 && addedSpeeds[1] >= 0) {
            finalSpeeds[1] = addedSpeeds[1];
        } else if (addedSpeeds[1] > 1){
            finalSpeeds[1] = 1;
        } else if (addedSpeeds[1] < 0){
            finalSpeeds[1] = 0;
        }

        addRequirements(s_Shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        s_Shooter.setSpeed(finalSpeeds, motorLoc);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {return false;}
}