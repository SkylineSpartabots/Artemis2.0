
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class SetShooter extends Command {
    private final Shooter s_Shooter;
    double[] addedSpeeds = {0,0}; // top, bottom
    double[] finalSpeeds = {0,0}; // top, bottom
    Shooter.ShooterMotors motorLoc;

    public SetShooter(Shooter.ShooterStates state, Shooter.ShooterMotors motor) { //change from off or max speed
        s_Shooter = Shooter.getInstance();
        if (motor == Shooter.ShooterMotors.BOTH){
            finalSpeeds[1] = state.getValue();
            finalSpeeds[2] = state.getValue();
        } else {
            finalSpeeds[motor.getValue()] = state.getValue();
        }
        addRequirements(s_Shooter);
    }

    public SetShooter(double difference, Shooter.ShooterMotors motor) { //increase or decrease speed. Makes sure not to increase above max or decrease below 0
        s_Shooter = Shooter.getInstance();

        if (motor == Shooter.ShooterMotors.BOTH){
            // cause if they are diff vals then that difference must be preserved by the both increase
            addedSpeeds[1] = s_Shooter.getBottomSpeed() + difference;
            addedSpeeds[2] = s_Shooter.getTopSpeed() + difference;
        } else {
            addedSpeeds[motor.getValue()] = s_Shooter.getBothSpeeds()[motor.getValue() - 1] + difference;
        }

        // bottom
        if (addedSpeeds[1] <= 1 && addedSpeeds[1] >= 0) {
            finalSpeeds[1] = addedSpeeds[1];
        } else if (addedSpeeds[1] > 1){
            finalSpeeds[1] = 1;
        } else if (addedSpeeds[1] < 0){
            finalSpeeds[1] = 0;
        }

        // top
        if (addedSpeeds[2] <= 1 && addedSpeeds[2] >= 0) {
            finalSpeeds[2] = addedSpeeds[2];
        } else if (addedSpeeds[2] > 1){
            finalSpeeds[2] = 1;
        } else if (addedSpeeds[2] < 0){
            finalSpeeds[2] = 0;
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