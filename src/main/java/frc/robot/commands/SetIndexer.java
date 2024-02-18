
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class SetIndexer extends Command {
    private final Indexer s_Indexer;
    double[] addedSpeeds = {0,0}; // top, bottom
    double[] finalSpeeds = {0,0}; // top, bottom
    int motorLoc;

    public SetIndexer(Indexer.IndexerStates state, Indexer.IndexerMotors motor) {
        s_Indexer = Indexer.getInstance();
        motorLoc = motor.getValue();
        finalSpeeds[motorLoc] = state.getValue();
        addRequirements(s_Indexer);
    }

    public SetIndexer(double difference, Indexer.IndexerMotors motor) { //increase or decrease speed. Makes sure not to increase above max or decrease below 0
        s_Indexer = Indexer.getInstance();
        motorLoc = motor.getValue();

        if (motor == Indexer.IndexerMotors.BOTH){
            // cause if they are diff vals then that difference must be preserved by the both increase
            addedSpeeds[0] = s_Indexer.getBottomSpeed() + difference;
            addedSpeeds[1] = s_Indexer.getTopSpeed() + difference;
        } else {
            addedSpeeds [motorLoc] = s_Indexer.getSpeed(motor) + difference;
        }

//        old clunky
//        if (motor == Indexer.IndexerMotors.TOP){
//            addedSpeeds[0] = s_Indexer.getTopSpeed() + difference;
//        } else if (motor == Indexer.IndexerMotors.BOTTOM){
//            addedSpeeds[1] = s_Indexer.getBottomSpeed() + difference;
//        } else if (motor == Indexer.IndexerMotors.BOTH){ /// wait what will this even mean
//            // cause if they are diff vals then that difference must be preserved by the both increase
//            addedSpeeds[0] = s_Indexer.getBottomSpeed() + difference;
//            addedSpeeds[1] = s_Indexer.getBottomSpeed() + difference;
//        }

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

        addRequirements(s_Indexer);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        s_Indexer.setSpeed(finalSpeeds, motorLoc);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}