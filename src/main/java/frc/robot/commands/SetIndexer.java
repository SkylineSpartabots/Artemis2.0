
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class SetIndexer extends Command {
    private final Indexer s_Indexer;
    double[] addedSpeeds = {0, 0}; // top, bottom
    double[] finalSpeeds = {0, 0}; // top, bottom
    Indexer.IndexerMotors motorLoc;

    public SetIndexer(Indexer.IndexerStates state, Indexer.IndexerMotors motor) {
        s_Indexer = Indexer.getInstance();
        if (motor == Indexer.IndexerMotors.BOTH) {
            finalSpeeds[1] = state.getValue();
            finalSpeeds[2] = state.getValue();
        } else {
            finalSpeeds[motor.getValue()] = state.getValue();
        }
        addRequirements(s_Indexer);
    }

    public SetIndexer(double difference, Indexer.IndexerMotors motor) { //increase or decrease speed. Makes sure not to increase above max or decrease below 0
        s_Indexer = Indexer.getInstance();

        if (motor == Indexer.IndexerMotors.BOTH) {
            // cause if they are diff vals then that difference must be preserved by the both increase
            addedSpeeds[1] = s_Indexer.getBottomSpeed() + difference;
            addedSpeeds[2] = s_Indexer.getTopSpeed() + difference;
        } else {
            addedSpeeds[motor.getValue() - 1] = s_Indexer.getBothSpeeds()[motor.getValue() - 1] + difference;
        }

        // bottom
        if (addedSpeeds[1] <= 1 && addedSpeeds[1] >= 0) {
            finalSpeeds[1] = addedSpeeds[1];
        } else if (addedSpeeds[1] > 1) {
            finalSpeeds[1] = 1;
        } else if (addedSpeeds[1] < 0) {
            finalSpeeds[1] = 0;
        }

        // top
        if (addedSpeeds[2] <= 1 && addedSpeeds[2] >= 0) {
            finalSpeeds[2] = addedSpeeds[2];
        } else if (addedSpeeds[2] > 1) {
            finalSpeeds[2] = 1;
        } else if (addedSpeeds[2] < 0) {
            finalSpeeds[2] = 0;
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