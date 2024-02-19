
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
            finalSpeeds[0] = state.getValue();
            finalSpeeds[1] = state.getValue();
        } else {
            finalSpeeds[motor.getMotor()] = state.getValue();
        }
        addRequirements(s_Indexer);
    }

    public SetIndexer(double difference, Indexer.IndexerMotors motor) { //increase or decrease speed. Makes sure not to increase above max or decrease below 0
        s_Indexer = Indexer.getInstance();

        if (motor == Indexer.IndexerMotors.BOTH) {
            // cause if they are diff vals then that difference must be preserved by the both increase
            addedSpeeds[0] = s_Indexer.getBottomSpeed() + difference;
            addedSpeeds[1] = s_Indexer.getTopSpeed() + difference;
        } else { // -1 is for the indexes cause getMotor returns 1 for bottom and 2 for top motor but indexes...
            addedSpeeds[motor.getMotor() - 1] = s_Indexer.getBothSpeeds()[motor.getMotor() - 1] + difference;
        }

        // way less jank way - called a clamp statement or smth (java21 has it built in but i dunno if we run that?)
        finalSpeeds[0] = Math.max(0.0,Math.min(addedSpeeds[0], 1.0));
        finalSpeeds[1] = Math.max(0.0,Math.min(addedSpeeds[1], 1.0));
        // How it works: min gives which ever is lower - addedspeed or 1. then max gives whichever is bigger 0 or result of min

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