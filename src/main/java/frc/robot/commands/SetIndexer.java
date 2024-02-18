
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class SetIndexer extends Command {
    private final Indexer s_Indexer;

    double finalSpeed;

    Boolean motorLoc = true;
    Indexer.IndexerStates state;

    /**
     * @param MotorLocation
     * true = top motor
     * false = bottom motor
     */
    public SetIndexer(Indexer.IndexerStates state, boolean MotorLocation) {
        s_Indexer = Indexer.getInstance();
        finalSpeed = state.getValue();
        motorLoc = MotorLocation;
        addRequirements(s_Indexer);
    }
    /**
     * @param MotorLocation
     * true = top motor
     * false = bottom motor
     */
    public SetIndexer(double difference, boolean MotorLocation) { //increase or decrease speed. Makes sure not to increase above max or decrease below 0
        s_Indexer = Indexer.getInstance();

        double addedSpeed = s_Indexer.getSpeed(MotorLocation) + difference;

        if (addedSpeed <= 1 && addedSpeed >= 0) {
            motorLoc = MotorLocation;
            finalSpeed = addedSpeed;
        }

        addRequirements(s_Indexer);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        s_Indexer.setSpeed(finalSpeed, motorLoc);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}