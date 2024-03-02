
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indexer.IndexerStates;
import frc.robot.subsystems.Lightz;

public class SetIndexer extends Command {
    private final Indexer s_Indexer;
    private final Lightz s_Lightz;
    IndexerStates state;
    private final int colorSensorProximityThreshold = 700; // Test this value later
    private final boolean intaking;

    public SetIndexer(IndexerStates state, boolean intaking) {
        this.intaking = intaking;
        s_Indexer = Indexer.getInstance();
        s_Lightz = Lightz.getInstance();
        this.state = state;

        addRequirements(s_Indexer);
    }


    @Override
    public void initialize() {
        s_Indexer.setSpeed(state.getValue());
    }

    @Override
    public void end(boolean interrupted) {
        if (intaking) {
            s_Indexer.setSpeed(0);

        }
    }

    @Override
    public boolean isFinished() {
        s_Lightz.setLEDs(Lightz.ledModes.RED); // should this be right here?
        return intaking ? s_Indexer.getColorSensorResult() >= colorSensorProximityThreshold : true;
    }
}