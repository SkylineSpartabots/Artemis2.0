
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indexer.IndexerStates;

public class SetIndexer extends Command {
    private final Indexer s_Indexer;
    IndexerStates state;
    private final int colorSensorProximityThreshold = 110; // Test this value later
    private final boolean intaking;

    public SetIndexer(IndexerStates state, boolean intaking) {
        this.intaking = intaking;
        s_Indexer = Indexer.getInstance();
        this.state = state;

        addRequirements(s_Indexer);
    }


    @Override
    public void initialize() {
        s_Indexer.setSpeed(state.getValue());
    }

    @Override
    public void end(boolean interrupted) { 
        if(intaking){
            s_Indexer.setSpeed(0);
        }
    }

    @Override
    public boolean isFinished() {
        return intaking ? s_Indexer.getColorSensorResult() >= colorSensorProximityThreshold : true;
        // return intaking ? s_Indexer.getLimitSwitchResult() : true;
    }
}