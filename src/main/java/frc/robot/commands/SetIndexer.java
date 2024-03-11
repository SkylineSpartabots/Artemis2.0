
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indexer.IndexerStates;

public class SetIndexer extends Command {
    private final Indexer s_Indexer;
    IndexerStates state;
    private final int colorSensorProximityThreshold = 110; // Test this value later
    private final boolean intaking;
    private double time;
    private Timer timer;

    public SetIndexer(IndexerStates state, boolean intaking, double time) {
        this.intaking = intaking;
        s_Indexer = Indexer.getInstance();
        this.state = state;
        timer = new Timer();
        addRequirements(s_Indexer);
        this.time = time;
    }

    public SetIndexer(IndexerStates state, boolean intaking) {
        this(state, intaking, 2.5);
    }

    @Override
    public void initialize() {
        s_Indexer.setSpeed(state.getValue());
        timer.reset();
        timer.start();
    }

    @Override
    public void end(boolean interrupted) { 
        if(intaking){
            s_Indexer.setSpeed(0);
        }
    }

    @Override
    public boolean isFinished() {
        return intaking ? ((s_Indexer.getColorSensorResult() >= colorSensorProximityThreshold) || timer.hasElapsed(time)): true;
    }
}