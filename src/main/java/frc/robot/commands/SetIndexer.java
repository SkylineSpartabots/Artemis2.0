
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indexer.IndexerStates;
import frc.robot.subsystems.Lightz;

public class SetIndexer extends Command {
    private final Indexer s_Indexer;
    private final Lightz s_Lightz;
    IndexerStates state;
    private final int colorSensorProximityThreshold = 110; // Test this value later
    private final boolean intaking;
    private double time = 2.5;
    private Timer timer;

    public SetIndexer(IndexerStates state, boolean intaking) {
        this.intaking = intaking;
        s_Indexer = Indexer.getInstance();
        s_Lightz = Lightz.getInstance();
        this.state = state;
        timer = new Timer();
        addRequirements(s_Indexer);
    }

    public SetIndexer(IndexerStates state, boolean intaking, double time) {
        this.intaking = intaking;
        s_Indexer = Indexer.getInstance();
        this.state = state;
        timer = new Timer();
        addRequirements(s_Indexer);
        this.time = time;
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
        // return intaking ? s_Indexer.getLimitSwitchResult() : true;
        s_Lightz.setLEDs(Lightz.ledModes.RED); // should this be right here?
    }
}