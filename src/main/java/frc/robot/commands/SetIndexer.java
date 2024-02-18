
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class SetIndexer extends Command {
    private final Indexer s_Indexer;
    Indexer.IndexerStates state;

    public SetIndexer(Indexer.IndexerStates state) {
        s_Indexer = Indexer.getInstance();
        this.state = state;
        addRequirements(s_Indexer);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        s_Indexer.setSpeed(state);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}