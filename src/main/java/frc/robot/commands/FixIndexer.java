
package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indexer.IndexerStates;

public class FixIndexer extends Command {
    private final Indexer s_Indexer;
    private int colorSensorProximityThreshold; // Test this value later
    private boolean correctPosition;

    public FixIndexer() {
        s_Indexer = Indexer.getInstance();

        addRequirements(s_Indexer);
    }


    @Override
    public void initialize() {
        colorSensorProximityThreshold = 300;
        correctPosition = false;
    }

    @Override
    public void execute(){
        s_Indexer.setState(IndexerStates.REV);
        SmartDashboard.putBoolean("IsRev", s_Indexer.getColorSensorResult() < colorSensorProximityThreshold);
    }

    @Override
    public void end(boolean interrupted) {
        s_Indexer.setState(IndexerStates.OFF);
    }

    @Override
    public boolean isFinished() {
        correctPosition = s_Indexer.getColorSensorResult() < colorSensorProximityThreshold;
        // correctPosition = !(s_Indexer.getLimitSwitchResult());
        return correctPosition;
    }
}