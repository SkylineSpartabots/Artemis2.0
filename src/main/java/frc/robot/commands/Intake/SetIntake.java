
package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeStates;

public class SetIntake extends Command {
    private final Intake s_Intake;
    Intake.IntakeStates state;

    private final Indexer s_Indexer;

    private final int colorSensorProximityThreshold = 110;

    public SetIntake(Intake.IntakeStates state) {
        s_Intake = Intake.getInstance();
        this.state = state;
        addRequirements(s_Intake);

        s_Indexer = Indexer.getInstance();
    }

    @Override
    public void initialize() {
        s_Intake.setSpeed(state);

    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        state = IntakeStates.OFF;
        s_Intake.setSpeed(state);
    }

    @Override
    public boolean isFinished() {
        return s_Indexer.getColorSensorResult() >= colorSensorProximityThreshold;
    }
}