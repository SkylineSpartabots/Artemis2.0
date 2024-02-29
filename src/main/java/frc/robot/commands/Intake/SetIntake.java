
package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class SetIntake extends Command {
    private final Intake s_Intake;
    Intake.IntakeStates state;

    public SetIntake(Intake.IntakeStates state) {
        s_Intake = Intake.getInstance();
        this.state = state;
        addRequirements(s_Intake);
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
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}