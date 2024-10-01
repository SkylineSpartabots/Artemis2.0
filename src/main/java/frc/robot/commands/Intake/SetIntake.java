
package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeStates;

public class SetIntake extends Command {
    private final Intake s_Intake;
    Intake.IntakeStates state;

    private final Indexer s_Indexer;

    private Timer timer;
    private double time; // Time to wait for command to complete
    private final int colorSensorProximityThreshold = 110; // How close does the note need to be to the color sensor to be considered "detected"

    // Overloaded Constructor
    public SetIntake(Intake.IntakeStates state, double time) {
        s_Intake = Intake.getInstance();
        s_Indexer = Indexer.getInstance();

        this.state = state;
        this.time = time;
        timer = new Timer();

        addRequirements(s_Intake);
    }

    // Overloaded constructor
    public SetIntake(Intake.IntakeStates state) {
        this(state, 4);
    }


    @Override
    public void initialize() {
        s_Intake.setSpeed(state);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        // Nothing needs to run repeatedly for this command
    }

    @Override
    public void end(boolean interrupted) {
        s_Intake.setSpeed(IntakeStates.OFF);
    }

    @Override
    public boolean isFinished() { // Return the color sensor result is greater than the threshold or the timer has elapsed, if true end the command
        return  s_Indexer.getColorSensorResult() >= colorSensorProximityThreshold || timer.hasElapsed(time);
    }
}