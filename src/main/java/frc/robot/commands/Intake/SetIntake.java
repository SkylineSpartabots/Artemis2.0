
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
    private double time = 2;
    private boolean intaking;
    private final int colorSensorProximityThreshold = 110;

    public SetIntake(Intake.IntakeStates state) {
        s_Intake = Intake.getInstance();
        this.state = state;
        if(state == IntakeStates.ON){
            intaking = true;
        }
        addRequirements(s_Intake);

        s_Indexer = Indexer.getInstance();

        timer = new Timer();
    }

    public SetIntake(Intake.IntakeStates state, double time){
        s_Intake = Intake.getInstance();
        this.time = time;
        timer = new Timer();
        this.state = state;
        if(state == IntakeStates.ON){
            intaking = true;
        }
        addRequirements(s_Intake);

        s_Indexer = Indexer.getInstance();
    }

    @Override
    public void initialize() {
        s_Intake.setSpeed(state);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        if(intaking){
            s_Intake.setSpeed(IntakeStates.OFF);
        }
    }

    @Override
    public boolean isFinished() {
        return intaking ?  (s_Indexer.getColorSensorResult() >= colorSensorProximityThreshold || timer.hasElapsed(time)) : true;
    }
}