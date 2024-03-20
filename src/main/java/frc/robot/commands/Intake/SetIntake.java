
package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Lights.ledModes;

public class SetIntake extends Command {
    private final Intake s_Intake;
    private final Lights s_Lights;
    Intake.IntakeStates state;

    private final Indexer s_Indexer;

    private Timer timer;
    private double time;
    private boolean intaking;
    private final int colorSensorProximityThreshold = 110;

    public SetIntake(Intake.IntakeStates state, double time){
        s_Intake = Intake.getInstance();
        s_Lights = Lights.getInstance();
        this.time = time;
        timer = new Timer();
        this.state = state;
        if(state == IntakeStates.ON){
            intaking = true;
        }
        addRequirements(s_Intake, s_Lights);

        s_Indexer = Indexer.getInstance();
    }


    public SetIntake(Intake.IntakeStates state) {
        this(state, 2);
    }


    @Override
    public void initialize() {
        s_Intake.setSpeed(state);
        timer.reset();
        timer.start();
        s_Lights.setLEDs(ledModes.Intaking);

    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        if(intaking){
            s_Intake.setSpeed(IntakeStates.OFF);
        }
        if (!interrupted){
            s_Lights.setLEDs(ledModes.IntakeSuccess);
        }
    }

    @Override
    public boolean isFinished() {
        return intaking ?  (s_Indexer.getColorSensorResult() >= colorSensorProximityThreshold || timer.hasElapsed(time)) : true;
    }
}