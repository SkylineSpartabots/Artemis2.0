package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;

public class TimerCommand extends Command {

    private final Lights s_Lights;
    private Timer timer;
    public TimerCommand(){
        s_Lights = Lights.getInstance();
        addRequirements(s_Lights);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        s_Lights.setLEDs(Lights.ledModes.Intaking);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println(interrupted);
        if (!interrupted){
            s_Lights.setLEDs(Lights.ledModes.IntakeSuccess);
        }
    }

    @Override
    public boolean isFinished() {
        System.out.println(timer.get());
        return timer.hasElapsed(10);
    }
}
