
package frc.robot.commands.TeleopAutomation;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer.IndexerMotors;
import frc.robot.subsystems.Indexer.IndexerStates;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Pivot.PivotState;

public class IndexForShooting extends Command {
    private final Indexer s_Indexer;
    private final Shooter s_Shooter;
    private final Pivot s_Pivot;
    private final Intake s_Intake;

    private final Timer time;
    Intake.IntakeStates state;
    private double desiredTime;

    public IndexForShooting(double desiredTime) {
        s_Indexer = Indexer.getInstance();
        s_Shooter = Shooter.getInstance();
        s_Pivot = Pivot.getInstance();
        s_Intake = Intake.getInstance();

        this.desiredTime = desiredTime;
        this.time = new Timer();
        addRequirements(this.s_Indexer, s_Indexer, s_Pivot, s_Intake);
    }

    public IndexForShooting(){
        this(2.0);
    }

    @Override
    public void initialize() {
        s_Indexer.setState(IndexerStates.ON);
        s_Intake.setSpeed(IntakeStates.INDEX);
        time.reset(); time.start();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        s_Indexer.setState(IndexerStates.OFF);
        s_Pivot.setState(PivotState.GROUND);
        s_Intake.setSpeed(IntakeStates.OFF);
        s_Shooter.setToIdle();
    }

    @Override
    public boolean isFinished() {//TODO: revise so this is based on distance sensor. Once no note in indexer, turn everything off after a 0.25 s delay.
        return time.get() > desiredTime; //stop after a desired amount of time
    }
}