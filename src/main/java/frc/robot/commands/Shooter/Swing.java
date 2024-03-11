package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Pivot.AlignPivot;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indexer.IndexerStates;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Pivot.PivotState;

public class Swing extends Command {
    Pivot s_Pivot;
    Shooter s_Shooter;
    Indexer s_Indexer;

    double indexerTimeDelay = 0.05;
    private Timer time = new Timer();

    public Swing() {
        s_Pivot = Pivot.getInstance();
        s_Shooter = Shooter.getInstance();
        s_Indexer = Indexer.getInstance();
        
        time = new Timer();
        addRequirements(s_Pivot, s_Shooter, s_Indexer);
    }

    @Override
    public void initialize() {
        new AlignPivot(PivotState.AMP).schedule();
        time.reset(); time.start();

    }

    @Override
    public void execute() {
        if (time.get() > indexerTimeDelay) {
            s_Indexer.setState(IndexerStates.ON);
        }
    }

    @Override
    public void end(boolean interrupted) {
        s_Indexer.setState(IndexerStates.OFF);
        s_Shooter.setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return time.get() > 1.5;
    }
}
