package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Pivot.SetPivot;
import frc.robot.commands.TeleopAutomation.IndexForShooting;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Pivot.PivotState;

public class ShootIntoAmp extends Command {

    private final Shooter s_Shooter;
    Intake.IntakeStates state;

    public ShootIntoAmp() {
        s_Shooter = Shooter.getInstance();
        addRequirements(s_Shooter);
    }

    @Override
    public void initialize() {
        // Command pivot;
        // pivot = new SetPivot()
        s_Shooter.setBotVelocity(600);
        s_Shooter.setTopVelocity(2000);
        // Command pivotCommand = new SetPivot(PivotState.AMP_BEFORE_SWING);
        // pivotCommand.schedule();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        //new Swing().schedule();
        // Command cmd = new IndexForShooting();
        // cmd.schedule();
    }

    @Override
    public boolean isFinished() {
        return s_Shooter.velocitiesWithinError(75);
    }
}