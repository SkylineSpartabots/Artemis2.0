package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoAlignDrive.PureAlignment;
import frc.robot.commands.Intake.SetIntake;
import frc.robot.commands.Pivot.AlignPivot;
import frc.robot.commands.Pivot.ZeroPivot;
import frc.robot.commands.Shooter.SetShooterCommand;
import frc.robot.subsystems.Indexer.IndexerStates;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Pivot.PivotState;

public class CommandFactory {

    public static Command Diagnostic() {
        return new SequentialCommandGroup(
            new ZeroPivot(), 
            new AlignPivot(PivotState.SUBWOOFER),  
            new AlignPivot(PivotState.GROUND), 
            new SetShooterCommand(35),  
            Commands.waitSeconds(1.0), 
            new SetShooterCommand(0), 
            new SetIntake(IntakeStates.ON, 1), 
            new SetIndexer(IndexerStates.ON, false), 
            Commands.waitSeconds(1.0), 
            new SetIndexer(IndexerStates.OFF, false)
        );
    }

    public static Command SubwooferShootSequence() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new AlignPivot(PivotState.SUBWOOFER), 
                new SetShooterCommand(35)
            ),
            Commands.waitSeconds(0.7), 
            new SetIndexer(IndexerStates.ON, false), 
            offEverything()
        );
    }

    public static Command offEverything(){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SetShooterCommand(-5), 
                new SetIntake(IntakeStates.OFF), 
                Commands.waitSeconds(0.5), 
                new SetIndexer(IndexerStates.OFF, false), 
                new AlignPivot(PivotState.GROUND)
            ), 
            new SetShooterCommand(0)
        );
    }

    public static Command autoShootSequence(){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SetShooterCommand(60), 
                new PureAlignment()
            ), 
            new AlignPivot(), 
            new SetIndexer(IndexerStates.ON, false)
        );
    }

    public static Command eject(){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SetIndexer(IndexerStates.REV, false), 
                new SetIntake(IntakeStates.REV)
            ), 
            new WaitCommand(0.065),
            new ParallelCommandGroup(
                new SetIndexer(IndexerStates.OFF, false), 
                new SetIntake(IntakeStates.OFF)
            )
        );
    }

    public static Command shootSubwooferPrep(){
        return new ParallelCommandGroup(
            new AlignPivot(PivotState.SUBWOOFER), 
            new SetShooterCommand(35)
        );
    }

    public static Command ampShootSequence() {
        return new SequentialCommandGroup(
            new SetShooterCommand(24, 12), 
            new ParallelCommandGroup(
                new AlignPivot(PivotState.AMP), 
                new SetIndexer(IndexerStates.AMP, false)
            )
        );
    }

}