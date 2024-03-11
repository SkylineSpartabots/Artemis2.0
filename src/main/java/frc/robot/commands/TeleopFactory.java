package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.SetIntake;
import frc.robot.commands.Pivot.SetPivot;
import frc.robot.commands.Pivot.ZeroPivot;
import frc.robot.commands.Shooter.SetShooterCommand;
import frc.robot.subsystems.Indexer.IndexerStates;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Pivot.PivotState;

public class TeleopFactory {
    
    public static Command IntelligentIntake(){
        return new ParallelCommandGroup(new SetPivot(PivotState.INTAKE),
        new SequentialCommandGroup(new WaitCommand(0.3), new SetIntake(IntakeStates.ON), new SetIndexer(IndexerStates.ON, true), new SetIntake(IntakeStates.OFF)));
    }

    public static Command Diagnostic() {
        return new SequentialCommandGroup(new ZeroPivot(), new SetPivot(PivotState.SUBWOOFER), new SetPivot(PivotState.GROUND), new SetShooterCommand(35), Commands.waitSeconds(1.0), new SetShooterCommand(0), new SetIntake(IntakeStates.ON, 1), new SetIndexer(IndexerStates.ON, false), Commands.waitSeconds(1.0), new SetIndexer(IndexerStates.OFF, false));
    }

    public static Command SubwooferShootSequence() {
        return new SequentialCommandGroup(new ParallelCommandGroup(new SetPivot(PivotState.SUBWOOFER), new SetShooterCommand(40)), new SetIndexer(IndexerStates.ON, false), offEverything());
    }

    public static Command offEverything(){
        return new SequentialCommandGroup(new ParallelCommandGroup(new SetShooterCommand(-5), new SetIntake(IntakeStates.OFF), Commands.waitSeconds(0.5), new SetIndexer(IndexerStates.OFF, false), new SetPivot(PivotState.GROUND)), new SetShooterCommand(0));
    }

}
