package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.SetIntake;
import frc.robot.commands.Pivot.SetPivot;
import frc.robot.subsystems.Indexer.IndexerStates;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Pivot.PivotState;

public class TeleopFactory {
    
    public static Command IntelligentIntake(){
        return new ParallelCommandGroup(new SetPivot(PivotState.INTAKE),
        new SequentialCommandGroup(new WaitCommand(0.3), new SetIntake(IntakeStates.ON), new SetIndexer(IndexerStates.ON, true), new SetIntake(IntakeStates.OFF)));
    }

}
