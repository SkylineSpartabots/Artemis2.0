package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Amp.SetAmp;
import frc.robot.commands.Amp.ZeroAmp;
import frc.robot.commands.Drive.SpeakerAlign;
//import frc.robot.commands.Drive.PureAlignment;
import frc.robot.commands.Intake.SetIntake;
import frc.robot.commands.Pivot.AlignPivot;
import frc.robot.commands.Pivot.ZeroPivot;
import frc.robot.commands.Shooter.SetShooterCommand;
import frc.robot.commands.Shooter.ZeroShooter;
import frc.robot.subsystems.Amp.AmpState;
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
                new SetShooterCommand(45)
            ),
            Commands.waitSeconds(0.2), 
            new SetIndexer(IndexerStates.ON), 
            Commands.waitSeconds(0.5),
            offEverything()
        );
    }

    public static Command defensiveStance() {
        return new ParallelCommandGroup(
            new AlignPivot(67.4),
            new SetAmp(AmpState.DEFENSE) 
        );
    }

    public static Command offEverything(){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new ZeroShooter(), 
                new SetIntake(IntakeStates.OFF), 
                Commands.waitSeconds(0.5), 
                new SetIndexer(IndexerStates.OFF), 
                new AlignPivot(PivotState.GROUND)
                //new SetAmp(AmpState.ZERO)
            )
        );
    }

    public static Command autoShootSequence(){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SetShooterCommand(60)
                //new PureAlignment()
            ), 
            new AlignPivot(), 
            new SetIndexer(IndexerStates.ON),
            Commands.waitSeconds(0.5),
            new SetIndexer(IndexerStates.OFF)
        );
    }

    public static Command eject(){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SetIndexer(IndexerStates.REV), 
                new SetIntake(IntakeStates.REV)
            ), 
            new WaitCommand(0.065),
            new ParallelCommandGroup(
                new SetIndexer(IndexerStates.OFF), 
                new SetIntake(IntakeStates.OFF)
            )
        );
    }

    public static Command shootSubwooferPrep(){
        return new ParallelCommandGroup(
            new AlignPivot(PivotState.SUBWOOFER), 
            new SetShooterCommand(45)
        );
    }

    public static Command ampPrep() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SetShooterCommand(22, 7),
                new AlignPivot(PivotState.AMP),
                new SetAmp(AmpState.DEPLOYED)
            )
           
        );
    }

    public static Command zeroAmpPivot() {
        return new ParallelCommandGroup(
            new ZeroPivot(),
            new ZeroAmp()
        );
    }

    public static Command stageShootPrep() {
        return new ParallelCommandGroup(
            new AlignPivot(PivotState.STAGE), //need to set pivot angle for stage
            new SetShooterCommand(45),
            new SpeakerAlign()
        );
    }

    public static Command lobNote() {
        return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new AlignPivot(PivotState.FARWING),
            new SetShooterCommand(45)  
          ),
        new SetIndexer(IndexerStates.SHOOTING)
        );
    }
}
