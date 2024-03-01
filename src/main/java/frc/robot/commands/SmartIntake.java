// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Intake.SetIntake;
import frc.robot.commands.Pivot.SetPivot;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Indexer.IndexerStates;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Pivot.PivotState;

public class SmartIntake extends Command {
  private final Intake s_Intake;
  private final Indexer s_Indexer;
  private final Pivot s_Pivot;

  public SmartIntake() {
    s_Intake = Intake.getInstance();
    s_Indexer = Indexer.getInstance();
    s_Pivot = Pivot.getInstance();
    addRequirements(s_Intake, s_Indexer, s_Pivot);
  }

  @Override
  public void initialize() {
    new ParallelCommandGroup(
      new SetIntake(IntakeStates.ON),
      new SetIndexer(IndexerStates.ON, true),
      new SetPivot(PivotState.SUBWOOFER)
    ).schedule();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    new ParallelCommandGroup(
      new SetIntake(IntakeStates.OFF),
      new SetIndexer(IndexerStates.OFF, false)
    ).schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_Indexer.getColorSensorResult() < 5; //TODO: need to tune so note doesn't touch shooter when command ends
  }
}
