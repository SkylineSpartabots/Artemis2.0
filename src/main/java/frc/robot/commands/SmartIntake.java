// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Pivot.AlignPivot;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer.IndexerStates;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Pivot.PivotState;

public class SmartIntake extends Command {
  private final Intake s_Intake;
  private final Indexer s_Indexer;

  private final int motorCurrentThreshold = 150;

  public SmartIntake() {
    s_Intake = Intake.getInstance();
    s_Indexer = Indexer.getInstance();
    addRequirements(s_Intake, s_Indexer);
  }

  @Override
  public void initialize() {
    new SetIndexer(IndexerStates.ON).schedule();
    s_Intake.setSpeed(IntakeStates.ON);
    new AlignPivot(PivotState.INTAKE).schedule();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    s_Intake.setSpeed(IntakeStates.OFF);
    s_Indexer.setState(IndexerStates.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
    // return s_Indexer.getMotorCurrent() > motorCurrentThreshold;
    // return s_Indexer.getLimitSwitchResult();
  }
}
