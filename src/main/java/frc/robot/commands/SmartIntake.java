// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Pivot.AlignPivot;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Indexer.IndexerStates;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Lights.ledModes;
import frc.robot.subsystems.Pivot.PivotState;

public class SmartIntake extends Command {
  private final Intake s_Intake;
  private final Indexer s_Indexer;
  private final Lights s_Lights;

  public SmartIntake() {
    s_Intake = Intake.getInstance();
    s_Indexer = Indexer.getInstance();
    s_Lights = Lights.getInstance();
    addRequirements(s_Intake, s_Indexer, s_Lights);
  }

  @Override
  public void initialize() {
    s_Indexer.setState(IndexerStates.ON);
    s_Intake.setSpeed(IntakeStates.ON);
    s_Lights.setLEDs(ledModes.Intaking);
    new AlignPivot(PivotState.INTAKE).schedule();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    s_Intake.setSpeed(IntakeStates.OFF);
    s_Indexer.setState(IndexerStates.OFF);
    if (!interrupted){
      s_Lights.setLEDs(ledModes.IntakeSuccess);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_Indexer.getColorSensorResult() > 145; //TODO: need to tune so note doesn't touch shooter when command ends
    // return s_Indexer.getLimitSwitchResult();
  }
}
