package frc.robot.commands.Pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot.PivotState;;

public class ZeroPivot extends Command {
    Pivot s_Pivot;

    public ZeroPivot() {
        s_Pivot = Pivot.getInstance();

        addRequirements(s_Pivot);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        s_Pivot.setPercentageOutput(-0.1);
    }

    private double currentTreshold = 10.0;

    @Override
	public boolean isFinished() {
        return s_Pivot.getMotorCurrent() > currentTreshold;
	}
		
	@Override
	public void end(boolean interrupted) {
        s_Pivot.resetMotorEncoders(0.2);
        s_Pivot.resetCANcoder(0.2);
	}
}
