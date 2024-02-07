package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot.PivotState;;

public class SetPivot extends Command {
    Pivot s_Pivot;
    PivotState state;
    ProfiledPIDController pivotController = new ProfiledPIDController(0.06, 1e-2, 1e-3, new TrapezoidProfile.Constraints(500000, 3000*1e5));

    public SetPivot(PivotState state) {
        s_Pivot = Pivot.getInstance();
        this.state = state;
        addRequirements(s_Pivot);
    }

    @Override
    public void initialize() {
        s_Pivot.setState(state);
        pivotController.reset(s_Pivot.getCANcoderPosition());
    }

    @Override
    public void execute() {
        double voltage = pivotController.calculate(s_Pivot.getCANcoderPosition(), s_Pivot.getSetPoint());
        if (Math.abs(s_Pivot.getCANcoderPosition() - s_Pivot.getSetPoint()) < 15) {
			voltage = 0.7;
		}
        s_Pivot.setVoltage(voltage);
    }

    @Override
	public boolean isFinished() {
		return Math.abs(s_Pivot.getSetPoint() - s_Pivot.getCANcoderPosition()) < 45;
	}
		
	@Override
	public void end(boolean interrupted) {
		s_Pivot.setVoltage(0);
		SmartDashboard.putString("eleEnd", "elevator end");
	}
}
