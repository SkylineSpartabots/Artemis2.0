package frc.robot.commands.Pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

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
        SmartDashboard.putBoolean("Running zero", true);
    }

    private double currentThreshold = 10;

    @Override
	public boolean isFinished() {
        return s_Pivot.getMotorCurrent() > currentThreshold;
	}
		
	@Override
	public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Running zero", false);
        s_Pivot.setPercentageOutput(0);
        // s_Pivot.resetMotorEncoders(0.2);
        s_Pivot.resetCANcoder(0.2);
	}
}
