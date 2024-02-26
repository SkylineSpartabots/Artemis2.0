package frc.robot.commands.Climb;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot.PivotState;

public class SetClimb extends Command{
    Climb s_Climb;
    int state;
    // Tune later
    PIDController controller = new PIDController(0.01, 0, 0);

    public SetClimb(int state) {
        s_Climb = Climb.getInstance();
        this.state = state;
        addRequirements(s_Climb);
    }

    @Override
    public void initialize() {
        s_Climb.setState(state);
        controller.reset();
    }

    @Override
    public void execute() {
        double voltage = controller.calculate(s_Climb.getPosition(), s_Climb.getSetPoint());

        s_Climb.setVoltage(voltage);
    }

    @Override
	public boolean isFinished() {
		return Math.abs(s_Climb.getSetPoint() - s_Climb.getPosition()) < 0.05;
	}
		
	@Override
	public void end(boolean interrupted) {
        s_Climb.setClimbSpeed(0);  

	}
}
