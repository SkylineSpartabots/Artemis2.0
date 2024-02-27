package frc.robot.commands.Pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot.PivotState;;

public class SetPivot extends Command {
    Pivot s_Pivot;
    PivotState state;
    double desiredCANcoderAngle;

    // Tune later
    PIDController CANController = new PIDController(50
    , 12, 0);
    PIDController motorContorller = new PIDController(0, 0, 0);

    public SetPivot(PivotState state) {
        s_Pivot = Pivot.getInstance();
        this.state = state;
        addRequirements(s_Pivot);
    }

    public SetPivot(double desiredAngle){
        s_Pivot = Pivot.getInstance();
        this.state = PivotState.NULL;
        addRequirements(s_Pivot);

        desiredCANcoderAngle = Pivot.pivotDegreeToCANcoder(desiredAngle);
    }

    @Override
    public void initialize() {
        s_Pivot.setState(state);
        CANController.reset();
    }

    @Override
    public void execute() {
        double voltage;
        if (s_Pivot.CANcoderWorking()) {
            voltage = CANController.calculate(s_Pivot.getCANcoderAbsolutePosition(),  state == PivotState.NULL ? desiredCANcoderAngle : s_Pivot.getSetPoint());
        }
        else {
            voltage = 0;
        }
        // if (Math.abs(s_Pivot.getCANcoderPosition() - s_Pivot.getSetPoint()) < 15) {
		// 	voltage = 0.7;
		// }
        s_Pivot.setVoltage(voltage);
        SmartDashboard.putBoolean("Running", true);
    }

    @Override
	public boolean isFinished() {
		return Math.abs(s_Pivot.getSetPoint() - s_Pivot.getCANcoderAbsolutePosition()) < 0.05;
	}
		
	@Override
	public void end(boolean interrupted) {
        s_Pivot.stopMotor();        
        SmartDashboard.putBoolean("Running", false);
		SmartDashboard.putString("pivot", "pivot end");
	}
}
