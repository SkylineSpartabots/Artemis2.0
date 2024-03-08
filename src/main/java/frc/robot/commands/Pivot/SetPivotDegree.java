package frc.robot.commands.Pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot.PivotState;

public class SetPivotDegree extends Command {
    Pivot s_Pivot;
    Indexer s_Indexer;

    double desiredCANcoderAngle;

    // Tune later
    PIDController CANController = new PIDController(45, 12, 0);
    // PIDController CANController = new PIDController(45, 10, 0); shit be too fast bro

    public SetPivotDegree(double desiredAngle){
        s_Pivot = Pivot.getInstance();
        addRequirements(s_Pivot);

        desiredCANcoderAngle = Pivot.pivotDegreeToCANcoder(desiredAngle);
    }

    @Override
    public void initialize() {
        CANController.reset();
    }

    @Override
    public void execute() {
        double voltage;
        if (s_Pivot.CANcoderWorking()) {
            voltage = CANController.calculate(s_Pivot.getCANcoderAbsolutePosition(), desiredCANcoderAngle);
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
		return Math.abs(desiredCANcoderAngle - s_Pivot.getCANcoderAbsolutePosition()) < 0.005 || s_Pivot.getMotorCurrent() > 18;
	}
		
	@Override
	public void end(boolean interrupted) {
        s_Pivot.stopMotor();        
        SmartDashboard.putBoolean("Running", false);
		SmartDashboard.putString("pivot", "pivot end");
	}
}
