package frc.robot.commands.Pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Vision;

public class PivotAlign extends Command {
    Pivot s_Pivot;
    Vision s_Vision;

    double desiredCANcoderAngle;

    // Tune later
    PIDController CANController = new PIDController(50, 15, 0);
    // PIDController CANController = new PIDController(45, 10, 0); shit be too fast bro

    public PivotAlign(){
        s_Pivot = Pivot.getInstance();
        s_Vision = Vision.getInstance();
        addRequirements(s_Pivot, s_Vision);
    }

    @Override
    public void initialize() {
        desiredCANcoderAngle = Pivot.pivotDegreeToCANcoder(Constants.getAngleForDistance(s_Vision.getFloorDistance()));
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
		return Math.abs(desiredCANcoderAngle - s_Pivot.getCANcoderAbsolutePosition()) < 0.0005 || s_Pivot.getMotorCurrent() > 18;
	}
		
	@Override
	public void end(boolean interrupted) {
        s_Pivot.stopMotor();        
        SmartDashboard.putBoolean("Running", false);
		SmartDashboard.putString("pivot", "pivot end");
	}
}
