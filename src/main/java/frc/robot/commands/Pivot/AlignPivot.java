package frc.robot.commands.Pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Pivot.PivotState;

public class AlignPivot extends Command {
    Pivot s_Pivot;
    Vision s_Vision;

    double desiredCANcoderValue;
    double forcedAngle = -1;
    boolean force;
    // Tune later
    PIDController CANController = new PIDController(50, 15, 0); //TODO: make this into a constant

    public AlignPivot(){
        s_Pivot = Pivot.getInstance();
        s_Vision = Vision.getInstance();
        force = false;
        addRequirements(s_Pivot, s_Vision);
    }

    public AlignPivot(double forcedAngle) {
        s_Pivot = Pivot.getInstance();
        s_Vision = Vision.getInstance();
        force = true;
        this.forcedAngle = forcedAngle;
        addRequirements(s_Pivot, s_Vision);
    }

    public AlignPivot(PivotState state) {
        s_Pivot = Pivot.getInstance();
        s_Vision = Vision.getInstance();
        force = true;
        this.forcedAngle = state.getPos();
        addRequirements(s_Pivot, s_Vision);
    }

    @Override
    public void initialize() {
        if (force) {
            desiredCANcoderValue = Pivot.pivotDegreeToCANcoder(forcedAngle);
        }
        else {
            desiredCANcoderValue = Pivot.pivotDegreeToCANcoder(Constants.getAngleForDistance(s_Vision.getFloorDistance()));
        }
        CANController.reset();
    }

    @Override
    public void execute() {
        double voltage;
        if (s_Pivot.CANcoderWorking()) {
            voltage = CANController.calculate(s_Pivot.getCANcoderAbsolutePosition(), desiredCANcoderValue);
        }
        else {
            voltage = 0;
        }
        s_Pivot.setVoltage(voltage);
        SmartDashboard.putBoolean("Running", true);
    }

    @Override
	public boolean isFinished() {
		return Math.abs(desiredCANcoderValue - s_Pivot.getCANcoderAbsolutePosition()) < 0.001;
	}
		
	@Override
	public void end(boolean interrupted) {
        s_Pivot.stopMotor();        
        SmartDashboard.putBoolean("Running", false);
		SmartDashboard.putString("pivot", "pivot end");
	}
}
