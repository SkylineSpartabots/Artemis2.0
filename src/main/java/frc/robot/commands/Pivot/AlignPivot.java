package frc.robot.commands.Pivot;

import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Pivot;
//import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Pivot.PivotState;

public class AlignPivot extends Command {
    Pivot s_Pivot;
    //Vision s_Vision;

    double desiredCANcoderValue;
    double desiredAngle;

    boolean shootingFromDistance;
    double distance;

    // Tune later
    PIDController CANController = new PIDController(30, 20, 0);

    public AlignPivot(double desiredAngle) {
        s_Pivot = Pivot.getInstance();
        //s_Vision = Vision.getInstance();
        shootingFromDistance = false;
        this.desiredAngle = desiredAngle;
        addRequirements(s_Pivot);
    }

    public AlignPivot() {
        s_Pivot = Pivot.getInstance();
        //s_Vision = Vision.getInstance();
        //distance = s_Vision.getFloorDistance();
        desiredAngle = Constants.getAngleForDistance(distance);
        shootingFromDistance = true;
        addRequirements(s_Pivot);
    }

    public AlignPivot(PivotState state) {
        this(state.getPos());
    }

    @Override
    public void initialize() {
        desiredCANcoderValue = Pivot.pivotDegreeToCANcoder(desiredAngle);
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
		return shootingFromDistance ? Math.abs(desiredCANcoderValue - s_Pivot.getCANcoderAbsolutePosition()) < 1/(125*distance) : Math.abs(desiredCANcoderValue - s_Pivot.getCANcoderAbsolutePosition()) < 0.001;
	}
		
	@Override
	public void end(boolean interrupted) {
        s_Pivot.stopMotor();        
        SmartDashboard.putBoolean("Running", false);
		SmartDashboard.putString("pivot", "pivot end");
	}
}
