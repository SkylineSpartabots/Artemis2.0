package frc.robot.commands.Pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot.PivotState;;

public class SetPivot extends Command {
    Pivot s_Pivot;
    Indexer s_Indexer;

    PivotState state;
    double desiredCANcoderAngle;

    boolean isShootingIntoAmp;

    // Tune later
    PIDController CANController = new PIDController(38, 10, 0);
    // PIDController CANController = new PIDController(45, 10, 0); shit be too fast bro
    PIDController motorContorller = new PIDController(0, 0, 0);

    public SetPivot(PivotState state) {
        s_Pivot = Pivot.getInstance();
        isShootingIntoAmp = false;
        this.state = state;
        addRequirements(s_Pivot);
    }

    public SetPivot(PivotState state, boolean isShootingIntoAmp) {
        s_Pivot = Pivot.getInstance();
        s_Indexer = Indexer.getInstance();
        this.isShootingIntoAmp = isShootingIntoAmp;
        this.state = state;
        addRequirements(s_Pivot, s_Indexer);
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
            voltage = CANController.calculate(s_Pivot.getCANcoderAbsolutePosition(), s_Pivot.getSetPoint());
            if(isShootingIntoAmp && s_Pivot.getCANcoderAbsolutePosition() > Pivot.pivotDegreeToCANcoder(70)){
                s_Indexer.setSpeed(0.8);
            } else if(isShootingIntoAmp && s_Pivot.getCANcoderAbsolutePosition() < Pivot.pivotDegreeToCANcoder(78)){ //tune how long it is fast for
                voltage = 6; //tune speed
            }
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
		return Math.abs(s_Pivot.getSetPoint() - s_Pivot.getCANcoderAbsolutePosition()) < 0.005 || s_Pivot.getMotorCurrent() > 18;
	}
		
	@Override
	public void end(boolean interrupted) {
        s_Pivot.stopMotor();        
        SmartDashboard.putBoolean("Running", false);
		SmartDashboard.putString("pivot", "pivot end");
	}
}
