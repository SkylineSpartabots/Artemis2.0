package frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain.Drivetrain;

public class ControlSystems {

    
    private double deadbandFactor = 0.8;

    private boolean slipControlOn = false;
    private boolean headingControl = false;
    private boolean shooterMode = false;
    private boolean aligning = false;
    private double lastHeading = 0;

    PIDController pidHeading = new PIDController(0, 0, 0);
    private final Drivetrain drivetrain = Drivetrain.getInstance(); // Drivetrain

    //interface with modules
    public SwerveModule getModule(int index) {
      return drivetrain.getModule(index);
    }   

     // =======---===[ ⚙ Joystick processing ]===---========
    public SwerveRequest drive(double driverLY, double driverLX, double driverRX){
        driverLX = scaledDeadBand(driverLX) * Constants.MaxSpeed;
        driverLY = scaledDeadBand(driverLY) * Constants.MaxSpeed;
        driverRX = scaledDeadBand(driverRX) * Constants.MaxSpeed;

        if (shooterMode) {
            driverRX = shooterMode();
        } else if (headingControl && driverRX < 0.1) {
            driverRX = headingControl(driverRX);
        } 

        //TODO consistent pivot and drivetrain alignment to target
        if (slipControlOn) {
        slipCorrection(slipControl(drivetrain.robotAbsoluteVelocity())); 
        }

        return new SwerveRequest.FieldCentric()
        .withVelocityX(driverLY)
        .withVelocityY(driverLX)
        .withRotationalRate(driverRX * Constants.MaxAngularRate)
        .withDeadband(Constants.MaxSpeed * RobotContainer.translationDeadband)
        .withRotationalDeadband(Constants.MaxAngularRate * RobotContainer.rotDeadband)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    }

    public double scaledDeadBand(double input) {
        return (deadbandFactor * Math.pow(input, 3)) + (1 - deadbandFactor) * input;
    }

    // =======---===[ ⚙ Heading control ]===---========
    public double headingControl(double driverRX){ //TODO tune high and low PID values
        if (!pidHeading.atSetpoint()) {
            double velocity = drivetrain.robotAbsoluteVelocity();
            updateGains(velocity);
            
            driverRX = pidHeading.calculate(drivetrain.getPose().getRotation().getRadians(), lastHeading);
            SmartDashboard.putBoolean("headingON", true);

        } else {
            SmartDashboard.putBoolean("headingON", false);
            SmartDashboard.putNumber("lastHeading", lastHeading);
        }

        return driverRX;
    }

    public void updateGains(double velocity) {
        double speedRatio = Math.abs(Constants.MaxSpeed/velocity); //velocity is from wheels so could be off
        speedRatio = Math.max(0, Math.min(1, speedRatio));
        //clamp between 0 and 1

        pidHeading.setPID(
            interpolate(Constants.robotPIDs.HeadingControlPID.lowP, Constants.robotPIDs.HeadingControlPID.highP, speedRatio), // P
            0, // I (we do not need I)
            interpolate(Constants.robotPIDs.HeadingControlPID.lowD, Constants.robotPIDs.HeadingControlPID.highD, speedRatio) // D
            ); 
    }

    public double interpolate(double lower, double upper, double scale) {
        return Interpolator.forDouble().interpolate(lower, upper, scale);
    }

    // =======---===[ ⚙ Slip Control ]===---========
    public Double[] slipControl(double currentVelocity) {

    Double[] outputs = new Double[4]; // reset to null every call
    SmartDashboard.putNumber("currentVelocity", currentVelocity);

        for (int i = 0; i < 4; i++) {  //4 is module count but i dont want to make a getter
        
        //gets the ratio between what the encoders think our velocity is and the real velocity
        double slipRatio;
        if(currentVelocity == 0) { slipRatio = 1; } else {
            slipRatio = ((getModule(i).getCurrentState().speedMetersPerSecond) / currentVelocity); 
        }
        SmartDashboard.putNumber("Module " + i + " slipratio", slipRatio);
        
        //if over the upper or lower threshold save the value
        if (slipRatio > (Constants.slipThreshold + 1) || slipRatio < (1 - Constants.slipThreshold)) {
            outputs[i] = slipRatio;
        }
    }

    return outputs;
    } // runs periodically as a default command

    public void slipCorrection(Double[] inputs) {
        // divides by slip factor, more aggressive if far above slip threshold 
        for (int i = 0; i < 4; i++) { //4 is module count but i dont want to make a getter

            if (inputs[i] != null) {
                TalonFX module = getModule(i).getDriveMotor();
                
                module.set(module.get() *
                 (1 + (Math.signum(inputs[i] - 1)) * (inputs[i] - Constants.slipThreshold)) / Constants.slipFactor);
                //https://www.desmos.com/calculator/afe5omf92p how slipfactor changes slip aggression

                SmartDashboard.putBoolean("slipON", true);
            }  else {
                SmartDashboard.putBoolean("slipON", false);
            } 
        }
    }


    //shooter mode??
    public double shooterMode() { // I WANT MODES FOR THE NEXT ROBOT!!!!
        // continuously sets pivot to based on lookup table and aligns drivetrain
        // only shoots when both systems return true
        return -1;
    }


    //toggling ----------------------------------------------------------------
    public void setLastHeading() {
        lastHeading = drivetrain.getPose().getRotation().getRadians(); 
    }

    public void toggleHeadingControl() {
        headingControl = !headingControl;
    }

    public void toggleAlignment() {
        aligning = !aligning;
    }

    public void toggleSlipControl() {
        slipControlOn = !slipControlOn;
    }

    public void setHeadingTolerance() {
        pidHeading.setTolerance(0.1745); // 10 degrees in radians
    }


}
