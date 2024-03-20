
package frc.robot.commands.Drive;

import java.util.List;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.print.DocFlavor.INPUT_STREAM;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Drive extends Command {
    private final CommandSwerveDrivetrain s_Swerve;
    private double maximumStep = 0.1; //needs to be tuned but caps maximum acceleration in a single step to maintain control and stability (theoretically) should be pretty high... ALL EXPERIMENTAL!!! IM BORED!!
    double driverLY;
    double driverLX;
    double driverRX;

    public Drive(double driverLY, double driverLX, double driverRX) { 
        s_Swerve = CommandSwerveDrivetrain.getInstance();
        this.driverLY = driverLY;
        this.driverLX = driverLX;
        this.driverRX = driverRX;
        
        addRequirements(s_Swerve);
    }


    @Override
    public void initialize() {
        driverLX = s_Swerve.scaledDeadBand(driverLX);
        driverLY = s_Swerve.scaledDeadBand(driverLY);
        driverRX = s_Swerve.scaledDeadBand(driverRX);
    }
    
    @Override
    public void execute(){

        if(s_Swerve.getTraction() == true) {

        double slipFactor = 0.995; // 0.5%
        double slipThreshold = 1.15; //a little bit of slip is good but needs to be tuned
        RobotContainer deadband = RobotContainer.getInstance();
        double desiredSpeed =
        Math.sqrt(Math.pow(driverLY * Constants.MaxSpeed, 2) + Math.pow(driverLX * Constants.MaxSpeed, 2)); //m/s

        for(int i = 0; i < 4; i++){

        TalonFX module = Modules[i].getDriveMotor();
        double slipRatio = (Math.abs(module.getRotorVelocity().getValue() * 60) * ((2
        * Math.PI)/60) * (TunerConstants.getWheelRadius() * 0.0254)) / desiredSpeed;

        if(slipRatio > slipThreshold) {
        module.set(module.get() * slipFactor);
        }
        SmartDashboard.putNumber("slip ratio", slipRatio);
        }
        SmartDashboard.putNumber("desired speed", desiredSpeed);

                                            // above and below are different approches to traction control tbh i think above 🦅🦅 
                                            // is better but i made both i guess (perhaps bottom one for water game??) (above uses speed and joysticks 
                                            // below is uses acceleration and gyro and would be more aggresive) (perhaps i could do both???)

        double frictionCoefficant = 0.7; // this is an educated guess of the dynamic coeffiant (need to simulate for this value or something idk)
        double desiredAcceleration = Math.sqrt(
                Math.pow(pigeon.getAccelerationX().getValue(), 2) + Math.pow(pigeon.getAccelerationY().getValue(), 2));
        // could implement gyro values with this for more accurate acceleration using
        // kalman filters but may be too expensive. idk!
        for (int i = 0; i < ModuleCount; i++) {

            TalonFX module = Modules[i].getDriveMotor();
            double WheelAcceleration = (Math.abs(module.getAcceleration().getValue() * 60) * ((2 * Math.PI) / 60)
                    * (TunerConstants.getWheelRadius() * 0.0254)); // not very sure about this math but should be m/s ill check later

        double desiredChange = desiredAcceleration - WheelAcceleration; //neg is wheel is faster and vice versa
        double maxAcceleration = (9.80665 * frictionCoefficant) * 0.025;
        // maximum acceleration we can have is equal to g*CoF, where g is the
        // acceleration due to gravity and CoF is the coefficient of friction between
        // the floor and the wheels (rubber and carpet i assumed), last number is for the max acceleration for traction in THIS time step

            if (-desiredChange > maxAcceleration) {
                desiredAcceleration = desiredChange + maxAcceleration * Math.signum(desiredChange);
            }
                if (module.getAcceleration().getValue() == 1) {
                    module.set(module.get() - 0.05 * desiredAcceleration); 
            }
        }
        // run periodically...
    }


        SwerveRequest request = new SwerveRequest() {
                        .withVelocityX(driverLX * Constants.MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(driverLY * Constants.MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(driverRX * Constants.MaxAngularRate); // Drive counterclockwise with negative X (left) 
        };
                    
        s_Swerve.applyRequest(() -> request);

    }

    @Override
    public void end(boolean interrupted) { 
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}