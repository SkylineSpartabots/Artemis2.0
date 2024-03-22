
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
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Drive extends Command {
    private final CommandSwerveDrivetrain s_Swerve;
    private double maximumStep = 0.1; //needs to be tuned but caps maximum acceleration in a single step to maintain control and stability (theoretically) should be pretty high... ALL EXPERIMENTAL!!! IM BORED!!
    private double driverLY;
    private double driverLX;
    private double driverRX;
    private double[] adjustedInputs;
    
    

    public Drive(double driverLY, double driverLX, double driverRX) { 
        s_Swerve = CommandSwerveDrivetrain.getInstance();
        this.driverLY = driverLY;
        this.driverLX = driverLX;
        this.driverRX = driverRX;
        
        addRequirements(s_Swerve);
    }


    @Override
    public void initialize() {
        driverLX = s_Swerve.scaledDeadBand(driverLX) * Constants.MaxSpeed;
        driverLY = s_Swerve.scaledDeadBand(driverLY) * Constants.MaxSpeed;
        driverRX = s_Swerve.scaledDeadBand(driverRX) * Constants.MaxSpeed;
    }
    
    @Override
    public void execute(){

        if(s_Swerve.getTraction() == true) {
             adjustedInputs = s_Swerve.tractionControl(driverLX , driverLY);
        }

        
            SwerveRequest request = new SwerveRequest() {
                        .withVelocityX(driverLX) // Drive forward with negative Y (forward)
                        .withVelocityY(driverLY) // Drive left with negative X (left)
                        .withRotationalRate(driverRX) // Drive counterclockwise with negative X (left) 
            }; //TODO fix

        s_Swerve.applyRequest(() -> request);
        //TODO factor in slip
        s_Swerve.lastTimeReset = System.currentTimeMillis();

        }
    }

    @Override
    public void end(boolean interrupted) { 
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}