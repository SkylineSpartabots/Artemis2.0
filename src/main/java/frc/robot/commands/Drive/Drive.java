
package frc.robot.commands.Drive;

import java.util.List;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.print.DocFlavor.INPUT_STREAM;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
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
    private double driverLY;
    private double driverLX;
    private double driverRX;
    private Double[] adjustedInputs;


    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.MaxSpeed * 0.1).withRotationalDeadband(Constants.MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

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

        if(s_Swerve.getTraction() == true) {
             adjustedInputs = s_Swerve.tractionControl(driverLX , driverLY);
             driverLX = adjustedInputs[4];
             driverLY = adjustedInputs[5];
        }
        
        s_Swerve.applyRequest(() ->
                         drive.withVelocityX(driverLX)
                        .withVelocityY(driverLY)
                        .withRotationalRate(driverRX));
        s_Swerve.slipCorrection(adjustedInputs);
        s_Swerve.resetTime();
    }
    
    @Override
    public void execute(){
    }


    @Override
    public void end(boolean interrupted) { 
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}