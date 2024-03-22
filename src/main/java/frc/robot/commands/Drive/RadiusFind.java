package frc.robot.commands.Drive;

import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.RobotContainer;

public class RadiusFind extends Command {
    private final CommandSwerveDrivetrain s_Swerve;
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
    private Timer timer;
    private double averageRotations = 0;
    private double sumPigeonYaw = 0;


    public RadiusFind() {
        s_Swerve = CommandSwerveDrivetrain.getInstance();
        addRequirements(s_Swerve);
    }


    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        s_Swerve.applyRequest(() -> drive.withRotationalRate(Constants.MaxAngularRate));
        
    }
    
    @Override
    public void execute(){
        SmartDashboard.putNumber("Swerve Mod 0 Rots", s_Swerve.getModule(0).getDriveMotor().getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Swerve Mod 1 Rots", s_Swerve.getModule(1).getDriveMotor().getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Swerve Mod 2 Rots", s_Swerve.getModule(2).getDriveMotor().getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Swerve Mod 3 Rots", s_Swerve.getModule(3).getDriveMotor().getPosition().getValueAsDouble());

        sumPigeonYaw += s_Swerve.getPigeon2().getYaw().getValueAsDouble();
        s_Swerve.getPigeon2().reset();
    }

    @Override
    public void end(boolean interrupted) { 
        for (int i = 0; i < 4; i++) {
            averageRotations += s_Swerve.getModule(i).getDriveMotor().getPosition().getValueAsDouble();
        }

        averageRotations = averageRotations/4 * 5.25; //multiply by drive gear ratio
        SmartDashboard.putNumber("average rotations", averageRotations);
        SmartDashboard.putNumber("gyro yaw total", sumPigeonYaw);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(3);
    }
}