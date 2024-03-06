
package frc.robot.commands.AutoAlignDrive;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indexer.IndexerStates;

public class VisionAlign extends Command {
    private final CommandSwerveDrivetrain s_Swerve;
    private final Vision s_Vision;

    PIDController rotController = new PIDController(1.7, 0.95, 0.05);//need to tune
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

    private PhotonTrackedTarget target;

    public VisionAlign() {
        s_Swerve = CommandSwerveDrivetrain.getInstance();
        s_Vision = Vision.getInstance();

        addRequirements(s_Swerve, s_Vision);
    }


    @Override
    public void initialize() {
    }
    
    @Override
    public void execute(){
        if(s_Vision.hasValidTarget()!=null){
            target = s_Vision.getBestTarget();
        }
        double rotSpeed = rotController.calculate(target.getYaw(), 0);
        s_Swerve.setControl(drive.withRotationalRate(rotSpeed)
        .withVelocityX(-RobotContainer.getInstance().getDriverController().getLeftY() * Constants.MaxSpeed)
        .withVelocityY(-RobotContainer.getInstance().getDriverController().getLeftX() * Constants.MaxSpeed));
    }

    @Override
    public void end(boolean interrupted) { 
        s_Swerve.setControl(drive.withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return s_Vision.hasValidTarget() == null || !(s_Vision.getBestTarget().getFiducialId() == 4 || s_Vision.getBestTarget().getFiducialId() == 8);
    }
}