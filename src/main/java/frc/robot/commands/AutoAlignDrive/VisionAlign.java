
package frc.robot.commands.AutoAlignDrive;

import java.util.List;

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
import frc.robot.subsystems.Vision.CameraResult;

public class VisionAlign extends Command {
    private final CommandSwerveDrivetrain s_Swerve;
    private final Vision s_Vision;

    PIDController rotController = new PIDController(0.2, 0, 0);//need to tune
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

    private PhotonTrackedTarget target;
    private boolean hasSpeaker;
    private double lastYaw;

    public VisionAlign() {
        s_Swerve = CommandSwerveDrivetrain.getInstance();
        s_Vision = Vision.getInstance();

        addRequirements(s_Swerve, s_Vision);
    }


    @Override
    public void initialize() {
        lastYaw = 0;
    }
    
    @Override
    public void execute(){
        hasSpeaker = false;
        List<PhotonTrackedTarget> targets = s_Vision.getTargets();
        for(PhotonTrackedTarget a : targets){
            if(a.getFiducialId() == 4 || a.getFiducialId() == 8){
                hasSpeaker = true;
                target = a;
            }
        }
        if(hasSpeaker){
            lastYaw = target.getYaw();
            double rotSpeed = rotController.calculate(lastYaw, 0);
            s_Swerve.setControl(drive.withRotationalRate(rotSpeed));
            // .withVelocityX(-RobotContainer.getInstance().getDriverController().getLeftY() * Constants.MaxSpeed)
            // .withVelocityY(-RobotContainer.getInstance().getDriverController().getLeftX() * Constants.MaxSpeed));
        }
    }

    @Override
    public void end(boolean interrupted) { 
        s_Swerve.setControl(drive.withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return lastYaw < 3; // < arctan(0.5/4) (0.5 is half of width of speaker, 4 is average distance you want to shoot from)
    }
}