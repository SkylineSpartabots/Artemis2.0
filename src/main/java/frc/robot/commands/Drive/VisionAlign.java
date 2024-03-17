
package frc.robot.commands.Drive;

import java.util.List;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.print.DocFlavor.INPUT_STREAM;

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


    public VisionAlign() { //by branch my rules
        s_Swerve = CommandSwerveDrivetrain.getInstance();
        addRequirements(s_Swerve);
    }


    @Override
    public void initialize() {
            s_Swerve.toggleTractionControl();
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