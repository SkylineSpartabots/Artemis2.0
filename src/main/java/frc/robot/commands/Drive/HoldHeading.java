package frc.robot.commands.Drive;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.RobotContainer;

public class HoldHeading extends Command {

    public HoldHeading() {
      s_Swerve = CommandSwerveDrivetrain.getInstance();
    }

    CommandSwerveDrivetrain s_Swerve;

    @Override
    public void initialize() {
      s_Swerve.setLastHeading();
      s_Swerve.toggleHeadingControl();
      SmartDashboard.putBoolean("headingON", true);
    }
    
    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted) {
       s_Swerve.toggleHeadingControl(); 
       SmartDashboard.putBoolean("headingON", false);
    }

    @Override
    public boolean isFinished() { //always use in WhileTrue
        return false;
    }
}