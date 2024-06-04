package frc.robot.commands.Drive;

import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.System_StateValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.RobotContainer;


public class Toggles extends Command {

    CommandSwerveDrivetrain s_Swerve;

    public Toggles() {
        s_Swerve = CommandSwerveDrivetrain.getInstance();
    }


    @Override
    public void initialize() {
        if (s_Swerve.getTractionBool() == false && s_Swerve.getHeadingControlBool() == false) { s_Swerve.toggleTractionControl();} 
        else if (s_Swerve.getTractionBool() == true && s_Swerve.getHeadingControlBool() == false) { s_Swerve.toggleHeadingControl();  s_Swerve.toggleTractionControl();}
     else if (s_Swerve.getTractionBool() == false && s_Swerve.getHeadingControlBool() == true){
            s_Swerve.toggleTractionControl();
        } else if (s_Swerve.getTractionBool() == true && s_Swerve.getHeadingControlBool() == true) { s_Swerve.toggleHeadingControl();  s_Swerve.toggleTractionControl();}
    } //first run toggles only traction, then traction and heading, then both turn off
    
    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() { //always use in WhileTrue
        return true;
    }
}