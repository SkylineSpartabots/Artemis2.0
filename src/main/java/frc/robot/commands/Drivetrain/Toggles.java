package frc.robot.commands.Drivetrain;

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
import frc.robot.subsystems.CommandSwerveDrivetrain.Drivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain.DriveControlSystems;
import frc.robot.RobotContainer;


public class Toggles extends Command {

    private DriveControlSystems controlSystem  = new DriveControlSystems();

    public Toggles() {
    }

    @Override
    public void initialize() {
        controlSystem.toggleHeadingControl();
    }
    
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