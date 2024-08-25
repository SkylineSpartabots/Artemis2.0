// package frc.robot.commands.Drive;

// import java.util.List;

// import org.photonvision.targeting.PhotonTrackedTarget;

// import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Vision;
// import frc.robot.RobotContainer;

// public class PureAlignment extends Command {
//     private final CommandSwerveDrivetrain s_Swerve;
//     private final Vision s_Vision;

//     private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

//     private PhotonTrackedTarget target;
//     private boolean hasSpeaker;
//     private double lastYaw;
//     private Timer time;

//     public PureAlignment() {
//         s_Swerve = CommandSwerveDrivetrain.getInstance();
//         s_Vision = Vision.getInstance();
//         time = new Timer();

//         addRequirements(s_Swerve, s_Vision);
//     }


//     @Override
//     public void initialize() {
//         lastYaw = Integer.MAX_VALUE;
//         time.restart();
//     }
    
//     @Override
//     public void execute(){
//         SmartDashboard.putBoolean("PureAlign", true);
//         hasSpeaker = false;
//         List<PhotonTrackedTarget> targets = s_Vision.getTargetsAsList();
//         for(PhotonTrackedTarget a : targets){
//             if(a.getFiducialId() == 4 || a.getFiducialId() == 8){
//                 hasSpeaker = true;
//                 target = a;
//             }
//         }
//         if(hasSpeaker){
//             lastYaw = target.getYaw();
//             s_Swerve.setControl(drive.withRotationalRate(Math.copySign(1.12, -lastYaw))); //0.2 is a constant, rad/s
//         }
//     }

//     @Override
//     public void end(boolean interrupted) { 
//         SmartDashboard.putBoolean("PureAlign", false);
//         s_Swerve.setControl(drive.withRotationalRate(0));
//     }

//     @Override
//     public boolean isFinished() {
//         return Math.abs(lastYaw) < 3 || time.get() > 1 || RobotContainer.getInstance().driver.getLeftX() > 0.2 || RobotContainer.getInstance().driver.getLeftY() > 0.2 || RobotContainer.getInstance().driver.getRightX() > 0.2; // < arctan(0.5/4) (0.5 is half of width of speaker, 4 is average distance you want to shoot from)
//     }
// }