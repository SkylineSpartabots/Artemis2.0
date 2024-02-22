
package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TrajectoryAlign extends Command {
    private final CommandSwerveDrivetrain s_Swerve;
    private final Vision s_Vision;

    private Pose3d currentPose3d;
    private Pose3d desiredPose3d;

    public TrajectoryAlign() {
        s_Swerve = CommandSwerveDrivetrain.getInstance();
        s_Vision = Vision.getInstance();

        try {
            currentPose3d = s_Vision.calculatePoseFromVision();
        } catch (Exception e) {
            //IF WE DONT SEE NUFFIN WE DONT DO NUFFIN!!!!
        }
        

        addRequirements(s_Vision);
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}