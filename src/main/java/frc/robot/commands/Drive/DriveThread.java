package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class DriveThread extends Command {

    private final RobotContainer container;

    public DriveThread(RobotContainer container) {
        this.container = container;
    }

    @Override
    public void execute() { // Create a new Drive.java command and then schedule it to be run. This should be the only thing running and being scheduled I would think
            Drive driveCommand = new Drive(container.getDriverLeftY(), container.getDriverLeftX(), container.getDriverRightX());
            driveCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return false; // Change this as needed
    }
}
