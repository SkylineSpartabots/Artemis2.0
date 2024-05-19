package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveThread extends CommandBase {

    private final RobotContainer container;


    public DriveThread(RobotContainer container) {
        this.container = container;
    }

    @Override
    public void execute(){
        new Thread(() -> {
            Drive driveCommand = new Drive(container.getDriverLeftY(), container.getDriverLeftX(), container.getDriverRightX());
            driveCommand.schedule(); // why am i scheduling this? - scheduling it should be scheduling the other thread to run the execute right, i think. this is the other thought process of my brain
        }).start();
    }
    @Override
    public boolean isFinished() {
        return false; // Change this as needed
    }
}
