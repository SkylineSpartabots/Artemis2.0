package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class DriveThread extends Command {

    private final RobotContainer container;

    public DriveThread(RobotContainer container) {
        this.container = container;
    }

    // When defaultCommand calls this command a new Thread is created which will make a command of Drive.java and then schedule that command to run in the Thread. Then it starts the thread
    @Override
    public void execute(){
        new Thread(() -> { // Creates a thread which makes driveCommand and then schedules it - should run all the stuff inside Drive.java as a normal command no need for run method i dont think
            Drive driveCommand = new Drive(container.getDriverLeftY(), container.getDriverLeftX(), container.getDriverRightX());
            driveCommand.schedule();
        }).start();
    }
    @Override
    public boolean isFinished() {
        return false; // Change this as needed
    }
}
