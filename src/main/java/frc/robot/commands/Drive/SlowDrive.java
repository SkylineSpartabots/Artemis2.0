package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class SlowDrive extends Command {

    public SlowDrive() {
    }


    @Override
    public void initialize() {
        Constants.MaxAngularRate = 1*Math.PI;
        Constants.MaxSpeed = 2;
    }
    
    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted) {
        Constants.MaxAngularRate = 3*Math.PI;
        Constants.MaxSpeed = 6;
    }

    @Override
    public boolean isFinished() { //always use in WhileTrue
        return false;
    }
}