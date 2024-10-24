package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ManualClimb extends Command {
    Climb s_Climb;
    boolean goingUp;

    public ManualClimb(boolean goingUp) {
        s_Climb = Climb.getInstance();
        this.goingUp = goingUp;
        addRequirements(s_Climb);
    }

    @Override
    public void initialize() { 
        if(goingUp){
            s_Climb.setClimbSpeed(-0.3);
        } else{
            s_Climb.setClimbSpeed(0.3);
        }
    }

    
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        s_Climb.setClimbSpeed(0);
    }
}
