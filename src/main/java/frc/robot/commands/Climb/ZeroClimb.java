package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ZeroClimb extends Command {
    Climb s_Climb;

    public ZeroClimb() {
        s_Climb = Climb.getInstance();
        addRequirements(s_Climb);
    }

    @Override
    public void initialize() {
        s_Climb.setClimbSpeed(-0.1);
    }

    private double currentThreshold = 20;
    @Override
    public boolean isFinished() {
        return s_Climb.getMotorCurrent() > currentThreshold;
    }

    @Override
    public void end(boolean interrupted) {
        s_Climb.setClimbSpeed(0);
        s_Climb.resetMotorEncoders();      

    }
}
