
package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SetShooter extends Command {
    private final Shooter s_Shooter;
    double[] addedSpeeds = {0, 0}; // bottom, top
    double[] finalSpeeds = {0, 0}; // bottom, top
    Shooter.ShooterMotors motorLoc;

    public SetShooter(Shooter.ShooterStates state, Shooter.ShooterMotors motor) { //Set using states, max or off
        s_Shooter = Shooter.getInstance();
        motorLoc = motor;
        if (motor == Shooter.ShooterMotors.BOTH) {
            finalSpeeds[0] = state.getValue();
            finalSpeeds[1] = state.getValue();
        } else {
            finalSpeeds[motor.getMotor() - 1] = state.getValue();
        }
        addRequirements(s_Shooter);
    }

    public SetShooter(double difference, Shooter.ShooterMotors motor) { //increase or decrease speed incrementally. Also checks if they are within speed bounds
        s_Shooter = Shooter.getInstance();

        if (motor == Shooter.ShooterMotors.BOTH) {
            // cause if they are diff vals then that difference must be preserved by the both increase
            addedSpeeds[0] = s_Shooter.getBottomSetpoint() + difference;
            addedSpeeds[1] = s_Shooter.getTopSpeed() + difference;
        } else { // -1 is for the indexes cause getMotor returns 1 for bottom and 2 for top motor but indexes...
            addedSpeeds[motor.getMotor() - 1] = s_Shooter.getBothSpeeds()[motor.getMotor() - 1] + difference;
        }

        // way less jank way - called a clamp statement or smth (java21 has it built in but i dunno if we run that?)
        finalSpeeds[0] = Math.max(0.0, Math.min(addedSpeeds[0], 1.0));
        finalSpeeds[1] = Math.max(0.0, Math.min(addedSpeeds[1], 1.0));
        // How it works: min gives which ever is lower - addedspeed or 1. then max gives whichever is bigger 0 or result of min

        addRequirements(s_Shooter);
    }

    @Override
    public void initialize() {
       s_Shooter.setSpeed(finalSpeeds, motorLoc);

    }

    @Override
    public void execute() {
    } //sets motors with final speeds

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}