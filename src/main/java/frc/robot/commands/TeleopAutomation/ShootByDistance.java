
package frc.robot.commands.TeleopAutomation;

import com.fasterxml.jackson.databind.ser.impl.IndexedListSerializer;
import com.google.flatbuffers.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Pivot.SetPivot;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class ShootByDistance extends Command {
    private final Pivot s_Pivot;
    private final Shooter s_Shooter;
    private final Indexer s_Indexer;

    private double distance;

    private double desiredVelocity;
    private double desiredAngle;

    private Command pivotCommand;

    public ShootByDistance(double distance) {
        s_Pivot = Pivot.getInstance();
        s_Shooter = Shooter.getInstance();
        s_Indexer = Indexer.getInstance();

        this.distance = distance;

        addRequirements(s_Pivot, s_Shooter, s_Indexer);
    }

    @Override
    public void initialize() {
        desiredAngle = frc.robot.Constants.ShootingLookupTable.PivotAngleMap.get(distance);
        desiredVelocity = frc.robot.Constants.ShootingLookupTable.FlywheelVelocitiesMap.get(distance);

        s_Shooter.setVelocity(desiredVelocity);
        pivotCommand = new SetPivot(desiredAngle);
        pivotCommand.schedule();
    
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        Command indexerCommand = new IndexForShooting(2);
        indexerCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        double[] shooterSpeeds = s_Shooter.getBothSpeeds();
        double averageError = (shooterSpeeds[0] + shooterSpeeds[1])/2 - desiredVelocity;
        return pivotCommand.isFinished() && Math.abs(averageError) < 75; //tunable encoder velocity
    }
}