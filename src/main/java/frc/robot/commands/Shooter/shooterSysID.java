package frc.robot.commands.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Shooter;

public class shooterSysID extends SequentialCommandGroup {
    private final Shooter s_Shooter;

    public shooterSysID() {
        s_Shooter = Shooter.getInstance();
        addRequirements(s_Shooter);
        addCommands(
                s_Shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
                s_Shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
                s_Shooter.sysIdDynamic(SysIdRoutine.Direction.kForward),
                s_Shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }
}
