
 package frc.robot.commands;

 import edu.wpi.first.wpilibj2.command.Command;
 import frc.robot.subsystems.Indexer;
 import frc.robot.subsystems.Intake;
 import frc.robot.subsystems.Lightz;

 public class SetLightz extends Command {
     private final Lightz s_Lightz;
     private Lightz.ledModes selectedMode;

     public SetLightz(Lightz.ledModes mode) {
         s_Lightz = Lightz.getInstance();
         this.selectedMode = mode;
         addRequirements(s_Lightz);
     }

     @Override
     public void initialize() {
     }

     @Override
     public void execute() {
         s_Lightz.setLEDs(selectedMode);
     }

     @Override
     public void end(boolean interrupted) {
     }

     @Override
     public boolean isFinished() {
         return false;
     }
 }