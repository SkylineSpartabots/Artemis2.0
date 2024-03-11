 package frc.robot.subsystems;

 import edu.wpi.first.wpilibj.SerialPort;
 import edu.wpi.first.wpilibj2.command.SubsystemBase;
 import edu.wpi.first.wpilibj.Timer;

 public class Lightz extends SubsystemBase {
     private static Lightz instance;
     private int selected = 0;
     private SerialPort arduino;
     private Timer timer;

     public static Lightz getInstance() {
         if (instance == null) {
             instance = new Lightz();
         }
         return instance;
     }

     public enum ledModes { //
         OFF(0),
         RED(1),
         ORANGE(2),
         YELLOW(3),
         GREEN(4),
         BLUE(5),
         PURPLE(6),
         PINK(7),
         WHITE(8),
         REDANT(9);
         private int modeNum;

         public int getModeNum() {
             return modeNum;
         }

         ledModes(int modeNum) {
             this.modeNum = modeNum;
         }
     }

     public Lightz() {
         try {
             arduino = new SerialPort(9600, SerialPort.Port.kUSB);
             System.out.println("Connect kUSB");
         } catch (Exception e) {
             System.out.println("Failed connection on kUSB. Trying kUSB1...");
             try {
                 arduino = new SerialPort(9600, SerialPort.Port.kUSB1);
                 System.out.println("Connect kUSB1");
             } catch (Exception e1) {
                 System.out.println("Failed connection on kUSB1. Trying kUSB2...");
                 try {
                     arduino = new SerialPort(9600, SerialPort.Port.kUSB2);
                     System.out.println("Connect kUSB2");
                 } catch (Exception e2) {
                     System.out.println("Failed connection on kUSB2. Failed on all ports");
                 }
             }
         }
         if (arduino != null){arduino.setTimeout(0.5);}

         timer = new Timer();
         timer.start();
     }

     public void setLEDs(ledModes mode) {
         this.selected = mode.getModeNum();
     }

     public int getLEDs() {
         return selected;
     }

     @Override
     public void periodic() {
         if (arduino != null){
             if (timer.get() >= 10) {

//                 arduino.writeString(String.valueOf(selected));
                 arduino.write(new byte[]{(byte)selected}, 1); // write byte array containing byte converted selected
                 // in theory this should work right?
                 timer.reset();
                 System.out.println("Wrote to Arduino");
             }
             // Two things to try 1: write as string, read as char or string (prolly string) or 2: write as byte[] with (byte)selected conversion and put hexadecimal in arduino code like vid
             if (arduino.getBytesReceived() > 0) {
                 System.out.println(arduino.readString());
             }
         }
     }

     @Override
     public void simulationPeriodic() {
     }
 }
