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

    public enum ledModes {
        RED(0),
        ORANGE(1),
        YELLOW(2),
        GREEN(3),
        BLUE(4),
        PURPLE(5),
        PINK(6),
        WHITE(7),
        OFF(8),
        ;
        private int modeNum;

        public int getModeNum() {
            return modeNum;
        }

        ledModes(int modeNum) {
            this.modeNum = modeNum;
        }
    }

    public Lightz() { // idk if this code should be in this
        // constructor or in the setLEDs method
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
        if (timer.get() >= 10) {
            arduino.write(new byte[]{(byte) selected}, 1); // in theory this should work right?
            timer.reset();
            System.out.println("Wrote to Arduino");
        }
        if (arduino.getBytesReceived() > 0) {
            System.out.println(arduino.readString());
        }
    }

    @Override
    public void simulationPeriodic() {
    }
}
