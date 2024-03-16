package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lightz extends SubsystemBase {
    private static Lightz instance;
    private int selected = 0;
    private String binaryString;
    private final DigitalOutput pin0;
    private final DigitalOutput pin1;
    private final DigitalOutput pin2;
    private final DigitalOutput pin3;

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
        pin0 = new DigitalOutput(0);
        pin1 = new DigitalOutput(1);
        pin2 = new DigitalOutput(2);
        pin3 = new DigitalOutput(3);
    }

    public void setLEDs(ledModes mode) {
        this.selected = mode.getModeNum();

        System.out.println("Selected LED Mode: " + selected);

        // Convert to binary
        binaryString = Integer.toBinaryString(selected);
        while (binaryString.length() < 4) { // Make sure that the string is 4 bits
            binaryString = '0' + binaryString;
        }
        setDigitalOutPins(binaryString);
    }

    public int getLEDs() {
        return selected;
    }

    private void setDigitalOutPins(String binStr) {
        pin0.set(Character.getNumericValue(binaryString.charAt(0)) > 0);
        pin1.set(Character.getNumericValue(binaryString.charAt(1)) > 0);
        pin2.set(Character.getNumericValue(binaryString.charAt(2)) > 0);
        pin3.set(Character.getNumericValue(binaryString.charAt(3)) > 0);

        // Debug
        System.out.println(pin0.get());
        System.out.println(pin1.get());
        System.out.println(pin2.get());
        System.out.println(pin3.get());
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
