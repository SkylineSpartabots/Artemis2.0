package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lightz extends SubsystemBase {
    private static Lightz instance;
    private int selected = 0;
    private DigitalOutput[] pins; // Its like swerve
    private final int pinCount = 4;

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
        for (int i = 0; i < pinCount; i++) {
            pins[i] = new DigitalOutput(i); // Might wanna be in a try catch? Yo no sé
        }
    }

    public void setLEDs(ledModes mode) {
        this.selected = mode.getModeNum();

        System.out.println("Selected LED Mode: " + selected);

        // Convert to binary
        String binaryString = Integer.toBinaryString(selected);
        while (binaryString.length() < 4) { // Make sure that the string is 4 bits
            binaryString = '0' + binaryString;
        }
        setDigitalOutPins(binaryString);
    }

    public int getLEDs() {
        return selected;
    }

    private void setDigitalOutPins(String binStr) {
        for (int i = 0; i < pins.length; i++) {
            pins[i].set(Character.getNumericValue(binStr.charAt(i)) > 0);
            System.out.println(pins[i].get());
        }
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
