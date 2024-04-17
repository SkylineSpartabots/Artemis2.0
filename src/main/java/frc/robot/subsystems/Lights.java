package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
    private static Lights instance;

    private int selected = 0; // LED Mode
    private String binaryString; // String to store the selected mode in binary

    // DIO Pins on the Roborio - use the pins that corrospond with the variable
    // names for cleanliness
    private final DigitalOutput pin2;
    private final DigitalOutput pin3;
    private final DigitalOutput pin4;
    private final DigitalOutput pin5;

    // Timer to Turn the LEDs off after preset time
    private Timer timer;
    private final int offTime = 5; // Time to wait until the LEDs should be turned off

    public static Lights getInstance() {
        if (instance == null) {
            instance = new Lights();
        }
        return instance;
    }

    public enum ledModes {
        OFF(0),
        RED(1),
        ShooterAtSpeed(2), // Shooter At Speed - Orange Solid
        ShooterRamping(3), // Shooter Ramp - Orange Flash Inequal On/Off
        IntakeSuccess(4), // Intake Success - Green Solid
        Intaking(5), // Intaking - Green Flash Equal On/Off
        PURPLE(6),
        PINK(7),
        WHITE(8),
        PINKANT(9);

        private int modeNum;

        public int getModeNum() {
            return modeNum;
        }

        ledModes(int modeNum) {
            this.modeNum = modeNum;
        }
    }

    public Lights() { // Initialize timer and DIO pins
        timer = new Timer();

        pin2 = new DigitalOutput(2);
        pin3 = new DigitalOutput(3);
        pin4 = new DigitalOutput(4);
        pin5 = new DigitalOutput(5);
    }

    /**
     * Set the LED mode with ledModes enum
     * 
     * @param mode
     */
    public void setLEDs(ledModes mode) {
        this.selected = mode.getModeNum();
        setLEDs(selected);
    }

    /**
     * Set the LED mode with an int corrosponding to the enum
     * 
     * @param selected
     */
    public void setLEDs(int selected) {
        timer.start();
        timer.reset();
        this.selected = selected;

        System.out.println("Selected LED Mode: " + selected);

        // Convert to binary
        StringBuilder string = new StringBuilder(Integer.toBinaryString(selected));
        while (string.length() < 4) { // Make sure that the string is 4 bits
            string.insert(0, "0");
        }
        binaryString = string.toString();

        setDigitalOutPins(binaryString);
        System.out.println(binaryString);
    }

    /**
     * @return selected LED mode number
     */
    public int getSelected() {
        return selected;
    }

    /**
     * Set the DIO pins to a string containing a ledMode number in binary
     * 
     * @param binStr String to be written to the DIO pins
     */
    private void setDigitalOutPins(String binStr) {
        pin2.set(Character.getNumericValue(binaryString.charAt(0)) > 0);
        pin3.set(Character.getNumericValue(binaryString.charAt(1)) > 0);
        pin4.set(Character.getNumericValue(binaryString.charAt(2)) > 0);
        pin5.set(Character.getNumericValue(binaryString.charAt(3)) > 0);
    }

    @Override
    public void periodic() {
        // Turns LEDs off after a set amount of time
        if (timer.get() > offTime) {
            setLEDs(ledModes.OFF);
        }
    }

}