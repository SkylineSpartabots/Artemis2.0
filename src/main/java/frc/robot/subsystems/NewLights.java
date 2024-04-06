package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashMap;
import java.util.Map;

public class NewLights extends SubsystemBase {
    private static NewLights instance;

    public static NewLights getInstance() {
        if (instance == null) {
            instance = new NewLights();
        }
        return instance;
    }

    public NewLights() {
        try {
            port = new SerialPort(9600, SerialPort.Port.kMXP);
            port.setTimeout(0.5);
            clear();
            connected = true;
        } catch (Exception e) {
            connected = false;
        }
        SmartDashboard.putBoolean("LEDs Connected", connected);
        DriverStation.reportWarning("LEDs Connected", connected); //FIXME may cause issues idk
    }

    /**
     * The Serial Port the arduino is on
     */
    private static SerialPort port;
    /**
     * Message to be sent over Serial Port
     */
    private static String command = "";
    private static String previousCommand = command; // start as off/clear?
    /**
     * Status of Arduino connection
     */
    private static boolean connected = false;

    //TODO Colors in Decimal (wtf is this just use hex right? or will that proivde issues later with arduino reading numbers, easier to keep all nums ig?)
    private Map<String, String> colors = new HashMap<String, String>() {
        {
            put("RED", "16711680");
            put("ORANGE", "16753920");
            put("YELLOW", "16775680");
            put("GREEN", "32768");
            put("BLUE", "255");
            put("TURQUOISE", "65535");
            put("PURPLE", "8388736");
            put("VIOLET", "15631086");
            put("PINK", "14027935");
            put("TAN", "16767411");
            put("GOLD", "16766720");
            put("SILVER", "12632256");
            put("WHITE", "16777215");
            put("OFF", "0");
        }
    };

//    private Enum colors { // ONLY COLORS, no modes those are decided by the other parts of the command
//        OFF // SHOULD I HAVE STRINGS OR NUMBERS or color codes?
//
//    }

    //TODO make this and enum, easier and it makes sure they type the name right, its still key value pairing
    // anad with and enum i wont have to do any weird stuff makign sure its right

    private void customDisplay(String color, int pattern, int delay,
                               int brightness, int startPixel, int endPixel) {

    }


    /**
     * Clear the LED strip
     */
    private void clear() {
        //TODO - basically sends an off command
    }

    /**
     * Write to Serial Port, making sure the Port is connected and that we are not writing a command again
     */
    private void send() {
        if (!(previousCommand.equalsIgnoreCase(command)) && connected) {
            port.writeString(command);
            previousCommand = command;
        }
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}


