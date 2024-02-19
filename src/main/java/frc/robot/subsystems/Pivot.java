package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {
    public static Pivot instance;
    public static Pivot getInstance() {
        if (instance == null) {
            instance = new Pivot();
        }
        return instance;
    }

    public enum PivotState {
        GROUND(0);

        private double pos;

        private PivotState(double position) {
            pos = position;
        }
    }

    private CANSparkFlex pivotLeaderM;
    private CANSparkFlex pivotFollowerM;
    private CANcoder pivotCANcoder;
    private PivotState currState = PivotState.GROUND;

    public Pivot() {
        pivotLeaderM = new CANSparkFlex(Constants.HardwarePorts.pivotLeftM, MotorType.kBrushless);
        configMotor();

        pivotFollowerM = new CANSparkFlex(Constants.HardwarePorts.pivotRightM, MotorType.kBrushless);
        pivotFollowerM.follow(pivotLeaderM);
        pivotCANcoder = new CANcoder(Constants.HardwarePorts.pivotCANcoderID);
        configCANcoder();
    } 

    /**
     * Sets the desired state for the pivot
     * @param state Desired state
     */
    public void setState(PivotState state) {
        currState = state;
    }

    /**
     * Gets the current position measured by the CANcoder
     * @return Current position measured by CANcoder
     */
    public double getCANcoderPosition() {
        return pivotCANcoder.getPosition().getValueAsDouble();
    }
    /**
     * Gets the current set point of the pivot. 
     * @return Current set point in CANcoder values. 
     */
    public double getSetPoint() {
        return currState.pos;
    }

    /**
     * Sets the voltage of the pivot motor
     * @param Desired voltage. 
     */
    public void setVoltage(double voltage) {
        pivotLeaderM.setVoltage(voltage);
    }

    /**
     * Configures the pivot motor with current limit, idle mode, and PID values
     */
    private void configMotor() {
        pivotLeaderM.setSmartCurrentLimit(Constants.pivotPeakCurrentLimit);
        pivotLeaderM.setIdleMode(IdleMode.kBrake);

        pivotLeaderM.getPIDController().setP(Constants.hardwarePIDs.pivotkP);
        pivotLeaderM.getPIDController().setI(Constants.hardwarePIDs.pivotkI);
        pivotLeaderM.getPIDController().setD(Constants.hardwarePIDs.pivotkD);
    }

    /**
     * Configures the pivot CANcoder with sensor direction and absolute range. 
     */
    private void configCANcoder(){  
        /* Swerve CANCoder Configuration */
        // CANcoder is always initialized to absolute position on boot in Phoenix 6 - https://www.chiefdelphi.com/t/what-kind-of-encoders-are-built-into-the-kraken-motors/447253/7

        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        magnetSensorConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        
        CANcoderConfiguration swerveCanCoderConfig = new CANcoderConfiguration();
        swerveCanCoderConfig.MagnetSensor = magnetSensorConfigs;  
        pivotCANcoder.getConfigurator().apply(swerveCanCoderConfig);
    }
}
