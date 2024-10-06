package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
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
        // positions are now in degrees, conversion is in AlignPivot()
        GROUND(16),
        // Current max is .38, can change later
        SUBWOOFER(55),

        STAGE(32), //PLACEHOLDER, change

        FARWING(23),

        AMP_BEFORE_SWING(60),

        INTAKE(70),

        AMP(112); //90 for shooting vertically
        //WING(position);

        private double pos;

        private PivotState(double position) {
            pos = position;
        }

        public double getPos() {
            return pos;
        }
    }


    private TalonFX pivotLeaderM;
    private TalonFX pivotFollowerM;

    private CANcoder pivotCANcoder;
    private static double pivotCANcoderAngleOffset = 57.89;
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public Pivot() {

        pivotLeaderM = new TalonFX(Constants.HardwarePorts.pivotLeaderM);
        pivotFollowerM = new TalonFX(Constants.HardwarePorts.pivotFollowerM);
        
        configMotor(pivotLeaderM);
        pivotLeaderM.setInverted(true);

        configMotor(pivotFollowerM);
        pivotFollowerM.setControl(new Follower(pivotLeaderM.getDeviceID(), true));


        pivotCANcoder = new CANcoder(Constants.HardwarePorts.pivotCANcoderID);
        configCANcoder();
        pivotCANcoder.setPosition(0.2);


    } 

    /**
     * Converts a desired pivot angle in degrees to CANcoder units. 
     * @param pivotDegrees The angle of the pivot in relation to the ground.
     * @return The angle converted to CANcoder units. Units are in rotations. 
     */
    public static double pivotDegreeToCANcoder(double pivotDegrees) {
        return Conversions.degreesToCANcoder(pivotDegrees + pivotCANcoderAngleOffset, 1);
    }

    /**
     * Sets the speed of the motor to 0, effectively stopping it. 
     */
    public void stopMotor(){
        pivotLeaderM.setControl(dutyCycleRequest.withOutput(0));
    }
    /**
     * Gets the current position measured by the CANcoder
     * @return Current position measured by CANcoder. Measured in rotations. 
     */
    public double getCANcoderPosition() {
        return pivotCANcoder.getPosition().getValueAsDouble();
    }

    /**
     * Gets the current absolute position measured by the CANcoder. 
     * @return Current absolute position measured by CANcoder. The range is between 0.0 and 1.0, as specified in the range for the Pivot's CANcoder configuration. 
     */
    public double getCANcoderAbsolutePosition() {
        return pivotCANcoder.getAbsolutePosition().getValueAsDouble();
    }

    /**
     * Sets the voltage of the pivot motor, use setPercentageOutput for backwards movement
     * @param voltage Desired voltage. 
     */
    public void setVoltage(double voltage) {
        pivotLeaderM.setControl(voltageRequest.withOutput(voltage));
    }
    
    public boolean CANcoderWorking() {
        return !pivotCANcoder.getFault_BadMagnet().getValue() && !pivotCANcoder.getFault_Hardware().getValue();
    }

    /**
     * Sets percentage output, value between -1.0 and 1.0. Positive increases pivot's angle, negative decreases
     * @param percentage The desired percentage to set the motor to. 
     */
    public void setPercentageOutput(double percentage){
        pivotLeaderM.setControl(dutyCycleRequest.withOutput(percentage));
    }

    public double getMotorCurrent(){
        return (Math.abs(pivotLeaderM.getStatorCurrent().getValueAsDouble()) + Math.abs(pivotFollowerM.getStatorCurrent().getValueAsDouble())) / 2;
    }

    /**
     * Configures the specified motor with current limit and idle mode plus PID. 
     * @param motor The motor to be configured. 
     */
    private void configMotor(TalonFX motor) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();

        motor.optimizeBusUtilization();

        currentLimitsConfigs.SupplyCurrentLimit = Constants.pivotContinuousCurrentLimit;
        currentLimitsConfigs.StatorCurrentLimit = Constants.pivotContinuousCurrentLimit;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentThreshold = Constants.pivotPeakCurrentLimit;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits = currentLimitsConfigs;
        motor.getConfigurator().apply(config);
    }

    /**
     * Resets the CANcoder to the specified value. 
     * @param value Value to reset the CANcoder to. Units are in rotations. 
     */
    public void resetCANcoder(double value) {
        pivotCANcoder.setPosition(value);
    }

    /**
     * Configures the pivot CANcoder with sensor direction and absolute range. 
     */
    private void configCANcoder(){  

        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
         
        magnetSensorConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        
        CANcoderConfiguration CanCoderConfig = new CANcoderConfiguration();
        CanCoderConfig.MagnetSensor = magnetSensorConfigs;  
        pivotCANcoder.getConfigurator().apply(CanCoderConfig);
        // resetCANcoder(0.2);
    }

    public double getPivotVelocity(){
        return pivotLeaderM.getVelocity().getValueAsDouble();
    }

    /**
     * Calculates the current angle the pivot is at in degrees, relative to the ground. 
     * @return The current angle the pivot is at relative to the ground. This is measured by converting CANcoder units to degrees and then subtracting the offset. 
     */
    public double pivotAngle() {
        return Conversions.CANcoderToDegrees(getCANcoderAbsolutePosition(), 1) - pivotCANcoderAngleOffset;
    }

    @Override
    public void periodic() {
        //Logger.recordOutput("Pivot/CurrentCANcoderRotation", getCANcoderAbsolutePosition());
        // Logger.recordOutput("Pivot/CurrentMotorEncoderRotation", getMotorEncoderPosition());
        //Logger.recordOutput("Pivot/PivotCurrent", getMotorCurrent());
        //Logger.recordOutput("Pivot/RotInDegrees", pivotAngle());
        //Logger.recordOutput("Pivot/CANCoderStatus", CANcoderWorking());
        SmartDashboard.putNumber("Pivot CANcoder", getCANcoderAbsolutePosition());
        SmartDashboard.putNumber("Pivot measured angle", pivotAngle());
        // SmartDashboard.putNumber("Pivot Motor Encoder", getMotorPosition());
        SmartDashboard.putBoolean("CANcoder working", CANcoderWorking());
        SmartDashboard.putNumber("Pivot Current", getMotorCurrent());
        SmartDashboard.putNumber("pivot velocity", getPivotVelocity());
    }
}