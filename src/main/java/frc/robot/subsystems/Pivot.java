package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
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
        NULL(0.3),

        GROUND(0.2),
        // Current max is .38, can change later
        SUBWOOFER(Pivot.pivotDegreeToCANcoder(60)),

        FARWING(Pivot.pivotDegreeToCANcoder(20)),

        MIDDLE(0.3),

        AMP_BEFORE_SWING(Pivot.pivotDegreeToCANcoder(60)),

        INTAKE(Pivot.pivotDegreeToCANcoder(70)),

        AMP(Pivot.pivotDegreeToCANcoder(45)); //90 for shooting vertically
        //WING(position);

        private double pos;

        private PivotState(double position) {
            pos = position;
        }
    }


    private TalonFX pivotLeaderM;
    private TalonFX pivotFollowerM;

    // private RelativeEncoder leaderEncoder;
    // private RelativeEncoder followEncoder;
    private CANcoder pivotCANcoder;
    private PivotState currState = PivotState.GROUND;
    private static double pivotCANcoderAngleOffset = 57.89;
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public Pivot() {
        // pivotLeaderM = new CANSparkFlex(Constants.HardwarePorts.pivotLeaderM, MotorType.kBrushless);
        // pivotFollowerM = new CANSparkFlex(Constants.HardwarePorts.pivotFollowerM, MotorType.kBrushless);

        pivotLeaderM = new TalonFX(Constants.HardwarePorts.pivotLeaderM);
        pivotFollowerM = new TalonFX(Constants.HardwarePorts.pivotFollowerM);
        
        // leaderEncoder = pivotLeaderM.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 7168);
        // followEncoder = pivotFollowerM.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 7168);
        
        configMotor(pivotLeaderM);
        pivotLeaderM.setInverted(true);

        configMotor(pivotFollowerM);
        pivotFollowerM.setControl(new Follower(pivotLeaderM.getDeviceID(), true));
        //pivotFollowerM.follow(pivotLeaderM, true);


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
     * Sets the integrated encoder positions of to the specified position. 
     * @param relativePosition The position to set the encoders to. Measured in rotations. 
     */
    // public void resetMotorEncoders(double relativePosition){
    //     leaderEncoder.setPosition(relativePosition);
    // }

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
     * 
     * @return leader motor's integrated encoder value
     */
    // public double getMotorEncoderPosition(){
    //     return leaderEncoder.getPosition();
    // }
    
    /**
     * Gets the current set point of the pivot. 
     * @return Current set point in CANcoder values. 
     */
    public double getSetPoint() {
        return currState.pos;
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

    // public double getMotorPosition() {
    //     return leaderEncoder.getPosition();
    // }

    /**
     * Configures the specified motor with current limit and idle mode plus PID. 
     * @param motor The motor to be configured. 
     */
    private void configMotor(TalonFX motor) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();


        currentLimitsConfigs.SupplyCurrentLimit = Constants.pivotContinuousCurrentLimit;
        currentLimitsConfigs.StatorCurrentLimit = Constants.pivotContinuousCurrentLimit;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentThreshold = Constants.pivotPeakCurrentLimit;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits = currentLimitsConfigs;
        motor.getConfigurator().apply(config);
        // leaderEncoder.setPosition(0.2);


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
        // CANcoder is always initialized to absolute position on boot in Phoenix 6 - https://www.chiefdelphi.com/t/what-kind-of-encoders-are-built-into-the-kraken-motors/447253/7

        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
         
        magnetSensorConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        
        CANcoderConfiguration swerveCanCoderConfig = new CANcoderConfiguration();
        swerveCanCoderConfig.MagnetSensor = magnetSensorConfigs;  
        pivotCANcoder.getConfigurator().apply(swerveCanCoderConfig);
        resetCANcoder(0.2);
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
        Logger.recordOutput("Pivot/CurrentCANcoderRotation", getCANcoderAbsolutePosition());
        // Logger.recordOutput("Pivot/CurrentMotorEncoderRotation", getMotorEncoderPosition());
        Logger.recordOutput("Pivot/AngleSetpoint", getSetPoint());
        SmartDashboard.putNumber("Pivot CANcoder", getCANcoderAbsolutePosition());
        SmartDashboard.putNumber("Pivot measured angle", pivotAngle());
        // SmartDashboard.putNumber("Pivot Motor Encoder", getMotorPosition());
        SmartDashboard.putNumber("Pivot Set Point", currState.pos);
        SmartDashboard.putBoolean("CANcoder working", CANcoderWorking());
        SmartDashboard.putNumber("Pivot Current", getMotorCurrent());
    }
}
