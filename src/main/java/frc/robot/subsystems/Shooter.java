// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {

    private static Shooter instance;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private TalonFX shooterTopM;
    private TalonFX shooterBottomM;
    
    private TalonFXSensorCollection shooterTopSensor;
    private TalonFXSensorCollection shooterBottomSensor;

    private RelativeEncoder topEncoder;
    private RelativeEncoder bottomEncoder;

    private double velocityCap = 3000;

    private double topVelocitySetpoint = 0;
    private double botVelocitySetpoint = 0;

    final VelocityVoltage topVelocityVoltage = new VelocityVoltage(0);
    final VelocityVoltage bottomVelocityVoltage = new VelocityVoltage(0);

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

    public Shooter() {
        currentPercentage = 0.0;
        shooterTopM = new TalonFX(Constants.HardwarePorts.shooterTopM);
        shooterBottomM = new TalonFX(Constants.HardwarePorts.shooterBottomM);

        // topEncoder = shooterTopM.getEncoder(SparkRelativeEncoder.Type.kQuadrature,
        // 7168);
        // bottomEncoder =
        // shooterBottomM.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 7168);
        invertMotors(shooterTopM);
        configMotors();

        // topEncoder.setPositionConversionFactor(2);

    }

    private void invertMotors(TalonFX motor) {
        System.out.println("INVERT F");
        motor.setInverted(true);
    }

    private void configMotors() {

        TalonFXConfiguration topConfig = new TalonFXConfiguration();
        TalonFXConfiguration botConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();

        Slot0Configs shooterTopConfigs = new Slot0Configs();
        shooterTopConfigs.kS = 0.21; // voltage to overcome static friction
        shooterTopConfigs.kV = 0.005;
        shooterTopConfigs.kP = 0;
        shooterTopConfigs.kI = 0;
        shooterTopConfigs.kD = 0;

        Slot1Configs shooterBottomConfigs = new Slot1Configs();
        shooterBottomConfigs.kS = 0.362;
        shooterBottomConfigs.kV = 0.005;
        shooterBottomConfigs.kP = 0;
        shooterBottomConfigs.kI = 0;
        shooterBottomConfigs.kD = 0;

        currentLimitsConfigs.SupplyCurrentLimit = Constants.shooterContinuousCurrentLimit;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentThreshold = Constants.shooterPeakCurrentLimit;
        topConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        topConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        topConfig.CurrentLimits = currentLimitsConfigs;
        botConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        botConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        botConfig.CurrentLimits = currentLimitsConfigs;
        shooterTopM.getConfigurator().apply(topConfig);
        shooterBottomM.getConfigurator().apply(botConfig);
        shooterTopM.getConfigurator().apply(shooterTopConfigs);
        shooterBottomM.getConfigurator().apply(shooterBottomConfigs);

        topVelocityVoltage.Slot = 0;
        bottomVelocityVoltage.Slot = 1;
    }

    public void voltageDrive(Measure<Voltage> voltage) {

    }

    public enum ShooterStates {
        MAX(1),
        OFF(0);

        private double speed;

        public double getValue() {
            return speed;
        }

        ShooterStates(double speed) {
            this.speed = speed;
        }

    }

    public enum ShooterMotors {
        BOTTOM(1),
        TOP(2),
        BOTH(0);

        private int motor;

        public int getMotor() {
            return motor;
        }

        ShooterMotors(int motor) {
            this.motor = motor;
        }
    }

    /**
     * @param speeds:        Array containing speeds for Bottom and Top motors
     *                       (bottom, top)
     * @param MotorLocation: Choose motors to set, 1 is Bottom motor, 2 is Top
     *                       motor, 0 is Both
     */
    /*
     * public void setSpeed(double[] speeds, ShooterMotors MotorLocation) { //sets
     * speed of motors using specific speed values
     * 
     * if (MotorLocation == Shooter.ShooterMotors.BOTTOM) {
     * shooterBottomM.set(speeds[0]);
     * botVelocitySetpoint = speeds[0];
     * } else if (MotorLocation == Shooter.ShooterMotors.TOP) {
     * shooterTopM.set(speeds[1]);
     * topVelocitySetpoint = speeds[1];
     * } else if (MotorLocation == Shooter.ShooterMotors.BOTH) {
     * shooterTopM.set(speeds[0]);
     * topVelocitySetpoint = speeds[0];
     * shooterBottomM.set(speeds[1]);
     * botVelocitySetpoint = speeds[1];
     * }
     * }
     */

    /*
     * public void setSpeed(double speed) { //-1.0 - 1.0, not rpm
     * double[] speeds = new double[3];
     * for (int i = 0; i < 3; i++) {
     * speeds[i] = speed;
     * }
     * setSpeed(speeds, ShooterMotors.BOTH);
     * } //what does do 🦅🦅🍔🍔🌭 ??
     */

    public double getTopSetpoint() { // gets specific Speed (i hope)
        return topVelocitySetpoint;
    }

    public void setVoltage(double volts) {
        shooterTopM.setVoltage(volts);
        shooterBottomM.setVoltage(volts);
    }

    public void setVelocity(double velocity) { // rotations per second
        velocity = Math.min(velocity, velocityCap);
        topVelocitySetpoint = velocity;
        botVelocitySetpoint = velocity;

        double rps = velocity / 60;
        shooterTopM.setControl(topVelocityVoltage.withVelocity(rps));
        shooterBottomM.setControl(bottomVelocityVoltage.withVelocity(rps));
    }

    /**
     * testing purposes only
     */
    double currentPercentage;

    public void setPercentOutput(double percent) {
        shooterTopM.set(percent);
        shooterBottomM.set(percent);
        currentPercentage = percent;
    }

    public void incPercentOutput() {
        currentPercentage += 0.05;
        shooterTopM.set(currentPercentage);
        shooterBottomM.set(currentPercentage);
    }

    public void decPercentOutput() {
        currentPercentage -= 0.05;
        shooterTopM.set(currentPercentage);
        shooterBottomM.set(currentPercentage);
    }

    public double getBottomSetpoint() { // gets specific Speed (i hope)
        return botVelocitySetpoint;
    }

    public double[] getBothSpeeds() {
        return new double[] { (shooterTopM.getVelocity().getValueAsDouble() * 60),
                (shooterBottomM.getVelocity().getValueAsDouble() * 60) };
    }

    public void setTopVoltage(double voltage) {
        shooterTopM.setVoltage(voltage);
    }

    public void setBotVoltage(double voltage) {
        shooterBottomM.setVoltage(voltage);
    }

    public void setTopPercent(double percent) {
        shooterTopM.set(percent);
    }

    public void setBotPercent(double percent) {
        shooterBottomM.set(percent);
    }

    public void setTopVelocity(double velocity) {
        topVelocitySetpoint = velocity;
        double rps = velocity / 60;
        shooterTopM.setControl(topVelocityVoltage.withVelocity(rps));
    }

    public void setBotVelocity(double velocity) {
        botVelocitySetpoint = velocity;
        double rps = velocity / 60;
        shooterBottomM.setControl(topVelocityVoltage.withVelocity(rps));
    }

    public void setToIdle() {
        setVelocity(1000);
    }

    public double getTopMotorVoltage() {
        return shooterTopM.getMotorVoltage().getValueAsDouble();
    }

    public double getBotMotorVoltage() {
        return shooterBottomM.getMotorVoltage().getValueAsDouble();
    }

    public boolean velocitiesWithinError(double acceptableError) {
        double[] shooterSpeeds = getBothSpeeds();
        double averageError = ((shooterSpeeds[0] - topVelocitySetpoint) + (shooterSpeeds[1] - botVelocitySetpoint)) / 2;
        return Math.abs(averageError) < acceptableError;
    }

    public double getTopMotorVelocity() {
        return ((shooterTopSensor.getIntegratedSensorVelocity() * 600)/2048);
    }

    public double getBottomMotorVelocity() {
        return ((shooterBottomSensor.getIntegratedSensorVelocity() * 600)/2048);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/TopSetpoints", topVelocitySetpoint);
        Logger.recordOutput("Shooter/BottomSetpoints", botVelocitySetpoint);
        Logger.recordOutput("Shooter/topMotorSpeed", getTopMotorVelocity());
        Logger.recordOutput("Shooter/bottomMotorSpeed", getBottomMotorVelocity());

        SmartDashboard.putNumber("Shooter top motor velocity", getTopMotorVelocity());
        SmartDashboard.putNumber("Shooter bot motor velocity", getBottomMotorVelocity());
        // Logger.recordOutput("Shooter/topMotorSpeed", topEncoder.getVelocity());
        // Logger.recordOutput("Shooter/bottomMotorSpeed", bottomEncoder.getVelocity());

        // SmartDashboard.putNumber("ShootT Err", 3000 - topEncoder.getVelocity());
        // SmartDashboard.putNumber("ShootB Err", 3000 - bottomEncoder.getVelocity());

        // SmartDashboard.putString("Top Speed", String.valueOf(currentTopSpeed));
        // SmartDashboard.putString("Bottom Speed", String.valueOf(currentBottomSpeed));
    }

    @Override
    public void simulationPeriodic() {
    }
}