// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

    private double velocityCap = 65;

    private double topVelocitySetpoint = 0;
    private double botVelocitySetpoint = 0;

    final VelocityVoltage topVelocityVoltage = new VelocityVoltage(0);
    final VelocityVoltage bottomVelocityVoltage = new VelocityVoltage(0);

    public Shooter() {
        currentPercentage = 0.0;
        shooterTopM = new TalonFX(Constants.HardwarePorts.shooterTopM);
        shooterBottomM = new TalonFX(Constants.HardwarePorts.shooterBottomM);

        shooterBottomM.setInverted(false);
        shooterTopM.setInverted(false);

        //invertMotor(shooterTopM);
        configMotor(shooterTopM, 0.21, 0.122);
        configMotor(shooterBottomM, 0.362, 0.1225);
    }

    private void configMotor(TalonFX motor, double kS, double kV) {

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.SupplyCurrentLimit = Constants.shooterContinuousCurrentLimit;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentThreshold = Constants.shooterPeakCurrentLimit;

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kS = kS;
        slot0Configs.kV = kV;

        motor.optimizeBusUtilization();


        config.CurrentLimits = currentLimitsConfigs;
        motor.getConfigurator().apply(config);
        motor.getConfigurator().apply(slot0Configs);
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
        return new double[] { (shooterTopM.getVelocity().getValueAsDouble()),
                (shooterBottomM.getVelocity().getValueAsDouble()) };
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
        shooterTopM.setControl(topVelocityVoltage.withVelocity(velocity));
    }

    public void setBotVelocity(double velocity) {
        botVelocitySetpoint = velocity;
        shooterBottomM.setControl(bottomVelocityVoltage.withVelocity(velocity));
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
        return (shooterTopM.getVelocity().getValueAsDouble());
    }

    public double getBottomMotorVelocity() {
        return shooterBottomM.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
        //Logger.recordOutput("Shooter/TopSetpoints", topVelocitySetpoint);
        //Logger.recordOutput("Shooter/BottomSetpoints", botVelocitySetpoint);

        
        // Logger.recordOutput("Shooter/topMotorSpeed", getTopMotorVelocity());
        // Logger.recordOutput("Shooter/bottomMotorSpeed", getBottomMotorVelocity());

        SmartDashboard.putNumber("Shooter top motor velocity", getTopMotorVelocity());
        SmartDashboard.putNumber("Shooter bot motor velocity", getBottomMotorVelocity());


        //Logger.recordOutput("Shooter/topMotorSpeed", getTopMotorVelocity());
        //Logger.recordOutput("Shooter/bottomMotorSpeed", getBottomMotorVelocity());

        // SmartDashboard.putNumber("ShootT Err", 3000 - topEncoder.getVelocity());
        // SmartDashboard.putNumber("ShootB Err", 3000 - bottomEncoder.getVelocity());

        // SmartDashboard.putString("Top Speed", String.valueOf(currentTopSpeed));
        // SmartDashboard.putString("Bottom Speed", String.valueOf(currentBottomSpeed));
    }

    @Override
    public void simulationPeriodic() {
    }
}