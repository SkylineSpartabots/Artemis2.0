// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    private static Shooter instance; //  Static variable meaning that it is shared between all instances of the class. Belongs to the class itself.

    // Theis makes sure that there is only ever one instance of the Shooter class. If the instance has not already been created (someone hasn't created an instance of it yet), then it creates a new instance of the Shooter class. If it has already been created, it returns the instance that has already been created.
    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private TalonFX shooterTopM;
    private TalonFX shooterBottomM;

    // Used to limit the speed of the shooter
    private double velocityCap = 65;

    private double topVelocitySetpoint = 0;
    private double botVelocitySetpoint = 0;

    final VelocityVoltage topVelocityVoltage = new VelocityVoltage(0);
    final VelocityVoltage bottomVelocityVoltage = new VelocityVoltage(0);

    // Constructor
    public Shooter() {
        // Initialize the shooter motors with their IDs in Constants.java
        shooterTopM = new TalonFX(Constants.HardwarePorts.shooterTopM);
        shooterBottomM = new TalonFX(Constants.HardwarePorts.shooterBottomM);

        shooterBottomM.setInverted(false);
        shooterTopM.setInverted(false);

        //invertMotor(shooterTopM);
        configMotor(shooterTopM, 0.21, 0.122);
        configMotor(shooterBottomM, 0.362, 0.1225);
    }

    /**
     * Used to set configurations to the motor themselves such as current limits.
     * @param motor The motor object to configure
     * @param kS Value for kS Feedforward. Voltage needed to overcome static friction.
     * @param kV Value for kV Feedforward. Voltage needed to overcome counter electromotive forces and additional friction that arises with speed.
     */
    private void configMotor(TalonFX motor, double kS, double kV) {

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Break or coast. When powered should the motor keep itself in place (brake) or allow itself to move freely (coast)

        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.SupplyCurrentLimit = Constants.shooterContinuousCurrentLimit;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentThreshold = Constants.shooterPeakCurrentLimit;

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kS = kS;
        slot0Configs.kV = kV;

        config.CurrentLimits = currentLimitsConfigs;
        motor.getConfigurator().apply(config);
        motor.getConfigurator().apply(slot0Configs);
    }

    // Getters for the shooter motors
    /**
     * Get both motor setpoints.
     * @return Double Array where the first element is the top motor setpoint and the second element is the bottom motor setpoint.
     */
    public double[] getBothSetpoints() {
        return new double[] { topVelocitySetpoint, botVelocitySetpoint };
    }

    public double getTopSetpoint() {
        return topVelocitySetpoint;
    }
    public double getBottomSetpoint() {
        return botVelocitySetpoint;
    }
    /**
     * Get both motor velocities in Rotations Per Second.
     * @return Double Array where the first element is the top motor velocity and the second element is the bottom motor velocity.
     */
    public double[] getBothVelocities() {
        return new double[] { (shooterTopM.getVelocity().getValueAsDouble()),
                (shooterBottomM.getVelocity().getValueAsDouble()) };
    }

    /**
     * Get the velocity of the top shooter motor in Rotations Per Second.
      * @return Velocity of the top shooter motor in Rotations Per Second.
     */
    public double getTopVelocity() {
        return shooterTopM.getVelocity().getValueAsDouble();
    }

    /**
     * Get the velocity of the bottom shooter motor in Rotations Per Second.
     * @return Velocity of the bottom shooter motor in Rotations Per Second.
     */
    public double getBottomVelocity() {
        return shooterBottomM.getVelocity().getValueAsDouble();
    }
    /**
     * Get both motor voltages.
     * @return Double Array where the first element is the top motor voltage and the second element is the bottom motor voltage.
     */
    public double[] getBothVoltages() {
        return new double[] { (shooterTopM.getMotorVoltage().getValueAsDouble()),
                (shooterBottomM.getMotorVoltage().getValueAsDouble()) };
    }
    public double getTopMotorVoltage() {
        return shooterTopM.getMotorVoltage().getValueAsDouble();
    }

    public double getBottomMotorVoltage() {
        return shooterBottomM.getMotorVoltage().getValueAsDouble();
    }

// Setters for the shooter motors

    /**
     * Set the velocity of both the shooter motors.
     * @param velocity Velocity in Rotations Per Second to set shooter motors to.
     */
    public void setVelocity(double velocity) { // rotations per second
        velocity = Math.min(velocity, velocityCap); // Make sure that the inputted velocity is not greater than the velocity cap
        topVelocitySetpoint = velocity;
        botVelocitySetpoint = velocity;

        shooterTopM.setControl(topVelocityVoltage.withVelocity(velocity));
        shooterBottomM.setControl(bottomVelocityVoltage.withVelocity(velocity));
    }

    /**
     * Set the velocity of the top shooter motor.
     * @param velocity Velocity in Rotations Per Second to set top shooter motor to.
     */
    public void setTopVelocity(double velocity) {
        topVelocitySetpoint = velocity;

        shooterTopM.setControl(topVelocityVoltage.withVelocity(velocity));
    }

    /**
     * Set the velocity of the bottom shooter motor.
     * @param velocity Velocity in Rotations Per Second to set bottom shooter motor to.
     */
    public void setBottomVelocity(double velocity) {
        botVelocitySetpoint = velocity;

        shooterBottomM.setControl(bottomVelocityVoltage.withVelocity(velocity));
    }

    /**
     * Set the voltage of both the shooter motors.
     * @param volts Voltage to set shooter motors to.
     */
    public void setVoltage(double volts) {
        shooterTopM.setVoltage(volts);
        shooterBottomM.setVoltage(volts);
    }

    public void setTopVoltage(double voltage) {
        shooterTopM.setVoltage(voltage);
    }

    public void setBottomVoltage(double voltage) {
        shooterBottomM.setVoltage(voltage);
    }


    /**
     * Set the speed of both the shooter motors.
     * @param volts Percent output to set shooter motors to. From -1 to 1.
     */
    public void setPercentage(double volts) {
        shooterTopM.setVoltage(volts);
        shooterBottomM.setVoltage(volts);
    }

    /**
     * Set the speed of the top shooter motor.
     * @param percent Percent output to set top shooter motor to. From -1 to 1.
     */
    public void setTopPercent(double percent) {
        shooterTopM.set(percent);
    }

    /**
     * Set the speed of the bottom shooter motor.
     * @param percent Percent output to set bottom shooter motor to. From -1 to 1.
     */
    public void setBottomPercent(double percent) {
        shooterBottomM.set(percent);
    }


    /**
     * Are the velocities of the shooter motors within an acceptable error?
     * @param acceptableError The acceptable error in rotations per second.
     * @return
     */
    public boolean velocitiesWithinError(double acceptableError) {
        double[] shooterSpeeds = getBothVelocities();
        double averageError = ((shooterSpeeds[0] - topVelocitySetpoint) + (shooterSpeeds[1] - botVelocitySetpoint)) / 2;
        return Math.abs(averageError) < acceptableError;
    }



    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/TopSetpoints", topVelocitySetpoint);
        Logger.recordOutput("Shooter/BottomSetpoints", botVelocitySetpoint);

        
         Logger.recordOutput("Shooter/topMotorVelocity", getTopVelocity());
         Logger.recordOutput("Shooter/bottomMotorVelocity", getBottomVelocity());

        SmartDashboard.putNumber("Shooter/topMotorVelocity", getTopVelocity());
        SmartDashboard.putNumber("Shooter/bottomMotorVelocity", getBottomVelocity());
    }

    @Override
    public void simulationPeriodic() {
    }
}