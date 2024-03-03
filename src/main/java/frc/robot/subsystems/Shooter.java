// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlFrameEnhanced;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;

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

    private CANSparkFlex shooterTopM;
    private CANSparkFlex shooterBottomM;

    private RelativeEncoder topEncoder;
    private RelativeEncoder bottomEncoder;

    private double velocityCap = 3700;

    private double topVelocitySetpoint = 0;
    private double botVelocitySetpoint = 0;

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));


    public Shooter() {
        currentPercentage = 0.0;
        shooterTopM = new CANSparkFlex(Constants.HardwarePorts.shooterTopM, MotorType.kBrushless);
        shooterBottomM = new CANSparkFlex(Constants.HardwarePorts.shooterBottomM, MotorType.kBrushless);
        topEncoder = shooterTopM.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 7168);
        bottomEncoder = shooterBottomM.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 7168);
        shooterTopM.setInverted(true);
        shooterBottomM.setInverted(true);
        configMotors();

        //topEncoder.setPositionConversionFactor(2);

    }
    
    private void configMotors(){
        shooterTopM.setSmartCurrentLimit(Constants.shooterPeakCurrentLimit);
        shooterTopM.enableVoltageCompensation(12.0);
        shooterBottomM.enableVoltageCompensation(12.0);
        // shooterTopM.getPIDController().setFF((12 / (6784 / 60)) * (28/18));
        shooterTopM.getPIDController().setFF(0.000185);
        shooterTopM.getPIDController().setP(0);
        // shooterTopM.getPIDController().setP(0.005);
        shooterBottomM.getPIDController().setFF(0.000175);
        shooterBottomM.getPIDController().setP(0);
        // shooterBottomM.getPIDController().setP(0.005);
        // shooterBottomM.getPIDController().setFF((12 / (6784 / 60)) * (28/18));
        // shooterTopM.getPIDController().setReference(0.34, ControlType.kVoltage);
        // shooterBottomM.getPIDController().setReference(0.43, ControlType.kVoltage);
    }

    public void voltageDrive(Measure<Voltage> voltage){

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
     * @param speeds: Array containing speeds for Bottom and Top motors (bottom, top)
     * @param MotorLocation: Choose motors to set, 1 is Bottom motor, 2 is Top motor, 0 is Both
     */
    public void setSpeed(double[] speeds, ShooterMotors MotorLocation) { //sets speed of motors using specific speed values

        if (MotorLocation == Shooter.ShooterMotors.BOTTOM) {
            shooterBottomM.set(speeds[0]);
            botVelocitySetpoint = speeds[0];
        } else if (MotorLocation == Shooter.ShooterMotors.TOP) {
            shooterTopM.set(speeds[1]);
            topVelocitySetpoint = speeds[1];
        } else if (MotorLocation == Shooter.ShooterMotors.BOTH) {
            shooterTopM.set(speeds[0]);
            topVelocitySetpoint = speeds[0];
            shooterBottomM.set(speeds[1]);
            botVelocitySetpoint = speeds[1];
        }
    }

    public void setSpeed(double rpm) {
        double[] speeds = new double[3];
        for (int i = 0; i < 3; i++) {
            speeds[i] = rpm;
        }
        setSpeed(speeds, ShooterMotors.BOTH);
    } //what does do 🦅🦅🍔🍔🌭 ??
    
    public double getTopSetpoint() { //gets specific Speed (i hope)
        return topVelocitySetpoint;
    }

    public void setVoltage(double volts){
        shooterTopM.setVoltage(volts);
        shooterBottomM.setVoltage(volts);
    }

    public void setVelocity(double velocity){
        velocity = Math.min(velocity, velocityCap);
        topVelocitySetpoint = velocity;
        botVelocitySetpoint = velocity;

        shooterTopM.getPIDController().setReference(velocity, ControlType.kVelocity);
        shooterBottomM.getPIDController().setReference(velocity, ControlType.kVelocity);
    }


    /**
     * testing purposes only
     */
    double currentPercentage;

    public void setPercentOutput(double percent){
        shooterTopM.set(percent);
        shooterBottomM.set(percent);
        currentPercentage = percent;
    }

    public void incPercentOutput(){
        currentPercentage += 0.05;
        shooterTopM.set(currentPercentage);
        shooterBottomM.set(currentPercentage);
    }

    public void decPercentOutput(){
        currentPercentage -= 0.05;
        shooterTopM.set(currentPercentage);
        shooterBottomM.set(currentPercentage);
    }

    public double getBottomSetpoint() { //gets specific Speed (i hope)
        return botVelocitySetpoint;
    }

    public double[] getBothSpeeds() {
        return new double[]{topEncoder.getVelocity(), bottomEncoder.getVelocity()};
    }
    
    public void setTopVoltage(double voltage){
        shooterTopM.setVoltage(voltage);
    }

    public void setBotVoltage(double voltage){
        shooterBottomM.setVoltage(voltage);
    }

    public void setTopPercent(double percent) {
        shooterTopM.set(percent);
    }
    public void setBotPercent(double percent) {
        shooterBottomM.set(percent);
    }

    public void setTopVelocity(double velocity){
        topVelocitySetpoint = velocity;
        shooterTopM.getPIDController().setReference(topVelocitySetpoint, ControlType.kVelocity);
    }

    public void setBotVelocity(double velocity){
        botVelocitySetpoint = velocity;
        shooterBottomM.getPIDController().setReference(botVelocitySetpoint, ControlType.kVelocity);
    }

    public void setToIdle(){
        setVelocity(1000);
    }

    public double getTopMotorVoltage() {
        return shooterTopM.getBusVoltage();
    }

    public double getBotMotorVoltage() {
        return shooterBottomM.getBusVoltage();
    }

    public boolean velocitiesWithinError(double acceptableError){
        double[] shooterSpeeds = getBothSpeeds();
        double averageError = ((shooterSpeeds[0] - topVelocitySetpoint) + (shooterSpeeds[1] - botVelocitySetpoint))/2;
        return Math.abs(averageError) < acceptableError;
    }


    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/TopSetpoints", topVelocitySetpoint);
        Logger.recordOutput("Shooter/BottomSetpoints", botVelocitySetpoint);
        Logger.recordOutput("Shooter/topMotorSpeed", topEncoder.getVelocity());
        Logger.recordOutput("Shooter/bottomMotorSpeed", bottomEncoder.getVelocity());

        SmartDashboard.putNumber("ShootT Err", 3000 - topEncoder.getVelocity());
        SmartDashboard.putNumber("ShootB Err", 3000 - bottomEncoder.getVelocity());
        
        // SmartDashboard.putString("Top Speed", String.valueOf(currentTopSpeed));
        // SmartDashboard.putString("Bottom Speed", String.valueOf(currentBottomSpeed));
    }

    @Override
    public void simulationPeriodic() {
    }
}