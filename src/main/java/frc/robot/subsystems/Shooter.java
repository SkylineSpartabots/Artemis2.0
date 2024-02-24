// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkFlex;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;

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

    private double currentTopSpeed = 0;
    private double currentBottomSpeed = 0;

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

    private final PWMSparkMax m_shooterMotor = new PWMSparkMax(0);

    // Creates a SysIdRoutine
        //you have to sysId the top and bottom motors seperately, they have different frictional forces in play.
        //finish with top, then edit this below routine for the bottom motor and do the process with that
        SysIdRoutine routineTop = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
                shooterTopM.setVoltage(volts.in(Volts));
              },
              log -> {
                // Record a frame for the shooter motor.
                log.motor("top-shooter-motor")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            shooterTopM.get() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(shooterTopM.getEncoder().getPosition(), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(shooterTopM.getEncoder().getVelocity()/60, RotationsPerSecond));
              }, this)
        );

        public Command sysIdQuasistatic(SysIdRoutine.Direction direction) { //bind these to a button you have to hold
            return routineTop.quasistatic(direction);
          }
          
          public Command sysIdDynamic(SysIdRoutine.Direction direction) {
            return routineTop.dynamic(direction);
          }

    public Shooter() {
        currentPercentage = 0.0;
        shooterTopM = new CANSparkFlex(Constants.HardwarePorts.shooterTopM, MotorType.kBrushless);
        shooterBottomM = new CANSparkFlex(Constants.HardwarePorts.shooterBottomM, MotorType.kBrushless);
        shooterTopM.setInverted(true);
        shooterBottomM.setInverted(true);
        configMotors();

        shooterTopM.getEncoder().setPositionConversionFactor(2);

    }
    
    private void configMotors(){
        shooterTopM.setSmartCurrentLimit(Constants.shooterPeakCurrentLimit);
        shooterTopM.enableVoltageCompensation(12.0);
        shooterBottomM.enableVoltageCompensation(12.0); // units are RPS
        // shooterTopM.getPIDController().setFF(12 / shooterTopM.get() / 60); // Units are RPS
        // shooterBottomM.getPIDController().setFF(12 / shooterBottomM.get() / 60);
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
            currentBottomSpeed = speeds[0];
        } else if (MotorLocation == Shooter.ShooterMotors.TOP) {
            shooterTopM.set(speeds[1]);
            currentTopSpeed = speeds[1];
        } else if (MotorLocation == Shooter.ShooterMotors.BOTH) {
            shooterTopM.set(speeds[0]);
            currentTopSpeed = speeds[0];
            shooterBottomM.set(speeds[1]);
            currentBottomSpeed = speeds[1];
        }
    }

    public void setSpeed(double rpm) {
        double[] speeds = new double[3];
        for (int i = 0; i < 3; i++) {
            speeds[i] = rpm;
        }
        setSpeed(speeds, ShooterMotors.BOTH);
    } //what does do 🦅🦅🍔🍔🌭 ??
    public double getTopSpeed() { //gets specific Speed (i hope)
        return currentTopSpeed;
    }

    public void setVoltage(double volts){
        shooterTopM.setVoltage(volts);
        shooterBottomM.setVoltage(volts);
    }

    public void setVelocity(double velocity){
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
        shooterBottomM.set(currentBottomSpeed);
    }

    public void decPercentOutput(){
        currentPercentage -= 0.05;
        shooterTopM.set(currentPercentage);
        shooterBottomM.set(currentPercentage);
    }

    public double getBottomSpeed() { //gets specific Speed (i hope)
        return currentBottomSpeed;
    }

    public double[] getBothSpeeds() {
        return new double[]{shooterTopM.getEncoder().getVelocity(), shooterBottomM.getEncoder().getVelocity()};
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

    public void setTopSetpoint(double velocity){
        currentTopSpeed = velocity;
    }

    public void setBotSetpoint(double velocity){
        currentBottomSpeed = velocity;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/TopSetpoints", currentTopSpeed);
        Logger.recordOutput("Shooter/BottomSetpoints", currentBottomSpeed);
        Logger.recordOutput("Shooter/topMotorSpeed", shooterTopM.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/bottomMotorSpeed", shooterBottomM.getEncoder().getVelocity());
        
        // SmartDashboard.putString("Top Speed", String.valueOf(currentTopSpeed));
        // SmartDashboard.putString("Bottom Speed", String.valueOf(currentBottomSpeed));
    }

    @Override
    public void simulationPeriodic() {
    }
}
