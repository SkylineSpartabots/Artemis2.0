// package frc.robot.subsystems;

// import javax.sound.midi.MidiEvent;

// import com.ctre.phoenix6.signals.ControlModeValue;

// import com.revrobotics.CANSparkFlex;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.ColorMatch;
// import com.revrobotics.ColorSensorV3;

// import edu.wpi.first.wpilibj.I2C;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class IntakeShooter extends SubsystemBase {

//     private static IntakeShooter mIntakeShooter;

//     private CANSparkFlex mIntakeMotor;
//     private CANSparkFlex mLeaderShooter;
//     private CANSparkFlex mFollowerShooter;


//     private static final I2C.Port onboardI2C = I2C.Port.kOnboard;
//     private static ColorSensorV3 m_intakeSensor;

//     private static final ColorMatch m_colorMatcherIntake = new ColorMatch();

//     public static IntakeShooter getInstance(){
//         if(mIntakeShooter == null){
//             mIntakeShooter = new IntakeShooter();
//         }
//         return mIntakeShooter;
//     }

//     public IntakeShooter(){
//         mIntakeMotor = new CANSparkFlex(Constants.HardwarePorts.intakeMotor, MotorType.kBrushless);
//         configIntakeMotor();

//         mLeaderShooter = new CANSparkFlex(Constants.HardwarePorts.shooterLeaderMotor, MotorType.kBrushless);
//         configLeaderMotor();

//         mFollowerShooter = new CANSparkFlex(Constants.HardwarePorts.shooterFollowerMotor, MotorType.kBrushless);
//         configFollowerMotor();

//         m_intakeSensor = new ColorSensorV3(onboardI2C);
//         m_colorMatcherIntake.addColorMatch(new Color(255, 165, 0)); //rgb values for orange, could be inaccurate. YBVS
//     }

//     /**
//      * Sets the percentage output of the intake and shooter motors
//      * @param val a value from [0, 1]
//      */
//     public void setPercentage(double val){
//         mLeaderShooter.set(val);
//         mIntakeMotor.set(val);
//     }

//     /**
//      * Sets the output of the intake motor
//      * @param val either a percentage from [0, 1] or desired voltage
//      * @param voltageControl is voltage control if true
//      */
//     public void setIntake(double val, boolean voltageControl){
//         if(voltageControl){
//             mIntakeMotor.setVoltage(val);
//         } else {
//             mIntakeMotor.set(val);
//         }
//     }

//     /**
//      * Equivalent to setIntake(val, false)
//      * @param val is the desired percentage output
//      */
//     public void setIntake(double val){
//         setIntake(val, false);
//     }

//     public void configIntakeMotor(){
//         mIntakeMotor.setSmartCurrentLimit(Constants.intakePeakCurrentLimit);
//         mIntakeMotor.setInverted(false);
//         mIntakeMotor.setIdleMode(Constants.shooterNeutralMode);
//         mIntakeMotor.setOpenLoopRampRate(Constants.openLoopRamp);

//     }

//     public void configLeaderMotor(){
//         mLeaderShooter.setSmartCurrentLimit(Constants.intakePeakCurrentLimit);
//         mLeaderShooter.setInverted(true);
//         mLeaderShooter.setIdleMode(Constants.shooterNeutralMode);
//         mLeaderShooter.setOpenLoopRampRate(Constants.openLoopRamp);


//         // mLeaderShooter.setOpenLoopRampRate(Constants.SwerveConstants.openLoopRamp);
//         // mLeaderShooter.setClosedLoopRampRate(Constants.SwerveConstants.closedLoopRamp);

//         mLeaderShooter.getPIDController().setP(Constants.hardwarePIDs.shooterkP);
//         mLeaderShooter.getPIDController().setI(Constants.hardwarePIDs.shooterkI);
//         mLeaderShooter.getPIDController().setD(Constants.hardwarePIDs.shooterkP);
//     }

//     public void configFollowerMotor(){
//         mFollowerShooter.follow(mLeaderShooter, false);
//         mFollowerShooter.setIdleMode(Constants.shooterNeutralMode);
        
//         // mLeaderShooter.getPIDController().setP(Constants.SwerveConstants.driveKP);
//         // mLeaderShooter.getPIDController().setI(Constants.SwerveConstants.driveKI);
//         // mLeaderShooter.getPIDController().setD(Constants.SwerveConstants.driveKD);
//     }

//     /**
//      * 
//      * @return whether the intake color sensor's proximity value is greater than a set threshold
//      */
//     public boolean hasNote(){
//         return m_intakeSensor.getProximity() >= Constants.noteIntakeDistance;
//     }
// }
