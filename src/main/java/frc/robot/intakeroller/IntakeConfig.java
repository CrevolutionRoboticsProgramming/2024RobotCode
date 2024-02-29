package frc.robot.intakeroller;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.revrobotics.CANSparkMax;

public class IntakeConfig {
    //motor IDs have to be changed
    public static final int kIntakeSparkID = 20;

    public static final boolean kShooterMotorInverted = false;

    public static final CANSparkMax.IdleMode kRollerIdleMode = CANSparkMax.IdleMode.kCoast;

    // Shooter Velocity & Percent Output Controllers
    public static final VelocityDutyCycle intakeVelocity = new VelocityDutyCycle(0);
    public static final DutyCycleOut intakePercentOutput = new DutyCycleOut(0);

    // Shooter Pivot Pos & Percent Output
    public static final MotionMagicDutyCycle intakePivotPosition = new MotionMagicDutyCycle(0);
    public static final DutyCycleOut intakePivotPercentOutput = new DutyCycleOut(0);

    public static final int intakeCurrentLimit = 35;
    public static final int intakeCurrentThreshold = 60;
    public static final double intakeCurrentThresholdTime = 0.1;
    public static final boolean intakeEnableCurrentLimit = true;

     public static final int kHOLimitSwitch = 53;
     public static final int kIndexerBeamBreak = 54;

    // Motion Profile Parameters
    public static final double kMaxAngularAcceleration = 4.5;
    public static final double kMaxAngularVelocity = 3.0;
    
    public static final int kDefaultContinuousCurrentLimit = 35;
    public static final int kDefaultPeakCurrentLimit = 60;

}
