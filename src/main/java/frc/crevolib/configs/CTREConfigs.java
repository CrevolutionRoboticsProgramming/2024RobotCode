package frc.crevolib.configs;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.drivetrain.DrivetrainConfig.DriveConstants;

public final class CTREConfigs {
    public TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();
    public TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
    public CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    public TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

    public CTREConfigs() {
        /*Cancoder Sensor Direction*/
        cancoderConfig.MagnetSensor.SensorDirection = DriveConstants.cancoderInvert;

        // Angle Motor Configuration
        angleMotorConfig.MotorOutput.Inverted = DriveConstants.angleMotorInvert;
        angleMotorConfig.MotorOutput.NeutralMode = DriveConstants.angleNeutralMode;

        angleMotorConfig.Feedback.SensorToMechanismRatio = DriveConstants.angleGearRatio;
        angleMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;

        //Angle Motor Current Limits
        angleMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = DriveConstants.angleEnableCurrentLimit;
        angleMotorConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.angleCurrentLimit;
        angleMotorConfig.CurrentLimits.SupplyCurrentThreshold = DriveConstants.angleCurrentThreshold;
        angleMotorConfig.CurrentLimits.SupplyTimeThreshold = DriveConstants.angleCurrentThresholdTime;

        //Angle Motor PID Config
        angleMotorConfig.Slot0.kP = DriveConstants.angleKP;
        angleMotorConfig.Slot0.kI = DriveConstants.angleKI;
        angleMotorConfig.Slot0.kD = DriveConstants.angleKD;

        //Drive Motor Configuration
        driveMotorConfig.MotorOutput.Inverted = DriveConstants.driveMotorInvert;
        driveMotorConfig.MotorOutput.NeutralMode = DriveConstants.driveNeutralMode;

        driveMotorConfig.Feedback.SensorToMechanismRatio = DriveConstants.driveGearRatio;

        //Drive Motor Current Limits
        driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = DriveConstants.driveEnableCurrentLimit;
        driveMotorConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.driveCurrentLimit;
        driveMotorConfig.CurrentLimits.SupplyCurrentThreshold = DriveConstants.driveCurrentThreshold;
        driveMotorConfig.CurrentLimits.SupplyTimeThreshold = DriveConstants.driveCurrentThresholdTime;

        //Drive Motor PID
        driveMotorConfig.Slot0.kP = DriveConstants.driveKP;
        driveMotorConfig.Slot0.kI = DriveConstants.driveKI;
        driveMotorConfig.Slot0.kD = DriveConstants.driveKD;

        //Open-Closed Loop Ramps (DO NOT CHANGE)
        driveMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = DriveConstants.openLoopRamp;
        driveMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = DriveConstants.openLoopRamp;

        driveMotorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = DriveConstants.closedLoopRamp;
        driveMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = DriveConstants.closedLoopRamp;
    }
}
