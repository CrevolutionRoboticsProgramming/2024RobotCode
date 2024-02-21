package frc.robot.Shooter.Commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain.DrivetrainConfig.DriveConstants;
import frc.robot.Intake.Intake;
import frc.robot.Intake.IntakeConfig;
import frc.robot.Intake.IntakePivot;
import frc.robot.Shooter.ShooterConfig;
import frc.robot.Shooter.ShooterIndexer;
import frc.robot.Shooter.ShooterPivot;

public class handOff extends Command{
    private IntakePivot mIntakePivot;
    private ShooterPivot mShooterPivot;
    private ShooterIndexer mIndexer;
    private Intake mIntake;

    private Long startTs;
    private double intakeHandOff, shooterHandOff;

    private TrapezoidProfile intakeProfile, shooterProfile;
    private final ArmFeedforward ffIntakeController, ffShooterController;
    private final PIDController pidIntakeController, pidShooterController;

    public handOff(Intake intake, IntakePivot intakePivot, ShooterPivot shooterPivot, ShooterIndexer shooterIndex){
        mIntake = intake;
        mIntakePivot = intakePivot;
        mShooterPivot = shooterPivot;
        mIndexer = shooterIndex;

        ffIntakeController = new ArmFeedforward(IntakeConfig.kS, IntakeConfig.kG, IntakeConfig.kV, IntakeConfig.kA);
        pidIntakeController = new PIDController(IntakeConfig.kVelP, IntakeConfig.kVelI, IntakeConfig.kVelD);

        ffShooterController = new ArmFeedforward(ShooterConfig.kS, ShooterConfig.kG, ShooterConfig.kV, ShooterConfig.kA);
        pidShooterController = new PIDController(ShooterConfig.kVelP, ShooterConfig.kVelI, ShooterConfig.kVelD);

        startTs = null;

        addRequirements(mIntake);
        addRequirements(mIntakePivot);
        addRequirements(shooterPivot);
        addRequirements(shooterIndex);
    }

    @Override
    public void initialize() {
        intakeHandOff = 120;
        shooterHandOff = 0;
        intakeProfile = generateIntakeProfile();
        shooterProfile = generateShooterProfile();
    }   

    @Override
    public void execute() {
        if (startTs == null) {
            startTs = System.currentTimeMillis();
        }

        final var targetShooterState = shooterProfile.calculate(getElapsedTime());
        final var targetIntakeState = intakeProfile.calculate(getElapsedTime());

        final var ffShooterOutput = ffShooterController.calculate(mShooterPivot.getAngleRads(), targetShooterState.velocity);
        final var pidShooterOutput = pidShooterController.calculate(mShooterPivot.getVelocityRps(), targetShooterState.velocity);

        final var ffIntakeOutput = ffShooterController.calculate(mIntakePivot.getAngleRads(), targetIntakeState.velocity);
        final var pidIntakeOutput = pidShooterController.calculate(mIntakePivot.getVelocityRps(), targetIntakeState.velocity);

        mShooterPivot.setOutput((ffShooterOutput + pidShooterOutput) / 12.0);
        mIntakePivot.setOutput((ffIntakeOutput + pidIntakeOutput) / 12.0);

        if(intakeProfile != null && intakeProfile.isFinished((double) getElapsedTime()) && shooterProfile != null && shooterProfile.isFinished((double) getElapsedTime())){
            mIntake.setIntakeRollerPercentOutput(1);
            mIndexer.setIntakeSpeed();
            if(mIndexer.getBeamBreaker() == false) {
                mIndexer.stopIndexer();
            }
        }
    }

    private double getElapsedTime() {
        return (startTs == null) ? 0 : ((double) System.currentTimeMillis() - startTs) / 1000.0;
    }

    @Override
    public boolean isFinished() {
        final var time = getElapsedTime();
        return intakeProfile != null && intakeProfile.isFinished((double) time) && shooterProfile != null && shooterProfile.isFinished((double) time);
    }

    @Override
    public void end(boolean interrupted) {
        mIntakePivot.stop();
        mShooterPivot.stop();
        mIndexer.stopIndexer();
    }

    private TrapezoidProfile generateIntakeProfile() {
        return new TrapezoidProfile(
            new TrapezoidProfile.Constraints(IntakeConfig.kMaxAngularVelocity, IntakeConfig.kMaxAngularAcceleration),
            new TrapezoidProfile.State(intakeHandOff, 0),
            new TrapezoidProfile.State(0, 0)
        );
    }

    private TrapezoidProfile generateShooterProfile() {
        return new TrapezoidProfile(
            new TrapezoidProfile.Constraints(ShooterConfig.kMaxAngularVelocity, ShooterConfig.kMaxAngularAcceleration),
            new TrapezoidProfile.State(shooterHandOff, 0),
            new TrapezoidProfile.State(0, 0)
        );
    }
}
