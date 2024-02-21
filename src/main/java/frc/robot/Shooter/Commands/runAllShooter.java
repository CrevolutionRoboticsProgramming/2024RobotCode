package frc.robot.Shooter.Commands;

import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Vision.Vision.PoseEstimator;
import frc.robot.Vision.VisionConfig.ShooterCamsConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain.DrivetrainConfig.DriveConstants;
import frc.robot.Shooter.Shooter;
import frc.robot.Shooter.ShooterConfig;
import frc.robot.Shooter.ShooterIndexer;
import frc.robot.Shooter.ShooterInterpolation;
import frc.robot.Shooter.ShooterPivot;

public class runAllShooter extends Command {
  private final ShooterPivot mPivot;
  private final Shooter mShooter;
  private final ShooterIndexer mIndex;
  private final PoseEstimator mPoseEstimator;
  private final ShooterInterpolation mInterpolate;

  private Pose2d robotPose2d;
  private PhotonCamera shooterCam1;
  private PhotonTrackedTarget lastTarget;
  private Pose3d targetPose;
  private boolean safeArea;
  private double startingAngle, mVelocity, mAngle;
  private Long startTs;

  private TrapezoidProfile profile;
  private final ArmFeedforward ffController;
  private final PIDController pidController;

  public runAllShooter(Shooter shooter, ShooterPivot pivot, ShooterIndexer index, PoseEstimator poseEst, ShooterInterpolation interpolate) {
    this.mShooter = shooter;
    this.mPivot = pivot;
    this.mIndex = index;
    this.mPoseEstimator = poseEst;
    this.mInterpolate = interpolate;
    shooterCam1 = ShooterCamsConfig.shooterCam1;

    ffController = new ArmFeedforward(ShooterConfig.kS, ShooterConfig.kG, ShooterConfig.kV, ShooterConfig.kA);
    pidController = new PIDController(ShooterConfig.kVelP, ShooterConfig.kVelI, ShooterConfig.kVelD);

    startTs = null;
    
    addRequirements(mShooter);
  }

  @Override
  public void initialize() {
    robotPose2d = mPoseEstimator.getCurrentPose();
    var robotPose = new Pose3d(
        robotPose2d.getX(), 
        robotPose2d.getY(),
        0.0, new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));
    //get last camera result
    var result = shooterCam1.getLatestResult();
    if (result.hasTargets()) {
            //verify target id
            var optionalTarget = result.getTargets().stream()
            .filter(t -> checkID(ShooterCamsConfig.targetList, t.getFiducialId()))
            .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
            .findFirst();
            if (optionalTarget.isPresent()){
                var target = optionalTarget.get();
                lastTarget = target;

                //get camera pose
                var cameraPose = robotPose.transformBy(ShooterCamsConfig.robotToCam1);

                //get target pose
                var camToTarget = target.getBestCameraToTarget();
                targetPose = cameraPose.transformBy(camToTarget);
            }
        }

    var xLength = targetPose.getX();
    var yLength = targetPose.getY();
    var actualDist = Math.sqrt((xLength * xLength) + (yLength * yLength));
    startingAngle = mPivot.getAngleRads();
    var interpolatedAngle = Units.degreesToRadians(mInterpolate.getInterpolatedAngle(actualDist));
    mAngle = interpolatedAngle - startingAngle;
    mVelocity = mInterpolate.getInterpolatedPercentOutput(actualDist);

    profile = generateProfile();

    var currentAlliance = DriverStation.getAlliance();
    if(currentAlliance.equals(DriverStation.Alliance.Blue)) {
        safeArea = (robotPose2d.getX() <= 6);
    } else {
        safeArea = (robotPose2d.getX() >= 10);
    }
    if(mIndex.getBeamBreaker() || safeArea == true) {
      mShooter.setShooterPercentOutput(1);
    }
    else{
      mShooter.stop();
    }
  }

  @Override
  public void execute() {
    final var duration = getElapsedTime();
    if (startTs == null) {
        startTs = System.currentTimeMillis();
    }

    final var targetState = profile.calculate(duration);

    final var ffOutput = ffController.calculate(mPivot.getAngleRads(), targetState.velocity);
    final var pidOutput = pidController.calculate(mPivot.getVelocityRps(), targetState.velocity);

    mPivot.setOutput((ffOutput + pidOutput) / 12.0);

    if(profile != null && profile.isFinished((double) duration)){
        mShooter.setShooterVeclocity(mVelocity);
        if(mIndex.getBeamBreaker()) {
            mIndex.setIntakeSpeed();
        }
        if(mIndex.getBeamBreaker() == false) {
            mIndex.stopIndexer();
            mShooter.stop();
        }
    }
  }

  @Override
  public boolean isFinished() {
    final var time = getElapsedTime();
    return profile != null && profile.isFinished((double) time) && mIndex.getBeamBreaker() == false;
  }

  @Override
  public void end(boolean interrupted) {
    mShooter.stop();
    mIndex.stopIndexer();
    mPivot.setState(null);
  }
  
  private double getElapsedTime() {
    return (startTs == null) ? 0 : ((double) System.currentTimeMillis() - startTs) / 1000.0;
  }
    
  private static boolean checkID(Integer[] arr, int fiducialID){
    boolean acceptedID = Arrays.asList(arr).contains(fiducialID);
    return acceptedID;
  }

  private TrapezoidProfile generateProfile() {
    return new TrapezoidProfile(new TrapezoidProfile.Constraints(ShooterConfig.kMaxAngularVelocity, ShooterConfig.kMaxAngularAcceleration), 
    new TrapezoidProfile.State(Units.degreesToRadians(mAngle),0),
    new TrapezoidProfile.State(Units.degreesToRadians(startingAngle), 0));
  }
}
