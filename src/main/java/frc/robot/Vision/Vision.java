package frc.robot.Vision;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import org.opencv.photo.Photo;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Drivetrain.DrivetrainConfig.DriveConstants;
import frc.robot.Vision.VisionConfig.ShooterCamsConfig;

public class Vision {

    //this is how 7028 used PhotonPoseEstimator, so far easiest way i found to use it
    public static class PhotonRunnable implements Runnable{

        private final PhotonCamera photoncamera;
        private final PhotonPoseEstimator photonPoseEstimator;
        private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();

        public PhotonRunnable() {
            //declare photoncamera
            this.photoncamera = ShooterCamsConfig.shooterCam1;
            PhotonPoseEstimator photonEstimator = null;
            try {
                var layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

                layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

                if (photoncamera != null){
                    photonEstimator = new PhotonPoseEstimator(
                        layout, 
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                        photoncamera, 
                        ShooterCamsConfig.robotToCam1);
                }
            } catch(Exception e){
                DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
                photonEstimator = null;
            }
            this.photonPoseEstimator = photonEstimator;
        }

        @Override
        public void run() {
            if (photonPoseEstimator != null && photoncamera != null){
                var photonResults = photoncamera.getLatestResult();
                if (photonResults.hasTargets()){
                    photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
                        var estimatedPose = estimatedRobotPose.estimatedPose;
                        if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= ShooterCamsConfig.fieldLength_m
                        && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= ShooterCamsConfig.fieldWidth_m){
                            atomicEstimatedRobotPose.set(estimatedRobotPose);
                        }
                    });
                }
            }
        }
        public EstimatedRobotPose getLatestEstimatedPose() {
            return atomicEstimatedRobotPose.getAndSet(null);
        }  
    }
    
    public static class PoseEstimator extends SubsystemBase{

        private final Supplier<Rotation2d> rotationSupplier;
        private final Supplier<SwerveModulePosition[]> modSupplier;
        private final SwerveDrivePoseEstimator poseEstimator;
        private final Field2d field2d = new Field2d();
        private final PhotonRunnable photonEstimator = new PhotonRunnable();
        private final Notifier photonNotifier = new Notifier(photonEstimator);
        
        private OriginPosition originPosition = OriginPosition.kBlueAllianceWallRightSide;
        private boolean sawTag = false;

        public PoseEstimator(
            Supplier<Rotation2d> rotationSupplier, Supplier<SwerveModulePosition[]> modSupplier){
                this.rotationSupplier = rotationSupplier;
                this.modSupplier = modSupplier;

                poseEstimator = new SwerveDrivePoseEstimator(
                    DriveConstants.swerveKinematics,
                    rotationSupplier.get(), 
                    modSupplier.get(),
                    new Pose2d());
                
                photonNotifier.setName("PhotonRunnable");
                photonNotifier.startPeriodic(0.02);

                ShuffleboardTab mVisionTab = Shuffleboard.getTab("Vision");
                addDashboardWidgets(mVisionTab);
        }
        
        public void addDashboardWidgets(ShuffleboardTab tab){
            tab.add("Field", field2d).withPosition(0, 0).withSize(6, 4);
            tab.addString("Pose", this::getFormattedPose).withPosition(6, 2).withSize(2, 1);
        }

        public void setAlliance(Alliance alliance){
            boolean allianceChanged = false;
            switch(alliance){
                case Blue:
                    allianceChanged = (originPosition == OriginPosition.kRedAllianceWallRightSide);
                    originPosition = OriginPosition.kBlueAllianceWallRightSide;
                    break;
                case Red:
                    allianceChanged = (originPosition == OriginPosition.kRedAllianceWallRightSide);
                    originPosition = OriginPosition.kBlueAllianceWallRightSide;
                    break;
                default:
                //no default, has to be either blue or red
            }
            if (allianceChanged && sawTag) {
                //alliance determines coordinate system,
                //if it changes the coordinate system also has to change
                var newPose = flipAlliance(getCurrentPose());
                poseEstimator.resetPosition(rotationSupplier.get(), modSupplier.get(), newPose);
            }
        }

        private String getFormattedPose(){
            var pose = getCurrentPose();
            return String.format("(%.3f, %.3f) %.2f degrees", 
            pose.getX(),
            pose.getY(),
            pose.getRotation().getDegrees());
        }

        public Pose2d getCurrentPose(){
            return poseEstimator.getEstimatedPosition();
        }

        public void setCurrentPose(Pose2d newPose){
            poseEstimator.resetPosition(rotationSupplier.get(), modSupplier.get(), newPose);
        }

        public void resetFieldPose(){
            setCurrentPose(new Pose2d());
        }

        private Pose2d flipAlliance(Pose2d poseToFlip){
            return poseToFlip.relativeTo(ShooterCamsConfig.flippingPose);
        }

        @Override
        public void periodic(){
            //Update estimator every 20 ms
            poseEstimator.update(rotationSupplier.get(), modSupplier.get());

            //adding vision measurement
            var visionPose = photonEstimator.getLatestEstimatedPose();
            if (visionPose != null){
                sawTag = true;
                var pose2d = visionPose.estimatedPose.toPose2d();
                if (originPosition != OriginPosition.kBlueAllianceWallRightSide){
                    pose2d = flipAlliance(pose2d);
                }
                poseEstimator.addVisionMeasurement(pose2d, visionPose.timestampSeconds);
            }

            //log to dashboard
            var dashboardPose = poseEstimator.getEstimatedPosition();
            if (originPosition == OriginPosition.kRedAllianceWallRightSide){
                dashboardPose = flipAlliance(dashboardPose);
            }

            SmartDashboard.putString("Estimated Pose", getFormattedPose());
            field2d.setRobotPose(getCurrentPose());
        }

        //methods needed for this class
       
    }
}
