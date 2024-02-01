package frc.robot.Vision;

import java.io.IOException;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Drivetrain.DrivetrainConfig.DriveConstants;
import frc.robot.Vision.VisionConfig.ShooterCamsConfig;

public class Vision {
    
    public class ShooterCams extends SubsystemBase {
        //Define our PhotonCameras
        public static PhotonCamera shooterCam1 = new PhotonCamera(ShooterCamsConfig.shooterCam1Name);
        public static PhotonCamera shooterCam2 = new PhotonCamera(ShooterCamsConfig.shooterCam2Name);
        
        
        // Declare AprilTagFieldLayout outside the try block
        public static AprilTagFieldLayout aprilTagField;

        public ShooterCams() {
            // Initialize AprilTagFieldLayout in the constructor
            try {
                aprilTagField = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            } catch (IOException e) {
                e.printStackTrace();
                // Handle the exception appropriately, e.g., log or throw a runtime exception
                throw new RuntimeException("Failed to load AprilTagFieldLayout", e);
            }
            }
    }
    
    public class PoseEstimator extends SubsystemBase{
    
        private final Supplier<Rotation2d> rotationSupplier;
        private final Supplier<SwerveModulePosition[]> modPoseSupplier;
        private final Field2d field2d = new Field2d();
        private final SwerveDrivePoseEstimator poseEstimator;
        

        // Photon Pose estimator
        PhotonPoseEstimator photonPose1 = new PhotonPoseEstimator(ShooterCams.aprilTagField, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ShooterCams.shooterCam1, ShooterCamsConfig.robotToCam1);

        public PoseEstimator(
            Supplier<Rotation2d> rotationSupplier, Supplier<SwerveModulePosition[]> modPoseSupplier){
                this.rotationSupplier = rotationSupplier;
                this.modPoseSupplier = modPoseSupplier;

            poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.swerveKinematics, 
                rotationSupplier.get(), 
                modPoseSupplier.get(), 
                new Pose2d());
            
        }
        public void addDashboardWidgets(Shuffleboard tab){
            Shuffleboard.getTab("Pose")
            .add("field", field2d)
            .withPosition(0, 0)
            .withSize(6, 4);
        }

        public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
            photonPose1.setReferencePose(prevEstimatedRobotPose);
            return photonPose1.update();
        }
        Pose2d visionRobotPose_M;
        double timeStamp_S;

        @Override
        public void periodic(){
            //update estimated pose
            poseEstimator.update(rotationSupplier.get(), modPoseSupplier.get());

            poseEstimator.addVisionMeasurement(visionRobotPose_M, timeStamp_S);

        }
    }
}
