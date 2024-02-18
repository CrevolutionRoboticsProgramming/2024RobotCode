// package frc.robot.Shooter.Commands;

// import java.util.function.DoubleSupplier;
// import java.util.function.Supplier;

// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.estimator.PoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Drivetrain.DrivetrainConfig.DriveConstants;
// import frc.robot.Shooter.Shooter;
// import frc.robot.Shooter.ShooterConfig;
// import frc.robot.Shooter.ShooterIndexer;
// import frc.robot.Shooter.ShooterPivot;

// public class runAllShooter extends Command {
//   private final ShooterPivot m_Pivot;
//   private final Shooter m_Shooter;
//   private final DoubleSupplier supplier;
//   private final ShooterIndexer m_Index;
//   private Supplier<Pose2d> poseEst;

//   private double staringAngle, endAngle;
//   private Long startTs;

//   private TrapezoidProfile profile;
//   private final ArmFeedforward ffController;
//   private final PIDController pidController;

//   public runAllShooter(Shooter shooter, ShooterPivot pivot, ShooterIndexer index, Supplier<Pose2d> poseEst, DoubleSupplier supplier) {
//     this.m_Shooter = shooter;
//     this.m_Pivot = pivot;
//     this.supplier = supplier;
//     this.m_Index = index;
//     this.poseEst = poseEst;

//     ffController = new ArmFeedforward(ShooterConfig.kS, ShooterConfig.kG, ShooterConfig.kV, ShooterConfig.kA);
//     pidController = new PIDController(ShooterConfig.kVelP, ShooterConfig.kVelI, ShooterConfig.kVelD);

//     startTs = null;
    
//     addRequirements(m_Shooter);
//   }

//   @Override
//   public void initialize() {
//     profile = generateProfile();
//     if(m_Index.getBeamBreaker()) {
//       m_Shooter.setShooterPercentOutput(1);
//     }
//     else{
//       m_Shooter.stop();
//     }
//   }

//   @Override
//   public void execute() {
      
//   }
//   @Override
//   public void end(boolean interrupted) {
//     m_Shooter.stop();
//   }

//   private TrapezoidProfile generateProfile() {
//     return new TrapezoidProfile(new TrapezoidProfile.Constraints(DriveConstants.maxAngularVelocity, 720.0), 
//     new TrapezoidProfile.State(Units.degreesToRadians(endAngle),0),
//     new TrapezoidProfile.State(Units.degreesToRadians(startingAngle), DriveConstants.maxAngularVelocity));
//   }
// }
