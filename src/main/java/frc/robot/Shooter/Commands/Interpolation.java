// package frc.robot.Shooter.Commands;

// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Shooter.ShooterInterpolation;
// import frc.robot.Shooter.ShooterPivot;
// import frc.robot.Shooter.ShooterConfig;
// import frc.robot.Intake.IntakeConfig;
// import frc.robot.Shooter.Shooter;

// public class Interpolation extends Command {
//   private final ShooterPivot pivot;
//   private final Shooter shooter;

//   private TrapezoidProfile profile;
//   private final ArmFeedforward ffController;
//   private final PIDController pidController;

//   private final double distance;

//   private Long startTs;

//   public Interpolation(ShooterPivot pivot, Shooter shooter, double distance) {
//     this.pivot = pivot;
//     this.shooter = shooter;
//     this.distance = distance;

//     ffController = new ArmFeedforward(ShooterConfig.kS, ShooterConfig.kG, ShooterConfig.kV, ShooterConfig.kA);
//     pidController = new PIDController(ShooterConfig.kVelP, ShooterConfig.kVelI, ShooterConfig.kVelD);

//     startTs = null;
    
//   }

//   @Override
//   public void initialize() {
//     profile = new TrapezoidProfile();

//     startTs = null;
                   
//   }

  
//   @Override
//   public void execute() {
    
//   }

//   @Override
//   public void end(boolean interrupted) {
    
//   }


//   @Override
//   public boolean isFinished() {
//     return true;
//   }
// }

