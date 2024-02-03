package frc.robot.Shooter;

public class ShooterConfig {

    /* Declarations */

    private final XboxController mController = new XboxController(0);
    public TalonFX mTopLeftShooter;
    public TalonFX mTopRightShooter;
    public TalonFX mBottomLeftShooter;
    public TalonFX mBottomRightShooter;

     /* Shooter Inverts and Neutral Mode */
     public static final InvertedValue SHOOTER_INVERTED = InvertedValue.Clockwise_Positive;
     public static final NeutralModeValue SHOOTER_NEUTRAL_MODE = NeutralModeValue.Coast;
 
     /* Shooter Current Limits */
     public static final boolean SHOOTER_ENABLE_CURRENT_LIMIT = true;
     public static final int SHOOTER_SUPPLY_CURRENT_LIMIT = 40;
     public static final int SHOOTER_SUPPLY_CURRENT_THRESHOLD = 60;
     public static final double SHOOTER_SUPPLY_TIME_THRESHOLD = 0.1;
 
     /* Shooter PID Constants */
     public static final double SHOOTER_P = 0.01;
     public static final double SHOOTER_I = 0;
     public static final double SHOOTER_D = 0;
     public static final double SHOOTER_V = 0.01;
     public static final double SHOOTER_A = 0;
 
     /* Shooter Velocities - Rotations per Second */
     public static final double SHOOTER_SHOOT;
 
}
