package frc.robot.Autos;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.util.Units;

public class AutonConfig {
    
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(5,0.1,0.25);
    public static final PIDConstants ROTATION_PID = new PIDConstants(2.5,0,0);

    //TODO: change the Drive Base Radius and Max speed
    // Drive base radius in meters. Distance from robot center to furthest module.
    // This is set for Theseus (assuming 26.25 x 26.25 frame, use the actual dimensions from 2024 robot here)
    // Calculate based on a^2 + b^2 = c^2
    public static final double MAX_AUTON_MODULE_SPEED = 4.5; // m/s
    public static final double DRIVE_BASE_RADIUS = Units.inchesToMeters(344.53125); 
}
