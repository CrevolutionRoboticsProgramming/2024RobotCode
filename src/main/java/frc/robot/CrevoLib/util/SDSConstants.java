package frc.robot.CrevoLib.util;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;

public class SDSConstants {
    public final double wheelDiameter;
    public final double wheelCircumference;
    public final double angleGearRatio;
    public final double driveGearRatio;
    public final double angleKP;
    public final double angleKI;
    public final double angleKD;
    public final InvertedValue driveMotorInvert;
    public final InvertedValue angleMotorInvert;
    public final SensorDirectionValue cancoderInvert;
    

    public SDSConstants(double wheelDiameter, double angleGearRatio, double driveGearRatio, 
                            double angleKP, double angleKI, double angleKD, InvertedValue driveMotorInvert, 
                                InvertedValue angleMotorInvert, SensorDirectionValue cancoderInvert){
        this.wheelDiameter = wheelDiameter;
        this.wheelCircumference = wheelDiameter * Math.PI;
        this.angleGearRatio = angleGearRatio;
        this.driveGearRatio = driveGearRatio;
        this.angleKP = angleKP;
        this.angleKI = angleKI;
        this.angleKD = angleKD;
        this.driveMotorInvert = driveMotorInvert;
        this.angleMotorInvert = angleMotorInvert;
        this.cancoderInvert = cancoderInvert;
    }


    public static final class MK4i  {
        /** MK4i Module (Falcon 500)*/
        public static final SDSConstants Falcon500(double driveGearRatio){
            double wheelDiameter = Units.inchesToMeters(4.0);
    
            // (150 / 7) : 1 
            double angleGearRatio = ((150.0 / 7.0) / 1.0);
    
            double angleKP = 100.0;
            double angleKI = 0.0;
            double angleKD = 0.0;
    
            InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;
            InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
            SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;
            return new SDSConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, driveMotorInvert, angleMotorInvert, cancoderInvert);
        }

        public static final class driveRatios{
            /** SDS MK4i - (8.14 : 1) */
            public static final double L1 = (8.14 / 1.0);
            /** SDS MK4i - (6.75 : 1) */
            public static final double L2 = (6.75 / 1.0);
            /** SDS MK4i - (6.12 : 1) */
            public static final double L3 = (6.12 / 1.0);
        }
    }

            
}
