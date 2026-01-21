package frc.robot.utils;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public class Utils {
    /**
     * 
     * @param diameter
     * @param ss = surface speed per second
     * @return
     */
    public static AngularVelocity ssToAngularVelocity(double diameter, double ss)
    {
        double cir = diameter * Math.PI;
        return RotationsPerSecond.of(ss/cir);
    }
    
}