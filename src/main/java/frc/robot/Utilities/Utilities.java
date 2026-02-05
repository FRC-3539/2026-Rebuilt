package frc.robot.Utilities;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Utilities {
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

    public static void publishPose2d(String key, Pose2d pose) {
		SmartDashboard.putNumberArray(key, new double[]{pose.getTranslation().getX(), pose.getTranslation().getY(),
				pose.getRotation().getRadians()});
	}

    public static void publishTranslation2d(String key, Translation2d translation) {
		SmartDashboard.putNumberArray(key, new double[]{translation.getX(), translation.getY()});
    }
}