package frc.robot.Constants;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AlignConstants {

    public static enum AlignPoint{}
   
    public static Set< Pose2d> trenchPoints = new HashSet<Pose2d>();
    public static Set< Pose2d> towerPoints = new HashSet<Pose2d>();
    private final static double INCHES_TO_METERS = 0.0254;

    public AlignConstants() {
        // Manual points

        // Algae points
        trenchPoints.add(new Pose2d( 3.2, 7.4, new Rotation2d(0)));
        trenchPoints.add(new Pose2d( 6.2, 7.4, new Rotation2d(0)));
        trenchPoints.add(new Pose2d( 3.2, .67, new Rotation2d(0)));
        trenchPoints.add(new Pose2d( 6.2, .67, new Rotation2d(0)));
        trenchPoints.add(new Pose2d( 13.34, 7.4, new Rotation2d(0)));
        trenchPoints.add(new Pose2d( 10.34, 7.4, new Rotation2d(0)));
        trenchPoints.add(new Pose2d( 13.34, 0.67, new Rotation2d(0)));
        trenchPoints.add(new Pose2d( 10.34, 0.67, new Rotation2d(0)));

        // algaePoints.put(AlignPoint.A, getOffsetPoint(3.6576, 4.0269, 0, 0, ALIGN_DISTANCE));
    }
}

// points.put(AlignMode.A, new Pose2d(3.10, 4.19, Rotation2d.fromDegrees(0)));
// points.put(AlignMode.B, new Pose2d(3.10, 3.86, Rotation2d.fromDegrees(0)));
// points.put(AlignMode.C, new Pose2d(3.65, 2.90, Rotation2d.fromDegrees(60)));
// points.put(AlignMode.D, new Pose2d(3.93, 2.74, Rotation2d.fromDegrees(60)));
// points.put(AlignMode.E, new Pose2d(5.04, 2.74, Rotation2d.fromDegrees(120)));
// points.put(AlignMode.F, new Pose2d(5.33, 2.90, Rotation2d.fromDegrees(120)));
// points.put(AlignMode.G, new Pose2d(5.88, 3.86, Rotation2d.fromDegrees(180)));
// points.put(AlignMode.H, new Pose2d(5.88, 4.19, Rotation2d.fromDegrees(180)));
// points.put(AlignMode.I, new Pose2d(5.33, 5.15,
// Rotation2d.fromDegrees(-120)));
// points.put(AlignMode.J, new Pose2d(5.04, 5.31,
// Rotation2d.fromDegrees(-120)));
// points.put(AlignMode.K, new Pose2d(3.93, 5.31, Rotation2d.fromDegrees(-60)));
// points.put(AlignMode.L, new Pose2d(3.65, 5.15, Rotation2d.fromDegrees(-60)));
// points.put(AlignMode.HUMANPLAYER1, new Pose2d(1.75, 1.75,
// Rotation2d.fromDegrees(50)));
// points.put(AlignMode.HUMANPLAYER2, new Pose2d(1.5, 6.75,
// Rotation2d.fromDegrees(-50)));
// points.put(AlignMode.CLIMB1, new Pose2d(8.5, 5.25,
// Rotation2d.fromDegrees(-90)));
// points.put(AlignMode.CLIMB2, new Pose2d(8.5, 5.25,
// Rotation2d.fromDegrees(-90)));
// points.put(AlignMode.CLIMB3, new Pose2d(8.5, 5.25,
// Rotation2d.fromDegrees(-90)));