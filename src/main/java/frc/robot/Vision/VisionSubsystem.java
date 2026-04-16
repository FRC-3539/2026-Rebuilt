package frc.robot.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

import java.util.Optional;
import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

public class VisionSubsystem extends Thread {

	// Vision Variables
	AprilTagFieldLayout aprilTagFieldLayout;

	public static PhotonCamera leftCam;
	Transform3d robotToLeftCam = new Transform3d(new Translation3d(-0.2749, .3490, 0.2114),
			new Rotation3d(Math.toRadians(0), Math.toRadians(-30), Math.toRadians(80)));

	public static PhotonCamera rightCam;
	Transform3d robotToRightCam = new Transform3d(new Translation3d(-0.2870, -0.3582, 0.2114),
			new Rotation3d(Math.toRadians(0), Math.toRadians(-30), Math.toRadians(-80)));

	public static PhotonCamera backLeftCam;
	Transform3d robotToBackLeftCam = new Transform3d(new Translation3d(-0.3023, 0.2911, 0.2114),
			new Rotation3d(Math.toRadians(0), Math.toRadians(-30), Math.toRadians(155)));

	public static PhotonCamera backRightCam;
	Transform3d robotToBackRightCam = new Transform3d(new Translation3d(-0.298, -0.2857, 0.2114),
			new Rotation3d(Math.toRadians(0), Math.toRadians(-30), Math.toRadians(-180)));

	PhotonPoseEstimator leftPhotonPoseEstimator;
	PhotonPoseEstimator rightPhotonPoseEstimator;
	PhotonPoseEstimator backLeftPhotonPoseEstimator;
	PhotonPoseEstimator backRightPhotonPoseEstimator;

	Optional<EstimatedRobotPose> resultLeft;
	Optional<EstimatedRobotPose> resultRight;
	Optional<EstimatedRobotPose> resultBackLeft;
	Optional<EstimatedRobotPose> resultBackRight;
	boolean useVision = true;

	double leftLastTimeStamp = 0;
	double rightLastTimeStamp = 0;
	double backLeftLastTimeStamp = 0;
	double backRightLastTimeStamp = 0;

	double visionRatio = 10;

	StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
			.getStructTopic("/Vision/BackRightPose3d", Pose3d.struct).publish();

	public VisionSubsystem() {
		super();
		try {
			aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

		} catch (Exception e) {
			System.out.println("ERROR Loading April Tag DATA");
			aprilTagFieldLayout = null;
		}

		leftCam = new PhotonCamera("leftCam");
		leftPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToLeftCam);

		rightCam = new PhotonCamera("rightCam");
		rightPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToRightCam);

		backLeftCam = new PhotonCamera("backLeftCam");
		backLeftPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToBackLeftCam);

		backRightCam = new PhotonCamera("backRightCam");
		backRightPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToBackRightCam);

		setVisionWeights(.2, .2, 10);
	}

	// Vision Methods

	public Optional<EstimatedRobotPose> getEstimatedLeftGlobalPose() {
		Optional<EstimatedRobotPose> visEst = Optional.empty();
		for (var change : leftCam.getAllUnreadResults()) {
			visEst = leftPhotonPoseEstimator.update(change);
		}
		return visEst;
	}

	public Optional<EstimatedRobotPose> getEstimatedRightGlobalPose() {
		Optional<EstimatedRobotPose> visEst = Optional.empty();
		for (var change : rightCam.getAllUnreadResults()) {
			visEst = rightPhotonPoseEstimator.update(change);
		}
		return visEst;
	}

	public Optional<EstimatedRobotPose> getEstimatedBackLeftGlobalPose() {
		Optional<EstimatedRobotPose> visEst = Optional.empty();
		for (var change : backLeftCam.getAllUnreadResults()) {
			visEst = backLeftPhotonPoseEstimator.update(change);
		}
		return visEst;
	}

	public Optional<EstimatedRobotPose> getEstimatedBackRightGlobalPose() {
		Optional<EstimatedRobotPose> visEst = Optional.empty();
		for (var change : backRightCam.getAllUnreadResults()) {
			visEst = backRightPhotonPoseEstimator.update(change);
		}
		return visEst;
	}

	public void useVision(boolean useVision) {
		this.useVision = useVision;
	}

	public void setVisionWeights(double visionX, double visionY, double visionDeg) {
		// RobotContainer.driveSubsystem
		// .setVisionMeasurementStdDevs(VecBuilder.fill(visionX, visionY,
		// Units.degreesToRadians(visionDeg)));
	}

	public static void publishPose2d(String key, Pose2d pose) {
		SmartDashboard.putNumberArray(key, new double[] { pose.getTranslation().getX(), pose.getTranslation().getY(),
				pose.getRotation().getRadians() });
	}

	Alliance lastAlliance = null;

	public void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> weights) {
		// RobotContainer.driveSubsystem.addVisionMeasurement(pose, timestampSeconds,
		// weights);
	}

	public void log() {
		SmartDashboard.putBoolean("/Vision/Left/Connected", leftCam.isConnected());
		SmartDashboard.putBoolean("/Vision/Right/Connected", rightCam.isConnected());
		SmartDashboard.putBoolean("/Vision/BackLeft/Connected", backLeftCam.isConnected());
		SmartDashboard.putBoolean("/Vision/BackRight/Connected", backRightCam.isConnected());
	}

	public Matrix<N3, N1> getVisionWeights(double distanceRatio, int numTargets) {
		double targetMultiplier = 0.75;
		double visionCutOffDistance = 4;
		distanceRatio = 0.1466 * Math.pow(1.6903, distanceRatio);
		if (numTargets == 1) {
			if (distanceRatio > visionCutOffDistance) {
				return new Matrix<N3, N1>(new SimpleMatrix(new double[] { 99999, 99999, 99999 }));
			}
			targetMultiplier = 3;
		}
		Matrix<N3, N1> weights = new Matrix<N3, N1>(new SimpleMatrix(new double[] { distanceRatio * targetMultiplier,
				distanceRatio * targetMultiplier, 3 + 15 * distanceRatio * targetMultiplier }));
		return weights;
	}

	public Matrix<N3, N1> evaluateMeasurement(Optional<EstimatedRobotPose> result) {
		EstimatedRobotPose camPose = result.get();
		double camTimeStamp = camPose.timestampSeconds;
		if (camTimeStamp > Timer.getFPGATimestamp()) {
			camTimeStamp = Timer.getFPGATimestamp();
		}

		double sum = 0;
		for (PhotonTrackedTarget target : camPose.targetsUsed) {
			Translation2d tagPosition = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get()
					.getTranslation().toTranslation2d();
			sum += result.get().estimatedPose.toPose2d().getTranslation().getDistance(tagPosition);
		}
		sum /= camPose.targetsUsed.size();
		double distanceRatio = sum;
		return getVisionWeights(distanceRatio, camPose.targetsUsed.size());
	}

	@Override
	public void run() {
		/* Run as fast as possible, our signals will control the timing */

		try{
		while (true) {
			// currentTimeStamp = Robotcontroller.
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			// Vision Calculations
			if (DriverStation.getAlliance().isPresent()) {
				if (DriverStation.getAlliance().get() == Alliance.Blue && lastAlliance != Alliance.Blue) {
					lastAlliance = Alliance.Blue;
					aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
				}
				if (DriverStation.getAlliance().get() == Alliance.Red && lastAlliance != Alliance.Red) {
					lastAlliance = Alliance.Red;
					aprilTagFieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
				}
			}

			leftPhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);
			rightPhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);
			backLeftPhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);
			backRightPhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);

			this.resultLeft = getEstimatedLeftGlobalPose();
			this.resultRight = getEstimatedRightGlobalPose();
			this.resultBackLeft = getEstimatedBackLeftGlobalPose();
			this.resultBackRight = getEstimatedBackRightGlobalPose();

			if (useVision) {
				// Left camera
				if (resultLeft.isPresent()) {
					// System.out.println("left camera has something!");
					EstimatedRobotPose camPose = resultLeft.get();
					double timeStamp = camPose.timestampSeconds;
					Matrix<N3, N1> weights = evaluateMeasurement(resultLeft);

					
					// System.out.println("current timestamp: " + timeStamp + " last timestamp: " + leftLastTimeStamp);
					if (timeStamp != leftLastTimeStamp) {
						publishPose2d("/Vision/LeftCamPose", camPose.estimatedPose.toPose2d());
						SmartDashboard.putString("/Vision/LeftWeights", weights.toString());

						RobotContainer.driveSubsystem.addVisionMeasurement(
								camPose.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(timeStamp), weights);
					}
					leftLastTimeStamp = timeStamp;
				}

				// Right camera
				if (resultRight.isPresent()) {
					EstimatedRobotPose camPose = resultRight.get();
					double timeStamp = camPose.timestampSeconds;
					Matrix<N3, N1> weights = evaluateMeasurement(resultRight);

					if (timeStamp != rightLastTimeStamp) {
						publishPose2d("/Vision/RightCamPose", camPose.estimatedPose.toPose2d());
						SmartDashboard.putString("/Vision/RightWeights", weights.toString());

						RobotContainer.driveSubsystem.addVisionMeasurement(
								camPose.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(timeStamp), weights);
					}
					rightLastTimeStamp = timeStamp;
				}

				// Back left camera
				if (resultBackLeft.isPresent()) {
					EstimatedRobotPose camPose = resultBackLeft.get();
					double timeStamp = camPose.timestampSeconds;
					Matrix<N3, N1> weights = evaluateMeasurement(resultBackLeft);

					if (timeStamp != backLeftLastTimeStamp) {
						publishPose2d("/Vision/BackLeftCamPose", camPose.estimatedPose.toPose2d());
						SmartDashboard.putString("/Vision/BackLeftWeights", weights.toString());

						RobotContainer.driveSubsystem.addVisionMeasurement(
								camPose.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(timeStamp), weights);
					}
					backLeftLastTimeStamp = timeStamp;
				}

				// Back right camera
				if (resultBackRight.isPresent()) {
					EstimatedRobotPose camPose = resultBackRight.get();
					double timeStamp = camPose.timestampSeconds;
					Matrix<N3, N1> weights = evaluateMeasurement(resultBackRight);

					if (timeStamp != backRightLastTimeStamp) {
						publishPose2d("/Vision/BackRightCamPose", camPose.estimatedPose.toPose2d());
						SmartDashboard.putString("/Vision/BackRightWeights", weights.toString());
						publisher.set(camPose.estimatedPose);

						RobotContainer.driveSubsystem.addVisionMeasurement(
								camPose.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(timeStamp), weights);
					}
					backRightLastTimeStamp = timeStamp;
				}
			}

			log();
		}
	}
	catch (Exception e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
}