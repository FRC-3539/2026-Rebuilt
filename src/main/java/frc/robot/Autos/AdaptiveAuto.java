// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import java.util.Optional;
import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Intake.IntakeBallCommand;
import frc.robot.Intake.SetIntakeCommand;
import frc.robot.Shooter.RevCommand;
import frc.robot.Shooter.ShootCommand;

/**
 * First pass at an adaptive (non-replayed) autonomous. Pattern and structure come from
 * docs/adaptive-auto-brief.md -- read that first. Everything here is PathPlanner-native
 * (Option A: inline command composition), matching how every other auto in this repo is built.
 *
 * <p>Two things below are placeholders, flagged where they occur: the field poses (borrowed from
 * existing .path files, not independently verified) and the game-piece sensing (no such sensor
 * exists anywhere in this codebase yet, so those checks are safe stubs, not real verification).
 */
public class AdaptiveAuto {
  private AdaptiveAuto() {
  }

  // ================= TUNE ME (starting guesses; verify on the real robot) =================
  private static final double LEG_TIMEOUT_S = 5.0; // hard cap per drive leg, including recovery
  private static final double REAPPROACH_TIMEOUT_S = 2.0; // short corrective hop before each score attempt
  private static final double SCORE_DURATION_S = 1.5; // matches existing autos' rev+shoot dwell
  private static final double SCORE_SETTLE_S = 0.3; // pause before reading pieceStillPresent()
  private static final double INTAKE_TIMEOUT_S = 3.0; // step 3 bounded wait for a piece
  private static final int MAX_SCORE_ATTEMPTS = 3;

  private static final double STUCK_SPEED_MPS = 0.15; // measured speed below this counts as "not moving"
  private static final double STUCK_DEBOUNCE_S = 0.35;
  private static final double STUCK_GRACE_S = 0.5; // ignore the accel-from-rest window at leg start
  private static final double NUDGE_SPEED_MPS = 1.0; // backward nudge to break contact before re-pathing
  private static final double NUDGE_S = 0.3;

  // Matches src/main/deploy/pathplanner/settings.json's defaultMaxVel/Accel/AngVel/AngAccel.
  private static final PathConstraints CONSTRAINTS =
      new PathConstraints(3.0, 3.0, Math.toRadians(540), Math.toRadians(720));

  // ============ SET ME (placeholders borrowed from existing paths -- verify on the field) ============
  // Shared endpoint of "Outpost to Tower.path" (its start) and "Right Anti Outpost.path" (its
  // end) -- both already agree this is a real, previously-driven pickup spot.
  private static final Pose2d POSE_HP = new Pose2d(0.600, 0.640, Rotation2d.fromDegrees(180));
  // End of "Outpost to Tower.path" -- a real, previously-driven shooting spot.
  private static final Pose2d POSE_A = new Pose2d(1.045, 2.900, Rotation2d.fromDegrees(-90));
  // POSE_A mirrored across the field's shooting midline (y ~= Hub's y, 4.0345). Geometrically
  // sound but nobody has actually driven this exact spot -- treat as a guess, not a fact.
  private static final Pose2d POSE_B = new Pose2d(1.045, 5.169, Rotation2d.fromDegrees(90));

  // Optional fixed .path segments for the known-to-known legs (brief step 2/4/5). If a file with
  // this name doesn't exist yet under src/main/deploy/pathplanner/paths/, that leg just falls
  // back to live pathfinding -- draw these in the PathPlanner UI later for a faster/smoother leg.
  private static final String PATH_A_TO_HP = "A To HP";
  private static final String PATH_HP_TO_B = "HP To B";
  private static final String PATH_B_TO_HP = "B To HP";

  /** Top-level routine: preload at A, cycle through HP, score at B with verify+retry, return to HP. */
  public static Command adaptiveAuto() {
    return Commands.sequence(
        Commands.runOnce(() -> {
          SmartDashboard.putBoolean("/AdaptiveAuto/SensorsStubbed", true);
          logEvent("start");
        }),
        driveLeg(null, POSE_A), // step 1: unknown start -> A, pathfind only (no fixed path is valid here)
        score().withTimeout(SCORE_DURATION_S), // step 1: score the preload
        driveLeg(PATH_A_TO_HP, POSE_HP), // step 2: A -> HP, stuck-aware
        waitForPiece(), // step 3: bounded wait for the intake sensor
        driveLeg(PATH_HP_TO_B, POSE_B), // step 4: HP -> B, stuck-aware
        scoreVerifyRetry(MAX_SCORE_ATTEMPTS), // step 4/5: score, verify by sensor, retry up to the cap
        driveLeg(PATH_B_TO_HP, POSE_HP), // step 5: only reached once scored or attempts are exhausted
        Commands.runOnce(() -> logEvent("complete")));
  }

  /**
   * Drives one leg to {@code goal}, preferring the fixed path named {@code pathName} (null skips
   * straight to pathfinding, which is required for step 1 since the start pose is unknown). Ends
   * early and recovers if {@link StuckMonitor} fires. Always bounded by {@link #LEG_TIMEOUT_S}.
   */
  private static Command driveLeg(String pathName, Pose2d goal) {
    return Commands.defer(() -> {
      StuckMonitor stuck = new StuckMonitor();
      Optional<PathPlannerPath> fixedPath = pathName == null ? Optional.empty() : loadPath(pathName);
      Command follow = fixedPath
          .map(AutoBuilder::followPath)
          .orElseGet(() -> AutoBuilder.pathfindToPose(goal, CONSTRAINTS));

      return Commands.race(follow, Commands.waitUntil(stuck::isStuck))
          .andThen(Commands.either(
              recover(goal),
              Commands.none(),
              stuck::isStuck));
    }, Set.of(RobotContainer.driveSubsystem)).withTimeout(LEG_TIMEOUT_S);
  }

  /** Runs the flywheel and feeds it. Never finishes on its own -- callers must bound it. */
  private static Command score() {
    return Commands.parallel(new RevCommand(), new ShootCommand());
  }

  /** Deploys + spins the intake until {@link #hasGamePiece()} trips or {@link #INTAKE_TIMEOUT_S} elapses. */
  private static Command waitForPiece() {
    Command bounded = Commands.waitUntil(AdaptiveAuto::hasGamePiece).withTimeout(INTAKE_TIMEOUT_S);
    Command running = Commands.sequence(
        new SetIntakeCommand(IntakeConstants.IntakeDeploySoftMax),
        new IntakeBallCommand());
    return Commands.deadline(bounded, running)
        .andThen(Commands.runOnce(
            () -> logEvent(hasGamePiece() ? "piece acquired" : "intake wait timed out, continuing")));
  }

  /**
   * Re-approaches B, scores, and checks {@link #pieceStillPresent()}. If the piece is still there
   * and attempts remain, recurses via {@link Commands#defer} so the retry is built lazily and
   * re-reads the sensor each pass, per the brief. Bottoms out gracefully at {@code attemptsLeft == 0}.
   */
  private static Command scoreVerifyRetry(int attemptsLeft) {
    return Commands.defer(() -> Commands.sequence(
        AutoBuilder.pathfindToPose(POSE_B, CONSTRAINTS).withTimeout(REAPPROACH_TIMEOUT_S),
        score().withTimeout(SCORE_DURATION_S),
        Commands.waitSeconds(SCORE_SETTLE_S),
        Commands.either(
            Commands.runOnce(() -> logEvent("score attempt " + attemptNumber(attemptsLeft) + " verified clear")),
            attemptsLeft > 1
                ? scoreVerifyRetry(attemptsLeft - 1)
                : Commands.runOnce(() -> logEvent("score attempts exhausted, giving up on B")),
            () -> !pieceStillPresent())),
        Set.of(RobotContainer.driveSubsystem, RobotContainer.hopperSubsystem, RobotContainer.shooterSubsystem));
  }

  /** Backward nudge to break contact, then a fresh pathfind to {@code goal} from wherever we ended up. */
  private static Command recover(Pose2d goal) {
    return Commands.sequence(
        Commands.runOnce(() -> logEvent("stuck -- recovering toward " + poseToString(goal))),
        nudgeBackward(),
        AutoBuilder.pathfindToPose(goal, CONSTRAINTS));
  }

  private static Command nudgeBackward() {
    return Commands.runOnce(
            () -> RobotContainer.driveSubsystem.applyRequest(
                new SwerveRequest.RobotCentric().withVelocityX(-NUDGE_SPEED_MPS)),
            RobotContainer.driveSubsystem)
        .andThen(Commands.waitSeconds(NUDGE_S))
        .finallyDo(() -> RobotContainer.driveSubsystem.applyRequest(new SwerveRequest.Idle()));
  }

  /** PathPlannerPath.fromPathFile's checked exceptions collapsed to "use pathfinding instead." */
  private static Optional<PathPlannerPath> loadPath(String name) {
    try {
      return Optional.of(PathPlannerPath.fromPathFile(name));
    } catch (Exception e) {
      return Optional.empty();
    }
  }

  private static void logEvent(String message) {
    SmartDashboard.putString("/AdaptiveAuto/LastEvent", message);
    DriverStation.reportWarning("[AdaptiveAuto] " + message, false);
  }

  private static String poseToString(Pose2d pose) {
    return String.format("(%.2f, %.2f, %.0f deg)", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }

  private static int attemptNumber(int attemptsLeft) {
    return MAX_SCORE_ATTEMPTS - attemptsLeft + 1;
  }

  // ================= NEEDS REAL HARDWARE =================
  // No game-piece sensor exists anywhere in this codebase today (checked Intake/Hopper/Shooter
  // subsystems for DigitalInput/CANrange/beam-break/etc. -- there is none). These two methods are
  // the sensor-gated checks steps 3 and 5 of the brief depend on. Until a real sensor exists:
  // hasGamePiece() always reports false, so step 3 always waits out INTAKE_TIMEOUT_S then
  // continues anyway; pieceStillPresent() always reports false too, i.e. "assume scored," so
  // scoreVerifyRetry() never actually retries -- it scores once and moves on, which matches what
  // the existing PathPlanner autos already do today. Both defaults are bounded and safe, but
  // neither one is really verifying anything yet -- wire these to a real sensor (a beam break in
  // the hopper is the usual choice) to make steps 3 and 5 do what they're for.
  private static boolean hasGamePiece() {
    return false;
  }

  private static boolean pieceStillPresent() {
    return false;
  }

  /** Debounced "commanding motion but the pose isn't actually moving," per the brief's §5.5 detector. */
  private static class StuckMonitor {
    private final Debouncer debouncer = new Debouncer(STUCK_DEBOUNCE_S, Debouncer.DebounceType.kRising);
    private final Timer legTimer = new Timer();

    StuckMonitor() {
      legTimer.restart();
    }

    boolean isStuck() {
      double speed = Math.hypot(RobotContainer.driveSubsystem.velocityX, RobotContainer.driveSubsystem.velocityY);
      boolean creeping = legTimer.hasElapsed(STUCK_GRACE_S) && speed < STUCK_SPEED_MPS;
      return debouncer.calculate(creeping);
    }
  }
}
