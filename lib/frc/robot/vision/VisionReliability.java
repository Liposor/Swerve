package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.config.ConfigVision;

/** Aplica gates geometricos e calcula a confianca de cada frame MegaTag2. */
public final class VisionReliability {
  private VisionReliability() {}

  public enum RejectionReason {
    ACCEPTED,
    NO_TAGS,
    UNKNOWN_TAG,
    STALE_TIMESTAMP,
    FUTURE_TIMESTAMP,
    INVALID_HEIGHT,
    INVALID_ROLL_PITCH,
    OUTSIDE_FIELD,
    DISTANCE_TOO_SMALL,
    DISTANCE_TOO_LARGE,
    HIGH_AMBIGUITY,
    HIGH_ANGULAR_VELOCITY,
    LARGE_POSE_INNOVATION,
    NON_FINITE_VALUE
  }

  public record Result(
      boolean accepted,
      RejectionReason reason,
      double xyStdDevMeters,
      double thetaStdDevRadians,
      double confidence) {
    public static Result rejected(RejectionReason reason) {
      return new Result(
          false, reason, Double.NaN, ConfigVision.MEGATAG2_THETA_STD_DEV_RADIANS, 0.0);
    }
  }

  public static Result evaluate(
      VisionObservation observation,
      Pose2d currentEstimate,
      double nowSeconds,
      double gyroRateDegreesPerSecond,
      boolean allowLargeCorrection,
      AprilTagFieldLayout fieldLayout) {
    if (!allFinite(observation)) {
      return Result.rejected(RejectionReason.NON_FINITE_VALUE);
    }
    int[] tagIds = observation.tagIds();
    if (observation.tagCount() < 1 || tagIds.length == 0) {
      return Result.rejected(RejectionReason.NO_TAGS);
    }
    for (int tagId : tagIds) {
      if (fieldLayout.getTagPose(tagId).isEmpty()) {
        return Result.rejected(RejectionReason.UNKNOWN_TAG);
      }
    }

    double ageSeconds = nowSeconds - observation.timestampSeconds();
    if (ageSeconds > ConfigVision.MAX_MEASUREMENT_AGE_SECONDS) {
      return Result.rejected(RejectionReason.STALE_TIMESTAMP);
    }
    if (ageSeconds < -ConfigVision.MAX_FUTURE_TIMESTAMP_SECONDS) {
      return Result.rejected(RejectionReason.FUTURE_TIMESTAMP);
    }

    if (Math.abs(observation.robotPose().getZ()) > ConfigVision.MAX_ROBOT_POSE_Z_METERS) {
      return Result.rejected(RejectionReason.INVALID_HEIGHT);
    }
    double rollDegrees =
        Math.abs(Units.radiansToDegrees(observation.robotPose().getRotation().getX()));
    double pitchDegrees =
        Math.abs(Units.radiansToDegrees(observation.robotPose().getRotation().getY()));
    if (rollDegrees > ConfigVision.MAX_ROBOT_ROLL_PITCH_DEGREES
        || pitchDegrees > ConfigVision.MAX_ROBOT_ROLL_PITCH_DEGREES) {
      return Result.rejected(RejectionReason.INVALID_ROLL_PITCH);
    }

    Pose2d measuredPose = observation.robotPose().toPose2d();
    double margin = ConfigVision.FIELD_BORDER_MARGIN_METERS;
    if (measuredPose.getX() < -margin
        || measuredPose.getY() < -margin
        || measuredPose.getX() > fieldLayout.getFieldLength() + margin
        || measuredPose.getY() > fieldLayout.getFieldWidth() + margin) {
      return Result.rejected(RejectionReason.OUTSIDE_FIELD);
    }

    double distance = observation.averageTagDistanceMeters();
    if (distance < ConfigVision.MIN_TAG_DISTANCE_METERS) {
      return Result.rejected(RejectionReason.DISTANCE_TOO_SMALL);
    }
    double maxDistance =
        observation.tagCount() == 1
            ? ConfigVision.MAX_SINGLE_TAG_DISTANCE_METERS
            : ConfigVision.MAX_MULTI_TAG_DISTANCE_METERS;
    if (distance > maxDistance) {
      return Result.rejected(RejectionReason.DISTANCE_TOO_LARGE);
    }
    if (observation.tagCount() == 1
        && observation.maximumAmbiguity() > ConfigVision.MAX_SINGLE_TAG_AMBIGUITY) {
      return Result.rejected(RejectionReason.HIGH_AMBIGUITY);
    }
    if (Math.abs(gyroRateDegreesPerSecond)
        > ConfigVision.MAX_GYRO_RATE_DEGREES_PER_SECOND) {
      return Result.rejected(RejectionReason.HIGH_ANGULAR_VELOCITY);
    }

    double innovationMeters =
        currentEstimate.getTranslation().getDistance(measuredPose.getTranslation());
    double maxInnovation =
        ConfigVision.BASE_MAX_INNOVATION_METERS
            + ConfigVision.INNOVATION_PER_DISTANCE * distance
            + (observation.tagCount() > 1 ? 0.50 : 0.0);
    if (!allowLargeCorrection && innovationMeters > maxInnovation) {
      return Result.rejected(RejectionReason.LARGE_POSE_INNOVATION);
    }

    double modeledStdDev =
        (ConfigVision.BASE_XY_STD_DEV_METERS
                + ConfigVision.DISTANCE_STD_DEV_FACTOR * distance * distance)
            / Math.sqrt(observation.tagCount());
    double reportedStdDev =
        Math.max(observation.reportedXStdDevMeters(), observation.reportedYStdDevMeters());
    if (Double.isFinite(reportedStdDev) && reportedStdDev > 0.0) {
      modeledStdDev = Math.max(modeledStdDev, reportedStdDev);
    }
    double xyStdDev =
        MathUtil.clamp(
            modeledStdDev,
            ConfigVision.MIN_XY_STD_DEV_METERS,
            ConfigVision.MAX_XY_STD_DEV_METERS);
    double confidence =
        MathUtil.clamp(
            (1.0 / (1.0 + xyStdDev))
                * (observation.tagCount() > 1 ? 1.0 : 0.80)
                * (1.0 - Math.min(0.8, observation.maximumAmbiguity())),
            0.0,
            1.0);
    return new Result(
        true,
        RejectionReason.ACCEPTED,
        xyStdDev,
        ConfigVision.MEGATAG2_THETA_STD_DEV_RADIANS,
        confidence);
  }

  private static boolean allFinite(VisionObservation observation) {
    Pose2d pose = observation.robotPose().toPose2d();
    return Double.isFinite(pose.getX())
        && Double.isFinite(pose.getY())
        && Double.isFinite(observation.robotPose().getZ())
        && Double.isFinite(observation.timestampSeconds())
        && Double.isFinite(observation.averageTagDistanceMeters())
        && Double.isFinite(observation.maximumAmbiguity());
  }
}
