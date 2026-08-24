package frc.robot.vision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.config.ConfigVision;

/** Seleciona um heading absoluto sem usar o yaw circular devolvido pelo MegaTag2. */
public final class VisionHeadingResetter {
  private VisionHeadingResetter() {}

  public enum Source {
    NONE,
    MEGATAG1_MULTI_TAG,
    MEGATAG1_SINGLE_TAG_VALIDATED_BY_MEGATAG2
  }

  public enum RejectionReason {
    ACCEPTED,
    ROBOT_MOVING,
    NO_FRESH_CAMERA_FRAME,
    NO_VALID_ESTIMATE,
    PIGEON_UNHEALTHY_FOR_MEGATAG2,
    CAMERAS_DISAGREE
  }

  public record Result(
      boolean accepted,
      RejectionReason reason,
      Rotation2d heading,
      Source source,
      String cameraNames,
      int tagCount,
      double averageTagDistanceMeters,
      double maximumAmbiguity,
      double headingSpreadDegrees) {

    public static Result rejected(RejectionReason reason) {
      return new Result(
          false,
          reason,
          Rotation2d.kZero,
          Source.NONE,
          "",
          0,
          Double.NaN,
          Double.NaN,
          Double.NaN);
    }
  }

  /**
   * Regras:
   *
   * <ul>
   *   <li>MT1 com duas ou mais tags: yaw absoluto aceito diretamente.</li>
   *   <li>MT1 com uma tag: aceito somente quando MT2 simultaneo confirma ID e translacao.</li>
   *   <li>O yaw do MT2 nunca participa da media.</li>
   * </ul>
   */
  public static Result evaluate(
      List<VisionHeadingResetSample> samples,
      double nowSeconds,
      boolean pigeonHealthy,
      AprilTagFieldLayout fieldLayout) {
    if (samples == null || samples.isEmpty()) {
      return Result.rejected(RejectionReason.NO_FRESH_CAMERA_FRAME);
    }

    List<Candidate> multiTagCandidates = new ArrayList<>();
    List<Candidate> singleTagCandidates = new ArrayList<>();
    boolean sawSingleTagPair = false;

    for (VisionHeadingResetSample sample : samples) {
      VisionObservation mt1 = sample.megaTag1();
      VisionObservation mt2 = sample.megaTag2();

      if (!isGeometricallyPlausible(mt1, nowSeconds, fieldLayout)) {
        continue;
      }

      if (mt1.tagCount() >= ConfigVision.HEADING_RESET_MEGATAG1_MIN_TAGS) {
        multiTagCandidates.add(
            candidateFrom(
                mt1,
                Source.MEGATAG1_MULTI_TAG,
                Math.max(1.0, mt1.tagCount() * 2.0)));
        continue;
      }

      if (mt1.tagCount() != 1
          || mt2 == null
          || mt2.tagCount() < ConfigVision.HEADING_RESET_MEGATAG2_MIN_TAGS) {
        continue;
      }
      sawSingleTagPair = true;

      if (!pigeonHealthy
          || !isGeometricallyPlausible(mt2, nowSeconds, fieldLayout)
          || !sharesAtLeastOneTag(mt1.tagIds(), mt2.tagIds())
          || mt1.maximumAmbiguity()
              > ConfigVision.HEADING_RESET_MAX_SINGLE_TAG_AMBIGUITY
          || mt1.averageTagDistanceMeters()
              > ConfigVision.HEADING_RESET_MAX_SINGLE_TAG_DISTANCE_METERS
          || Math.abs(mt1.timestampSeconds() - mt2.timestampSeconds())
              > ConfigVision.HEADING_RESET_MAX_MT1_MT2_TIMESTAMP_DIFFERENCE_SECONDS) {
        continue;
      }

      double translationDifference =
          mt1.robotPose()
              .toPose2d()
              .getTranslation()
              .getDistance(mt2.robotPose().toPose2d().getTranslation());
      if (translationDifference
          > ConfigVision.HEADING_RESET_MAX_MT1_MT2_TRANSLATION_DIFFERENCE_METERS) {
        continue;
      }

      double validationWeight =
          1.0
              / (1.0
                  + translationDifference
                  + 4.0 * Math.max(0.0, mt1.maximumAmbiguity()));
      singleTagCandidates.add(
          candidateFrom(
              mt1,
              Source.MEGATAG1_SINGLE_TAG_VALIDATED_BY_MEGATAG2,
              validationWeight));
    }

    List<Candidate> selectedCandidates =
        !multiTagCandidates.isEmpty() ? multiTagCandidates : singleTagCandidates;
    if (selectedCandidates.isEmpty()) {
      if (sawSingleTagPair && !pigeonHealthy) {
        return Result.rejected(RejectionReason.PIGEON_UNHEALTHY_FOR_MEGATAG2);
      }
      return Result.rejected(RejectionReason.NO_VALID_ESTIMATE);
    }

    // A melhor observacao serve de referencia para detectar cameras montadas/calibradas errado.
    Candidate best =
        selectedCandidates.stream()
            .max(Comparator.comparingDouble(Candidate::weight))
            .orElseThrow();
    double maximumSpreadRadians = 0.0;
    for (Candidate candidate : selectedCandidates) {
      double disagreement =
          Math.abs(
              MathUtil.angleModulus(
                  candidate.heading().minus(best.heading()).getRadians()));
      maximumSpreadRadians = Math.max(maximumSpreadRadians, disagreement);
    }
    if (Units.radiansToDegrees(maximumSpreadRadians)
        > ConfigVision.HEADING_RESET_MAX_CAMERA_DISAGREEMENT_DEGREES) {
      return Result.rejected(RejectionReason.CAMERAS_DISAGREE);
    }

    double sineSum = 0.0;
    double cosineSum = 0.0;
    double distanceWeightedSum = 0.0;
    double ambiguityWeightedSum = 0.0;
    double weightSum = 0.0;
    int maximumTagCount = 0;
    for (Candidate candidate : selectedCandidates) {
      double weight = candidate.weight();
      sineSum += Math.sin(candidate.heading().getRadians()) * weight;
      cosineSum += Math.cos(candidate.heading().getRadians()) * weight;
      distanceWeightedSum += candidate.distanceMeters() * weight;
      ambiguityWeightedSum += candidate.ambiguity() * weight;
      weightSum += weight;
      maximumTagCount = Math.max(maximumTagCount, candidate.tagCount());
    }

    Rotation2d averagedHeading = Rotation2d.fromRadians(Math.atan2(sineSum, cosineSum));
    String cameras =
        selectedCandidates.stream()
            .map(Candidate::cameraName)
            .distinct()
            .sorted()
            .collect(Collectors.joining(", "));

    return new Result(
        true,
        RejectionReason.ACCEPTED,
        averagedHeading,
        best.source(),
        cameras,
        maximumTagCount,
        distanceWeightedSum / weightSum,
        ambiguityWeightedSum / weightSum,
        Units.radiansToDegrees(maximumSpreadRadians));
  }

  private static Candidate candidateFrom(
      VisionObservation observation,
      Source source,
      double sourceWeight) {
    double qualityWeight =
        sourceWeight
            / (1.0
                + Math.max(0.0, observation.averageTagDistanceMeters())
                + 3.0 * Math.max(0.0, observation.maximumAmbiguity()));
    return new Candidate(
        observation.robotPose().toPose2d().getRotation(),
        source,
        observation.cameraName(),
        observation.tagCount(),
        observation.averageTagDistanceMeters(),
        observation.maximumAmbiguity(),
        qualityWeight);
  }

  private static boolean isGeometricallyPlausible(
      VisionObservation observation,
      double nowSeconds,
      AprilTagFieldLayout fieldLayout) {
    if (observation == null
        || observation.tagCount() < 1
        || observation.tagIds().length == 0) {
      return false;
    }

    Pose2d pose = observation.robotPose().toPose2d();
    if (!Double.isFinite(pose.getX())
        || !Double.isFinite(pose.getY())
        || !Double.isFinite(pose.getRotation().getRadians())
        || !Double.isFinite(observation.robotPose().getZ())
        || !Double.isFinite(observation.timestampSeconds())
        || !Double.isFinite(observation.averageTagDistanceMeters())
        || !Double.isFinite(observation.maximumAmbiguity())) {
      return false;
    }

    double ageSeconds = nowSeconds - observation.timestampSeconds();
    if (ageSeconds > ConfigVision.HEADING_RESET_MAX_MEASUREMENT_AGE_SECONDS
        || ageSeconds < -ConfigVision.MAX_FUTURE_TIMESTAMP_SECONDS) {
      return false;
    }
    if (observation.averageTagDistanceMeters() < ConfigVision.MIN_TAG_DISTANCE_METERS
        || observation.averageTagDistanceMeters()
            > ConfigVision.MAX_MULTI_TAG_DISTANCE_METERS
        || Math.abs(observation.robotPose().getZ())
            > ConfigVision.MAX_ROBOT_POSE_Z_METERS) {
      return false;
    }

    double rollDegrees =
        Math.abs(Units.radiansToDegrees(observation.robotPose().getRotation().getX()));
    double pitchDegrees =
        Math.abs(Units.radiansToDegrees(observation.robotPose().getRotation().getY()));
    if (rollDegrees > ConfigVision.MAX_ROBOT_ROLL_PITCH_DEGREES
        || pitchDegrees > ConfigVision.MAX_ROBOT_ROLL_PITCH_DEGREES) {
      return false;
    }

    double margin = ConfigVision.FIELD_BORDER_MARGIN_METERS;
    if (pose.getX() < -margin
        || pose.getY() < -margin
        || pose.getX() > fieldLayout.getFieldLength() + margin
        || pose.getY() > fieldLayout.getFieldWidth() + margin) {
      return false;
    }

    for (int tagId : observation.tagIds()) {
      if (fieldLayout.getTagPose(tagId).isEmpty()) {
        return false;
      }
    }
    return true;
  }

  private static boolean sharesAtLeastOneTag(int[] first, int[] second) {
    for (int firstId : first) {
      for (int secondId : second) {
        if (firstId == secondId) {
          return true;
        }
      }
    }
    return false;
  }

  private record Candidate(
      Rotation2d heading,
      Source source,
      String cameraName,
      int tagCount,
      double distanceMeters,
      double ambiguity,
      double weight) {}
}
