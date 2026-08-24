package frc.robot.vision;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose3d;

/** Um frame de visao normalizado, independente de hardware ou simulacao. */
public record VisionObservation(
    String cameraName,
    Pose3d robotPose,
    double timestampSeconds,
    int[] tagIds,
    int tagCount,
    double averageTagDistanceMeters,
    double averageTagAreaPercent,
    double maximumAmbiguity,
    double reportedXStdDevMeters,
    double reportedYStdDevMeters) {

  public VisionObservation {
    if (cameraName == null || cameraName.isBlank() || robotPose == null) {
      throw new IllegalArgumentException("Observacao precisa de camera e pose.");
    }
    tagIds = tagIds == null ? new int[0] : tagIds.clone();
  }

  @Override
  public int[] tagIds() {
    return tagIds.clone();
  }

  public String tagIdsText() {
    return Arrays.toString(tagIds);
  }
}
