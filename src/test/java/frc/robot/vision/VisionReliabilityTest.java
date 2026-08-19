package frc.robot.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.config.ConfigVision;
import frc.robot.vision.VisionReliability.RejectionReason;

class VisionReliabilityTest {
  @Test
  void acceptsPlausibleSingleTagFrame() {
    VisionObservation observation = observation(new Pose3d(4.2, 4.0, 0.02, new Rotation3d()), 25);

    var result =
        VisionReliability.evaluate(
            observation,
            observation.robotPose().toPose2d(),
            10.0,
            20.0,
            false,
            ConfigVision.FIELD_LAYOUT);

    assertTrue(result.accepted());
    assertEquals(RejectionReason.ACCEPTED, result.reason());
  }

  @Test
  void rejectsImpossibleRobotHeightBeforeFusion() {
    VisionObservation observation =
        observation(new Pose3d(4.2, 4.0, 0.50, new Rotation3d()), 25);

    var result =
        VisionReliability.evaluate(
            observation,
            new Pose2d(),
            10.0,
            0.0,
            true,
            ConfigVision.FIELD_LAYOUT);

    assertEquals(RejectionReason.INVALID_HEIGHT, result.reason());
  }

  @Test
  void rejectsUnknownTagId() {
    VisionObservation observation = observation(new Pose3d(4.2, 4.0, 0.02, new Rotation3d()), 999);

    var result =
        VisionReliability.evaluate(
            observation,
            new Pose2d(4.2, 4.0, edu.wpi.first.math.geometry.Rotation2d.kZero),
            10.0,
            0.0,
            false,
            ConfigVision.FIELD_LAYOUT);

    assertEquals(RejectionReason.UNKNOWN_TAG, result.reason());
  }

  @Test
  void rejectsLargeStartupCorrectionUntilHeadingResetArmsIt() {
    VisionObservation observation = observation(new Pose3d(4.2, 4.0, 0.02, new Rotation3d()), 25);

    var result =
        VisionReliability.evaluate(
            observation,
            new Pose2d(),
            10.0,
            0.0,
            false,
            ConfigVision.FIELD_LAYOUT);

    assertFalse(result.accepted());
    assertEquals(RejectionReason.LARGE_POSE_INNOVATION, result.reason());
  }

  @Test
  void acceptsLargeCorrectionWhenHeadingResetExplicitlyArmsIt() {
    VisionObservation observation = observation(new Pose3d(4.2, 4.0, 0.02, new Rotation3d()), 25);

    var result =
        VisionReliability.evaluate(
            observation,
            new Pose2d(),
            10.0,
            0.0,
            true,
            ConfigVision.FIELD_LAYOUT);

    assertTrue(result.accepted());
    assertEquals(RejectionReason.ACCEPTED, result.reason());
  }

  private static VisionObservation observation(Pose3d pose, int tagId) {
    return new VisionObservation(
        "test-camera",
        pose,
        10.0,
        new int[] {tagId},
        1,
        1.5,
        2.0,
        0.10,
        0.15,
        0.15);
  }
}
