package frc.robot.vision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Random;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ConfigVision;
import frc.robot.config.ConfigVision.CameraConfig;

/**
 * Simulador geometrico leve para tres Limelights.
 *
 * <p>Publica as mesmas chaves NT consumidas pelo MegaTag2. Nao simula pixels, o pipeline neural ou
 * oclusao por outros robos; o objetivo e validar FOV, IDs, filtros e autonomia sem sobrecarregar a
 * simulacao.
 */
public final class LimelightSimulation extends SubsystemBase {
  private final Supplier<Pose2d> robotPoseSupplier;
  private final List<CameraSim> cameras = new ArrayList<>();
  private double lastCaptureSeconds = Double.NEGATIVE_INFINITY;

  public LimelightSimulation(Supplier<Pose2d> robotPoseSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    for (CameraConfig config : ConfigVision.CAMERAS) {
      cameras.add(new CameraSim(config));
    }
  }

  @Override
  public void periodic() {
    if (!RobotBase.isSimulation()) {
      return;
    }

    double nowSeconds = Timer.getFPGATimestamp();
    for (CameraSim camera : cameras) {
      camera.publishIfReady(nowSeconds);
    }

    if (nowSeconds - lastCaptureSeconds < ConfigVision.OBSERVATION_PERIOD_SECONDS) {
      return;
    }
    lastCaptureSeconds = nowSeconds;

    Pose2d robotPose = robotPoseSupplier.get();
    for (CameraSim camera : cameras) {
      camera.capture(robotPose, nowSeconds);
    }
  }

  private static final class CameraSim {
    private final CameraConfig config;
    private final Random random;
    private final NetworkTableEntry targetValidEntry;
    private final NetworkTableEntry primaryTagEntry;
    private final NetworkTableEntry heartbeatEntry;
    private final NetworkTableEntry poseEntry;
    private final NetworkTableEntry rawFiducialsEntry;
    private final NetworkTableEntry stdDevsEntry;
    private final NetworkTableEntry visibleIdsEntry;
    private SimFrame pendingFrame;
    private long heartbeat;

    CameraSim(CameraConfig config) {
      this.config = config;
      random = new Random(2026L + config.name().hashCode());
      NetworkTable table = NetworkTableInstance.getDefault().getTable(config.name());
      targetValidEntry = table.getEntry("tv");
      primaryTagEntry = table.getEntry("tid");
      heartbeatEntry = table.getEntry("hb");
      poseEntry = table.getEntry("botpose_orb_wpiblue");
      rawFiducialsEntry = table.getEntry("rawfiducials");
      stdDevsEntry = table.getEntry("stddevs");
      visibleIdsEntry =
          NetworkTableInstance.getDefault()
              .getTable("VisionSim")
              .getSubTable(config.name())
              .getEntry("VisibleTagIds");
    }

    void capture(Pose2d robotPose2d, double captureTimeSeconds) {
      Pose3d robotPose = new Pose3d(robotPose2d);
      Pose3d cameraPose = robotPose.transformBy(config.robotToCamera());
      List<VisibleTag> visible = new ArrayList<>();

      double halfHorizontalFov = Units.degreesToRadians(config.horizontalFovDegrees() / 2.0);
      double halfVerticalFov = Units.degreesToRadians(config.verticalFovDegrees() / 2.0);
      for (AprilTag tag : ConfigVision.FIELD_LAYOUT.getTags()) {
        Pose3d cameraInTag = cameraPose.relativeTo(tag.pose);
        Pose3d tagInCamera = tag.pose.relativeTo(cameraPose);
        Translation3d translation = tagInCamera.getTranslation();
        double cameraDistanceFromTag = cameraInTag.getTranslation().getNorm();
        double tagFacingCosine =
            cameraDistanceFromTag > 1e-6
                ? cameraInTag.getX() / cameraDistanceFromTag
                : -1.0;
        if (translation.getX() <= 0.0
            || tagFacingCosine
                < Math.cos(Units.degreesToRadians(ConfigVision.SIM_MAX_TAG_OBLIQUITY_DEGREES))) {
          continue;
        }
        double distance = translation.getNorm();
        double horizontalAngle = Math.atan2(translation.getY(), translation.getX());
        double verticalAngle =
            Math.atan2(
                translation.getZ(), Math.hypot(translation.getX(), translation.getY()));
        if (distance <= config.maxTagDistanceMeters()
            && Math.abs(horizontalAngle) <= halfHorizontalFov
            && Math.abs(verticalAngle) <= halfVerticalFov) {
          double robotDistance =
              tag.pose.getTranslation().getDistance(robotPose.getTranslation());
          visible.add(
              new VisibleTag(
                  tag.ID, distance, robotDistance, horizontalAngle, verticalAngle));
        }
      }

      visible.sort(Comparator.comparingDouble(VisibleTag::cameraDistanceMeters));
      if (visible.size() > ConfigVision.SIM_MAX_TAGS_PER_CAMERA) {
        visible = new ArrayList<>(visible.subList(0, ConfigVision.SIM_MAX_TAGS_PER_CAMERA));
      }
      pendingFrame = new SimFrame(robotPose, captureTimeSeconds, visible);
    }

    void publishIfReady(double nowSeconds) {
      if (pendingFrame == null
          || nowSeconds - pendingFrame.captureTimeSeconds()
              < ConfigVision.SIM_LATENCY_MILLISECONDS / 1000.0) {
        return;
      }

      List<VisibleTag> tags = pendingFrame.tags();
      heartbeatEntry.setDouble(++heartbeat);
      if (tags.isEmpty()) {
        targetValidEntry.setDouble(0.0);
        primaryTagEntry.setDouble(-1.0);
        poseEntry.setDoubleArray(new double[0]);
        rawFiducialsEntry.setDoubleArray(new double[0]);
        visibleIdsEntry.setIntegerArray(new long[0]);
        pendingFrame = null;
        return;
      }

      double averageDistance =
          tags.stream().mapToDouble(VisibleTag::cameraDistanceMeters).average().orElse(0.0);
      double averageArea =
          tags.stream()
              .mapToDouble(tag -> Math.min(100.0, 8.0 / Math.max(0.04, tag.cameraDistanceMeters()
                  * tag.cameraDistanceMeters())))
              .average()
              .orElse(0.0);
      double translationNoise =
          ConfigVision.SIM_BASE_TRANSLATION_NOISE_METERS
              + ConfigVision.SIM_DISTANCE_NOISE_FACTOR * averageDistance * averageDistance;
      Pose3d truth = pendingFrame.robotPose();
      Pose3d noisyPose =
          new Pose3d(
              truth.getX() + random.nextGaussian() * translationNoise,
              truth.getY() + random.nextGaussian() * translationNoise,
              truth.getZ() + random.nextGaussian() * translationNoise * 0.25,
              new Rotation3d(
                  truth.getRotation().getX(),
                  truth.getRotation().getY(),
                  truth.getRotation().getZ()
                      + Units.degreesToRadians(
                          random.nextGaussian() * ConfigVision.SIM_YAW_NOISE_DEGREES)));

      double[] botpose =
          new double[] {
            noisyPose.getX(),
            noisyPose.getY(),
            noisyPose.getZ(),
            Units.radiansToDegrees(noisyPose.getRotation().getX()),
            Units.radiansToDegrees(noisyPose.getRotation().getY()),
            Units.radiansToDegrees(noisyPose.getRotation().getZ()),
            (nowSeconds - pendingFrame.captureTimeSeconds()) * 1000.0,
            tags.size(),
            tags.size(),
            averageDistance,
            averageArea
          };
      double[] rawFiducials = new double[tags.size() * 7];
      long[] ids = new long[tags.size()];
      for (int i = 0; i < tags.size(); i++) {
        VisibleTag tag = tags.get(i);
        int offset = i * 7;
        double area =
            Math.min(
                100.0,
                8.0
                    / Math.max(
                        0.04, tag.cameraDistanceMeters() * tag.cameraDistanceMeters()));
        rawFiducials[offset] = tag.id();
        rawFiducials[offset + 1] = Units.radiansToDegrees(tag.horizontalAngleRadians());
        rawFiducials[offset + 2] = Units.radiansToDegrees(tag.verticalAngleRadians());
        rawFiducials[offset + 3] = area;
        rawFiducials[offset + 4] = tag.cameraDistanceMeters();
        rawFiducials[offset + 5] = tag.robotDistanceMeters();
        rawFiducials[offset + 6] = MathUtil.clamp(0.015 * tag.cameraDistanceMeters(), 0.01, 0.20);
        ids[i] = tag.id();
      }

      double simulatedStdDev =
          MathUtil.clamp(
              translationNoise / Math.sqrt(tags.size()),
              ConfigVision.MIN_XY_STD_DEV_METERS,
              ConfigVision.MAX_XY_STD_DEV_METERS);
      targetValidEntry.setDouble(1.0);
      primaryTagEntry.setDouble(tags.get(0).id());
      poseEntry.setDoubleArray(botpose);
      rawFiducialsEntry.setDoubleArray(rawFiducials);
      stdDevsEntry.setDoubleArray(
          new double[] {
            Double.NaN,
            Double.NaN,
            Double.NaN,
            Double.NaN,
            Double.NaN,
            Double.NaN,
            simulatedStdDev,
            simulatedStdDev,
            ConfigVision.MEGATAG2_THETA_STD_DEV_RADIANS,
            Double.NaN,
            Double.NaN,
            Double.NaN
          });
      visibleIdsEntry.setIntegerArray(ids);
      pendingFrame = null;
    }
  }

  private record VisibleTag(
      int id,
      double cameraDistanceMeters,
      double robotDistanceMeters,
      double horizontalAngleRadians,
      double verticalAngleRadians) {}

  private record SimFrame(Pose3d robotPose, double captureTimeSeconds, List<VisibleTag> tags) {}
}