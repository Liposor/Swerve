package frc.robot.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.ConfigVision;
import frc.robot.config.ConfigVision.CameraConfig;

/** Le MegaTag1 e MegaTag2 diretamente por NetworkTables. */
public final class LimelightCameraIO {
  private static final double[] EMPTY = new double[0];
  private static final int RAW_FIDUCIAL_STRIDE = 7;
  private static final int MT1_STD_DEV_OFFSET = 0;
  private static final int MT2_STD_DEV_OFFSET = 6;

  private final CameraConfig config;
  private final NetworkTableEntry heartbeatEntry;
  private final NetworkTableEntry targetValidEntry;
  private final NetworkTableEntry megaTag1PoseEntry;
  private final NetworkTableEntry megaTag2PoseEntry;
  private final NetworkTableEntry rawFiducialsEntry;
  private final NetworkTableEntry primaryTagEntry;
  private final NetworkTableEntry stdDevsEntry;
  private final NetworkTableEntry orientationEntry;
  private final NetworkTableEntry cameraPoseEntry;
  private final double[] orientation = new double[6];
  private double lastHeartbeat = Double.NaN;
  private double lastHeartbeatChangeSeconds = Double.NEGATIVE_INFINITY;

  public LimelightCameraIO(CameraConfig config) {
    this.config = config;
    NetworkTable table = NetworkTableInstance.getDefault().getTable(config.name());
    heartbeatEntry = table.getEntry("hb");
    targetValidEntry = table.getEntry("tv");
    megaTag1PoseEntry = table.getEntry("botpose_wpiblue");
    megaTag2PoseEntry = table.getEntry("botpose_orb_wpiblue");
    rawFiducialsEntry = table.getEntry("rawfiducials");
    primaryTagEntry = table.getEntry("tid");
    stdDevsEntry = table.getEntry("stddevs");
    orientationEntry = table.getEntry("robot_orientation_set");
    cameraPoseEntry = table.getEntry("camerapose_robotspace_set");
    publishRobotSpaceCameraPose();
  }

  public CameraConfig config() {
    return config;
  }

  /** MegaTag2 exige orientacao de origem azul e velocidades angulares em graus/s. */
  public void updateRobotOrientation(
      double yawDegrees,
      double yawRateDegreesPerSecond,
      double pitchDegrees,
      double pitchRateDegreesPerSecond,
      double rollDegrees,
      double rollRateDegreesPerSecond) {
    orientation[0] = yawDegrees;
    orientation[1] = yawRateDegreesPerSecond;
    orientation[2] = pitchDegrees;
    orientation[3] = pitchRateDegreesPerSecond;
    orientation[4] = rollDegrees;
    orientation[5] = rollRateDegreesPerSecond;
    orientationEntry.setDoubleArray(orientation);
  }

  /** Retorna somente frames MegaTag2 novos para a fusao normal de X/Y. */
  public Optional<VisionObservation> readLatestObservation() {
    double nowSeconds = Timer.getFPGATimestamp();
    if (!markAndCheckNewHeartbeat(nowSeconds)) {
      return Optional.empty();
    }
    if (targetValidEntry.getDouble(0.0) != 1.0) {
      return Optional.empty();
    }
    return parseObservation(megaTag2PoseEntry, MT2_STD_DEV_OFFSET, nowSeconds);
  }

  /**
   * Le o ultimo par MT1/MT2 mesmo que o frame ja tenha sido usado pela fusao normal.
   * O heartbeat impede que um valor antigo retido no NetworkTables seja usado para zerar heading.
   */
  public Optional<VisionHeadingResetSample> readHeadingResetSample() {
    double nowSeconds = Timer.getFPGATimestamp();
    updateHeartbeatAge(nowSeconds);

    if (!Double.isFinite(lastHeartbeat)
        || nowSeconds - lastHeartbeatChangeSeconds
            > ConfigVision.HEADING_RESET_MAX_CAMERA_HEARTBEAT_AGE_SECONDS
        || targetValidEntry.getDouble(0.0) != 1.0) {
      return Optional.empty();
    }

    VisionObservation megaTag1 =
        parseObservation(megaTag1PoseEntry, MT1_STD_DEV_OFFSET, nowSeconds).orElse(null);
    VisionObservation megaTag2 =
        parseObservation(megaTag2PoseEntry, MT2_STD_DEV_OFFSET, nowSeconds).orElse(null);

    if (megaTag1 == null && megaTag2 == null) {
      return Optional.empty();
    }
    return Optional.of(new VisionHeadingResetSample(config.name(), megaTag1, megaTag2));
  }

  private Optional<VisionObservation> parseObservation(
      NetworkTableEntry poseEntry,
      int stdDevOffset,
      double nowSeconds) {
    double[] pose = poseEntry.getDoubleArray(EMPTY);
    if (pose.length < 11) {
      return Optional.empty();
    }

    double latencySeconds = Math.max(0.0, pose[6]) / 1000.0;
    double timestampSeconds = nowSeconds - latencySeconds;
    int reportedTagCount = Math.max(0, (int) Math.round(pose[7]));

    double[] rawFiducials = rawFiducialsEntry.getDoubleArray(EMPTY);
    int parsedCount = rawFiducials.length / RAW_FIDUCIAL_STRIDE;
    int[] tagIds;
    double maxAmbiguity = 0.0;
    if (parsedCount > 0) {
      tagIds = new int[parsedCount];
      for (int i = 0; i < parsedCount; i++) {
        int offset = i * RAW_FIDUCIAL_STRIDE;
        tagIds[i] = (int) Math.round(rawFiducials[offset]);
        maxAmbiguity = Math.max(maxAmbiguity, rawFiducials[offset + 6]);
      }
    } else {
      int primaryId = (int) Math.round(primaryTagEntry.getDouble(-1));
      tagIds = primaryId > 0 ? new int[] {primaryId} : new int[0];
    }

    double[] stdDevs = stdDevsEntry.getDoubleArray(EMPTY);
    double reportedXStdDev =
        stdDevs.length > stdDevOffset ? stdDevs[stdDevOffset] : Double.NaN;
    double reportedYStdDev =
        stdDevs.length > stdDevOffset + 1 ? stdDevs[stdDevOffset + 1] : Double.NaN;

    Pose3d robotPose =
        new Pose3d(
            pose[0],
            pose[1],
            pose[2],
            new Rotation3d(
                Units.degreesToRadians(pose[3]),
                Units.degreesToRadians(pose[4]),
                Units.degreesToRadians(pose[5])));

    return Optional.of(
        new VisionObservation(
            config.name(),
            robotPose,
            timestampSeconds,
            tagIds,
            reportedTagCount,
            pose[9],
            pose[10],
            maxAmbiguity,
            reportedXStdDev,
            reportedYStdDev));
  }

  private boolean markAndCheckNewHeartbeat(double nowSeconds) {
    double heartbeat = heartbeatEntry.getDouble(Double.NaN);
    if (!Double.isFinite(heartbeat)) {
      return true;
    }
    if (Double.isFinite(lastHeartbeat) && heartbeat == lastHeartbeat) {
      return false;
    }
    lastHeartbeat = heartbeat;
    lastHeartbeatChangeSeconds = nowSeconds;
    return true;
  }

  private void updateHeartbeatAge(double nowSeconds) {
    double heartbeat = heartbeatEntry.getDouble(Double.NaN);
    if (Double.isFinite(heartbeat)
        && (!Double.isFinite(lastHeartbeat) || heartbeat != lastHeartbeat)) {
      lastHeartbeat = heartbeat;
      lastHeartbeatChangeSeconds = nowSeconds;
    }
  }

  private void publishRobotSpaceCameraPose() {
    Transform3d transform = config.robotToCamera();
    cameraPoseEntry.setDoubleArray(
        new double[] {
          transform.getX(),
          -transform.getY(), // Limelight documenta este eixo como "right".
          transform.getZ(),
          Units.radiansToDegrees(transform.getRotation().getX()),
          Units.radiansToDegrees(transform.getRotation().getY()),
          Units.radiansToDegrees(transform.getRotation().getZ())
        });
  }
}
