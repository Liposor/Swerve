package frc.robot.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ConfigVision;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.vision.VisionHeadingResetter.RejectionReason;
import frc.robot.vision.VisionHeadingResetter.Result;

/** Envia orientacao ao MegaTag2, filtra frames e oferece reset seguro do heading. */
public final class VisionSubsystem extends SubsystemBase {
  private final CommandSwerveDrivetrain drivetrain;
  private final List<CameraState> cameras = new ArrayList<>();
  private final StatusSignal<Angle> pitch;
  private final StatusSignal<Angle> roll;
  private final StatusSignal<AngularVelocity> yawRate;
  private final StatusSignal<AngularVelocity> pitchRate;
  private final StatusSignal<AngularVelocity> rollRate;
  private double lastUpdateSeconds = Double.NEGATIVE_INFINITY;
  private double lastTelemetrySeconds = Double.NEGATIVE_INFINITY;
  private boolean pigeonHealthy;
  private String pigeonStatusName = "StatusCodeNotInitialized";
  private Result lastHeadingResetResult =
      Result.rejected(RejectionReason.NO_FRESH_CAMERA_FRAME);
  private double lastHeadingBeforeResetDegrees = Double.NaN;
  private int successfulHeadingResets;

  public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    for (var cameraConfig : ConfigVision.CAMERAS) {
      cameras.add(new CameraState(new LimelightCameraIO(cameraConfig)));
    }

    var pigeon = drivetrain.getPigeon2();
    pitch = pigeon.getPitch();
    roll = pigeon.getRoll();
    yawRate = pigeon.getAngularVelocityZDevice();
    pitchRate = pigeon.getAngularVelocityYDevice();
    rollRate = pigeon.getAngularVelocityXDevice();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, pitch, roll, yawRate, pitchRate, rollRate);
  }

  /**
   * Comando instantaneo para o botao do piloto. Requer drivetrain por um ciclo e funciona
   * tambem enquanto o robo esta desabilitado.
   */
  public Command resetHeadingFromVisionCommand() {
    return Commands.runOnce(this::resetHeadingFromVision, this, drivetrain)
        .ignoringDisable(true)
        .withName("Vision/ResetHeadingFromMT1MT2");
  }

  @Override
  public void periodic() {
    double nowSeconds = Timer.getFPGATimestamp();
    if (nowSeconds - lastUpdateSeconds < ConfigVision.UPDATE_PERIOD_SECONDS) {
      return;
    }
    lastUpdateSeconds = nowSeconds;

    // refreshAll e nao bloqueante; o retorno impede o uso de valores antigos do Pigeon.
    var pigeonStatus =
        BaseStatusSignal.refreshAll(false, pitch, roll, yawRate, pitchRate, rollRate);
    pigeonHealthy = pigeonStatus.isOK();
    pigeonStatusName = pigeonStatus.getName();
    if (!pigeonHealthy) {
      publishTelemetryIfDue(nowSeconds);
      return;
    }

    double pitchDegrees = pitch.getValue().in(Degrees);
    double rollDegrees = roll.getValue().in(Degrees);
    double yawRateDegreesPerSecond = yawRate.getValue().in(DegreesPerSecond);
    double pitchRateDegreesPerSecond = pitchRate.getValue().in(DegreesPerSecond);
    double rollRateDegreesPerSecond = rollRate.getValue().in(DegreesPerSecond);
    double fieldYawDegrees = drivetrain.getState().Pose.getRotation().getDegrees();

    for (CameraState camera : cameras) {
      camera.io.updateRobotOrientation(
          fieldYawDegrees,
          yawRateDegreesPerSecond,
          pitchDegrees,
          pitchRateDegreesPerSecond,
          rollDegrees,
          rollRateDegreesPerSecond);
      camera.io
          .readLatestObservation()
          .ifPresent(
              observation ->
                  processObservation(
                      camera, observation, nowSeconds, yawRateDegreesPerSecond));
    }

    publishTelemetryIfDue(nowSeconds);
  }

  private void resetHeadingFromVision() {
    var state = drivetrain.getState();
    double translationSpeed =
        Math.hypot(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond);
    double omegaDegreesPerSecond = Math.toDegrees(state.Speeds.omegaRadiansPerSecond);
    if (translationSpeed
            > ConfigVision.HEADING_RESET_MAX_TRANSLATION_SPEED_METERS_PER_SECOND
        || Math.abs(omegaDegreesPerSecond)
            > ConfigVision.HEADING_RESET_MAX_OMEGA_DEGREES_PER_SECOND) {
      setHeadingResetResult(Result.rejected(RejectionReason.ROBOT_MOVING));
      DriverStation.reportWarning(
          String.format(
              "[VisionHeadingReset] Rejeitado: robo em movimento (%.2f m/s, %.1f deg/s).",
              translationSpeed,
              omegaDegreesPerSecond),
          false);
      return;
    }

    double nowSeconds = Timer.getFPGATimestamp();
    List<VisionHeadingResetSample> samples = new ArrayList<>();
    for (CameraState camera : cameras) {
      camera.io.readHeadingResetSample().ifPresent(samples::add);
    }

    Result result =
        VisionHeadingResetter.evaluate(
            samples,
            nowSeconds,
            pigeonHealthy,
            ConfigVision.FIELD_LAYOUT);
    setHeadingResetResult(result);
    if (!result.accepted()) {
      DriverStation.reportWarning(
          "[VisionHeadingReset] Rejeitado: " + result.reason().name(), false);
      return;
    }

    lastHeadingBeforeResetDegrees = state.Pose.getRotation().getDegrees();

    // API CTRE: altera somente a rotacao da pose; X e Y permanecem intocados.
    drivetrain.resetRotation(result.heading());
    successfulHeadingResets++;

    double correctionDegrees =
        Math.toDegrees(
            MathUtil.angleModulus(
                result.heading().getRadians() - state.Pose.getRotation().getRadians()));
    SmartDashboard.putNumber("Vision/HeadingReset/CorrectionDeg", correctionDegrees);
    DriverStation.reportWarning(
        String.format(
            "[VisionHeadingReset] OK: %.2f deg, correcao=%+.2f deg, fonte=%s, camera=%s.",
            result.heading().getDegrees(),
            correctionDegrees,
            result.source().name(),
            result.cameraNames()),
        false);
  }

  private void setHeadingResetResult(Result result) {
    lastHeadingResetResult = result;
    SmartDashboard.putBoolean("Vision/HeadingReset/Accepted", result.accepted());
    SmartDashboard.putString("Vision/HeadingReset/Reason", result.reason().name());
    SmartDashboard.putString("Vision/HeadingReset/Source", result.source().name());
    SmartDashboard.putString("Vision/HeadingReset/Cameras", result.cameraNames());
    SmartDashboard.putNumber("Vision/HeadingReset/TagCount", result.tagCount());
    SmartDashboard.putNumber(
        "Vision/HeadingReset/HeadingDeg", result.heading().getDegrees());
    SmartDashboard.putNumber(
        "Vision/HeadingReset/AverageDistanceMeters",
        result.averageTagDistanceMeters());
    SmartDashboard.putNumber(
        "Vision/HeadingReset/MaximumAmbiguity", result.maximumAmbiguity());
    SmartDashboard.putNumber(
        "Vision/HeadingReset/CameraSpreadDeg", result.headingSpreadDegrees());
  }

  private void publishTelemetryIfDue(double nowSeconds) {
    if (nowSeconds - lastTelemetrySeconds < ConfigVision.TELEMETRY_PERIOD_SECONDS) {
      return;
    }
    lastTelemetrySeconds = nowSeconds;
    publishTelemetry();
  }

  private void processObservation(
      CameraState camera,
      VisionObservation observation,
      double nowSeconds,
      double yawRateDegreesPerSecond) {
    VisionReliability.Result result =
        VisionReliability.evaluate(
            observation,
            drivetrain.getState().Pose,
            nowSeconds,
            yawRateDegreesPerSecond,
            DriverStation.isDisabled(),
            ConfigVision.FIELD_LAYOUT);

    camera.lastObservation = observation;
    camera.lastResult = result;
    if (!result.accepted()) {
      camera.rejectedFrames++;
      return;
    }

    drivetrain.addVisionMeasurement(
        observation.robotPose().toPose2d(),
        observation.timestampSeconds(),
        VecBuilder.fill(
            result.xyStdDevMeters(),
            result.xyStdDevMeters(),
            result.thetaStdDevRadians()));
    camera.acceptedFrames++;
  }

  private void publishTelemetry() {
    SmartDashboard.putBoolean("Vision/PigeonHealthy", pigeonHealthy);
    SmartDashboard.putString("Vision/PigeonStatus", pigeonStatusName);
    SmartDashboard.putNumber(
        "Vision/HeadingReset/SuccessfulCount", successfulHeadingResets);
    SmartDashboard.putNumber(
        "Vision/HeadingReset/PreviousHeadingDeg", lastHeadingBeforeResetDegrees);
    SmartDashboard.putString(
        "Vision/HeadingReset/LastReason", lastHeadingResetResult.reason().name());

    int totalAccepted = 0;
    int totalRejected = 0;
    for (CameraState camera : cameras) {
      String prefix = "Vision/" + camera.io.config().name() + "/";
      SmartDashboard.putBoolean(prefix + "Accepted", camera.lastResult.accepted());
      SmartDashboard.putString(prefix + "Reason", camera.lastResult.reason().name());
      SmartDashboard.putNumber(prefix + "Confidence", camera.lastResult.confidence());
      SmartDashboard.putNumber(prefix + "AcceptedFrames", camera.acceptedFrames);
      SmartDashboard.putNumber(prefix + "RejectedFrames", camera.rejectedFrames);
      if (camera.lastObservation != null) {
        SmartDashboard.putString(
            prefix + "VisibleTagIds", camera.lastObservation.tagIdsText());
        SmartDashboard.putNumber(
            prefix + "AverageDistanceMeters",
            camera.lastObservation.averageTagDistanceMeters());
      }
      totalAccepted += camera.acceptedFrames;
      totalRejected += camera.rejectedFrames;
    }
    SmartDashboard.putNumber("Vision/AcceptedFrames", totalAccepted);
    SmartDashboard.putNumber("Vision/RejectedFrames", totalRejected);
  }

  private static final class CameraState {
    final LimelightCameraIO io;
    VisionObservation lastObservation;
    VisionReliability.Result lastResult =
        VisionReliability.Result.rejected(
            VisionReliability.RejectionReason.NO_TAGS);
    int acceptedFrames;
    int rejectedFrames;

    CameraState(LimelightCameraIO io) {
      this.io = io;
    }
  }
}
