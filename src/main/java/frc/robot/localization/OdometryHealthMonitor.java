package frc.robot.localization;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ConfigLocalization;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Mantem historico de 20 ms e identifica impactos/bloqueios sem assumir que o IMU mede velocidade.
 *
 * <p>As heuristicas sao deliberadamente conservadoras. Rodas podem girar contra uma parede e o
 * Pigeon mede aceleracao, nao velocidade translacional; por isso a pose de recuperacao e apenas uma
 * recomendacao e o rollback automatico vem desligado.
 */
public final class OdometryHealthMonitor extends SubsystemBase {
  public enum HealthState {
    NOMINAL,
    IMPACT_SUSPECTED,
    BLOCKED,
    RECOVERING
  }

  private final CommandSwerveDrivetrain drivetrain;
  private final PoseSample[] history = new PoseSample[ConfigLocalization.HISTORY_CAPACITY];
  private final StatusSignal<LinearAcceleration> accelerationX;
  private final StatusSignal<LinearAcceleration> accelerationY;
  private int nextHistoryIndex;
  private int historySize;
  private boolean recoveryPoseCandidateAvailable;
  private boolean recoveryPoseAvailable;
  private double recoveryXMeters;
  private double recoveryYMeters;
  private double recoveryHeadingRadians;
  private HealthState healthState = HealthState.NOMINAL;
  private double requestedVelocityX;
  private double requestedVelocityY;
  private double requestedOmega;
  private double previousMeasuredSpeed;
  private double previousFilteredImuAcceleration;
  private double filteredImuAcceleration;
  private double lastSampleSeconds = Double.NEGATIVE_INFINITY;
  private double stateStartSeconds;
  private double stallStartSeconds = Double.NaN;
  private double slipStartSeconds = Double.NaN;
  private double stableStartSeconds = Double.NaN;
  private double lastTelemetrySeconds = Double.NEGATIVE_INFINITY;
  private double lastImuAcceleration;
  private double lastJerk;
  private double lastMeasuredSpeed;
  private double lastMeasuredOmega;
  private double lastWheelAcceleration;
  private boolean lastImpactCandidate;
  private boolean lastStallCandidate;
  private boolean lastSlipCandidate;
  private boolean lastStalled;
  private boolean lastSlipping;
  private boolean imuHealthy;
  private boolean wasImuHealthy;
  private String imuStatusName = "StatusCodeNotInitialized";

  public OdometryHealthMonitor(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    for (int i = 0; i < history.length; i++) {
      history[i] = new PoseSample();
    }
    accelerationX = drivetrain.getPigeon2().getAccelerationX();
    accelerationY = drivetrain.getPigeon2().getAccelerationY();
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, accelerationX, accelerationY);
  }

  /** Registra a intencao do controlador; chame tambem nos comandos autonomos. */
  public void setRequestedSpeeds(double velocityX, double velocityY, double omegaRadiansPerSecond) {
    requestedVelocityX = velocityX;
    requestedVelocityY = velocityY;
    requestedOmega = omegaRadiansPerSecond;
  }

  public void setRequestedSpeeds(ChassisSpeeds speeds) {
    setRequestedSpeeds(
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  public HealthState getHealthState() {
    return healthState;
  }

  /** Solicita X-lock somente depois de um bloqueio confirmado. */
  public boolean shouldHoldPosition() {
    return ConfigLocalization.ENABLE_COLLISION_X_LOCK
        && healthState == HealthState.BLOCKED;
  }

  public Optional<Pose2d> getRecommendedRecoveryPose() {
    if (!recoveryPoseAvailable) {
      return Optional.empty();
    }
    Pose2d current = drivetrain.getState().Pose;
    Pose2d candidate =
        new Pose2d(
            recoveryXMeters,
            recoveryYMeters,
            Rotation2d.fromRadians(recoveryHeadingRadians));
    if (current.getTranslation().getDistance(candidate.getTranslation())
        > ConfigLocalization.MAX_MANUAL_ROLLBACK_DISTANCE_METERS) {
      return Optional.empty();
    }
    return Optional.of(candidate);
  }

  /** Retorna true somente quando havia uma pose historica plausivel para aplicar. */
  public boolean applyRecommendedRecoveryPose() {
    Optional<Pose2d> recoveryPose = getRecommendedRecoveryPose();
    if (recoveryPose.isEmpty()) {
      return false;
    }
    drivetrain.resetPose(recoveryPose.get());
    transitionTo(HealthState.RECOVERING, Timer.getFPGATimestamp());
    return true;
  }

  @Override
  public void periodic() {
    double nowSeconds = Timer.getFPGATimestamp();
    if (nowSeconds - lastSampleSeconds < ConfigLocalization.SAMPLE_PERIOD_SECONDS) {
      return;
    }
    double dt =
        Double.isFinite(lastSampleSeconds)
            ? Math.max(0.001, nowSeconds - lastSampleSeconds)
            : ConfigLocalization.SAMPLE_PERIOD_SECONDS;
    lastSampleSeconds = nowSeconds;

    var imuStatus = BaseStatusSignal.refreshAll(false, accelerationX, accelerationY);
    imuHealthy = imuStatus.isOK();
    imuStatusName = imuStatus.getName();

    double imuAcceleration = 0.0;
    double jerk = 0.0;
    if (imuHealthy) {
      double ax = accelerationX.getValue().in(MetersPerSecondPerSecond);
      double ay = accelerationY.getValue().in(MetersPerSecondPerSecond);
      double rawImuAcceleration = Math.hypot(ax, ay);
      if (wasImuHealthy) {
        filteredImuAcceleration +=
            ConfigLocalization.IMU_ACCELERATION_FILTER_ALPHA
                * (rawImuAcceleration - filteredImuAcceleration);
        jerk =
            Math.abs(filteredImuAcceleration - previousFilteredImuAcceleration) / dt;
      } else {
        filteredImuAcceleration = rawImuAcceleration;
      }
      imuAcceleration = filteredImuAcceleration;
      previousFilteredImuAcceleration = filteredImuAcceleration;
    }
    wasImuHealthy = imuHealthy;

    var drivetrainState = drivetrain.getState();
    ChassisSpeeds measured = drivetrainState.Speeds;
    double measuredSpeed = Math.hypot(measured.vxMetersPerSecond, measured.vyMetersPerSecond);
    double requestedSpeed = Math.hypot(requestedVelocityX, requestedVelocityY);
    double wheelAcceleration = Math.abs(measuredSpeed - previousMeasuredSpeed) / dt;
    previousMeasuredSpeed = measuredSpeed;
    lastImuAcceleration = imuAcceleration;
    lastJerk = jerk;
    lastMeasuredSpeed = measuredSpeed;
    lastMeasuredOmega = measured.omegaRadiansPerSecond;
    lastWheelAcceleration = wheelAcceleration;

    saveSample(nowSeconds, drivetrainState.Pose, measured);

    boolean translationCommanded =
        requestedSpeed
            >= ConfigLocalization.MIN_COMMANDED_TRANSLATION_FOR_IMPACT_METERS_PER_SECOND;
    boolean impact =
        ConfigLocalization.ENABLE_COLLISION_MONITOR
            && imuHealthy
            && translationCommanded
            && (imuAcceleration
                    >= ConfigLocalization.IMPACT_ACCELERATION_METERS_PER_SECOND_SQUARED
                || jerk >= ConfigLocalization.IMPACT_JERK_METERS_PER_SECOND_CUBED);
    boolean stallCandidate =
        requestedSpeed
                >= ConfigLocalization.MIN_COMMANDED_SPEED_FOR_STALL_METERS_PER_SECOND
            && measuredSpeed
                <= requestedSpeed
                    * ConfigLocalization.MAX_MEASURED_TO_COMMANDED_STALL_RATIO;
    boolean slipCandidate =
        ConfigLocalization.ENABLE_COLLISION_MONITOR
            && imuHealthy
            && translationCommanded
            && wheelAcceleration >= ConfigLocalization.MIN_WHEEL_ACCELERATION_FOR_SLIP
            && imuAcceleration
                <= wheelAcceleration
                    * ConfigLocalization.MAX_IMU_TO_WHEEL_ACCELERATION_RATIO;
    boolean stalled = debounced(stallCandidate, nowSeconds, true);
    boolean slipping = debounced(slipCandidate, nowSeconds, false);
    boolean stable =
        !imuHealthy
            || imuAcceleration <= ConfigLocalization.STABLE_ACCELERATION_METERS_PER_SECOND_SQUARED;

    lastImpactCandidate = impact;
    lastStallCandidate = stallCandidate;
    lastSlipCandidate = slipCandidate;
    lastStalled = stalled;
    lastSlipping = slipping;

    if (DriverStation.isDisabled() || !ConfigLocalization.ENABLE_COLLISION_MONITOR) {
      transitionTo(HealthState.NOMINAL, nowSeconds);
      if (nowSeconds - lastTelemetrySeconds >= 0.10) {
        lastTelemetrySeconds = nowSeconds;
        publishTelemetry(requestedSpeed);
      }
      return;
    }

    switch (healthState) {
      case NOMINAL -> {
        if (impact) {
          captureRecoveryPose();
          transitionTo(HealthState.IMPACT_SUSPECTED, nowSeconds);
        }
      }
      case IMPACT_SUSPECTED -> {
        /*
         * Slip sozinho nao confirma colisao: em um carrinho as rodas aceleram, mas o chassi e o
         * Pigeon nao. Para frear, exigimos impacto seguido de stall persistente.
         */
        if (stalled) {
          recoveryPoseAvailable = recoveryPoseCandidateAvailable;
          transitionTo(HealthState.BLOCKED, nowSeconds);
          if (ConfigLocalization.ENABLE_AUTOMATIC_COLLISION_ROLLBACK) {
            applyRecommendedRecoveryPose();
          }
        } else if (nowSeconds - stateStartSeconds
            >= ConfigLocalization.COLLISION_CONFIRMATION_WINDOW_SECONDS) {
          transitionTo(HealthState.NOMINAL, nowSeconds);
        }
      }
      case BLOCKED -> {
        boolean driverReleasedMotion =
            requestedSpeed < ConfigLocalization.BLOCK_RELEASE_TRANSLATION_METERS_PER_SECOND
                && Math.abs(requestedOmega)
                    < ConfigLocalization.BLOCK_RELEASE_OMEGA_RADIANS_PER_SECOND;
        if (driverReleasedMotion
            && stableFor(
                stable, nowSeconds, ConfigLocalization.BLOCK_RELEASE_STABLE_SECONDS)) {
          transitionTo(HealthState.RECOVERING, nowSeconds);
        }
      }
      case RECOVERING -> {
        if (stableFor(stable, nowSeconds, ConfigLocalization.RECOVERY_STABLE_SECONDS)) {
          transitionTo(HealthState.NOMINAL, nowSeconds);
        } else if (impact) {
          captureRecoveryPose();
          transitionTo(HealthState.IMPACT_SUSPECTED, nowSeconds);
        }
      }
    }

    if (nowSeconds - lastTelemetrySeconds >= 0.10) {
      lastTelemetrySeconds = nowSeconds;
      publishTelemetry(requestedSpeed);
    }
  }

  private boolean debounced(boolean condition, double nowSeconds, boolean stall) {
    double start = stall ? stallStartSeconds : slipStartSeconds;
    if (!condition) {
      if (stall) {
        stallStartSeconds = Double.NaN;
      } else {
        slipStartSeconds = Double.NaN;
      }
      return false;
    }
    if (!Double.isFinite(start)) {
      start = nowSeconds;
      if (stall) {
        stallStartSeconds = start;
      } else {
        slipStartSeconds = start;
      }
    }
    double required =
        stall
            ? ConfigLocalization.STALL_DEBOUNCE_SECONDS
            : ConfigLocalization.SLIP_DEBOUNCE_SECONDS;
    return nowSeconds - start >= required;
  }

  private boolean stableFor(boolean stable, double nowSeconds, double requiredSeconds) {
    if (!stable) {
      stableStartSeconds = Double.NaN;
      return false;
    }
    if (!Double.isFinite(stableStartSeconds)) {
      stableStartSeconds = nowSeconds;
    }
    return nowSeconds - stableStartSeconds >= requiredSeconds;
  }

  private void transitionTo(HealthState nextState, double nowSeconds) {
    if (healthState == nextState) {
      return;
    }
    healthState = nextState;
    stateStartSeconds = nowSeconds;
    resetDebounceTimers();
    if (nextState == HealthState.NOMINAL) {
      recoveryPoseCandidateAvailable = false;
      recoveryPoseAvailable = false;
    }
  }

  private void resetDebounceTimers() {
    stallStartSeconds = Double.NaN;
    slipStartSeconds = Double.NaN;
    stableStartSeconds = Double.NaN;
  }

  private void saveSample(double timestamp, Pose2d pose, ChassisSpeeds speeds) {
    PoseSample sample = history[nextHistoryIndex];
    sample.timestampSeconds = timestamp;
    sample.xMeters = pose.getX();
    sample.yMeters = pose.getY();
    sample.headingRadians = pose.getRotation().getRadians();
    sample.velocityX = speeds.vxMetersPerSecond;
    sample.velocityY = speeds.vyMetersPerSecond;
    sample.omega = speeds.omegaRadiansPerSecond;
    nextHistoryIndex = (nextHistoryIndex + 1) % history.length;
    historySize = Math.min(history.length, historySize + 1);
  }

  /**
   * Reconstroi a pose anterior ao impacto a partir de uma amostra confiavel e das velocidades
   * relativas salvas. A ultima amostra, potencialmente contaminada pelo impacto, nao e integrada.
   */
  private void captureRecoveryPose() {
    int lookback = ConfigLocalization.TRUSTED_POSE_LOOKBACK_SAMPLES;
    if (historySize <= lookback) {
      recoveryPoseCandidateAvailable = false;
      return;
    }

    int startIndex = floorMod(nextHistoryIndex - 1 - lookback, history.length);
    PoseSample previous = history[startIndex];
    double x = previous.xMeters;
    double y = previous.yMeters;
    double heading = previous.headingRadians;
    for (int step = 1; step < lookback; step++) {
      int index = (startIndex + step) % history.length;
      PoseSample current = history[index];
      double dt =
          Math.max(
              0.0,
              Math.min(
                  0.040, current.timestampSeconds - previous.timestampSeconds));
      double cosine = Math.cos(heading);
      double sine = Math.sin(heading);
      x += (previous.velocityX * cosine - previous.velocityY * sine) * dt;
      y += (previous.velocityX * sine + previous.velocityY * cosine) * dt;
      heading += previous.omega * dt;
      previous = current;
    }
    recoveryXMeters = x;
    recoveryYMeters = y;
    recoveryHeadingRadians = heading;
    recoveryPoseCandidateAvailable = true;
  }

  private void publishTelemetry(double requestedSpeed) {
    SmartDashboard.putString("OdometryHealth/State", healthState.name());
    SmartDashboard.putBoolean("OdometryHealth/ImuHealthy", imuHealthy);
    SmartDashboard.putString("OdometryHealth/ImuStatus", imuStatusName);
    SmartDashboard.putNumber("OdometryHealth/HistorySamples", historySize);
    SmartDashboard.putNumber("OdometryHealth/RequestedTranslationMps", requestedSpeed);
    SmartDashboard.putNumber("OdometryHealth/MeasuredTranslationMps", lastMeasuredSpeed);
    SmartDashboard.putNumber("OdometryHealth/RequestedOmegaRadPerSec", requestedOmega);
    SmartDashboard.putNumber("OdometryHealth/MeasuredOmegaRadPerSec", lastMeasuredOmega);
    SmartDashboard.putNumber("OdometryHealth/WheelAccelerationMps2", lastWheelAcceleration);
    SmartDashboard.putNumber("OdometryHealth/ImuAccelerationMps2", lastImuAcceleration);
    SmartDashboard.putNumber("OdometryHealth/JerkMps3", lastJerk);
    SmartDashboard.putBoolean("OdometryHealth/ImpactCandidate", lastImpactCandidate);
    SmartDashboard.putBoolean("OdometryHealth/StallCandidate", lastStallCandidate);
    SmartDashboard.putBoolean("OdometryHealth/SlipCandidate", lastSlipCandidate);
    SmartDashboard.putBoolean("OdometryHealth/Stalled", lastStalled);
    SmartDashboard.putBoolean("OdometryHealth/Slipping", lastSlipping);
    SmartDashboard.putBoolean(
        "OdometryHealth/CollisionXLockEnabled",
        ConfigLocalization.ENABLE_COLLISION_X_LOCK);
    SmartDashboard.putBoolean("OdometryHealth/HoldingPosition", shouldHoldPosition());
    SmartDashboard.putBoolean(
        "OdometryHealth/RecoveryPoseAvailable", getRecommendedRecoveryPose().isPresent());
  }

  private static int floorMod(int value, int modulus) {
    int result = value % modulus;
    return result < 0 ? result + modulus : result;
  }

  private static final class PoseSample {
    double timestampSeconds;
    double xMeters;
    double yMeters;
    double headingRadians;
    double velocityX;
    double velocityY;
    double omega;
  }
}