package frc.robot.localization;

import java.util.Locale;
import java.util.function.Consumer;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.ConfigLocalization;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Mede automaticamente a posicao XY dos quatro modulos do swerve.
 *
 * <p>A regressao usa o deslocamento de cada roda e o delta angular bruto do Pigeon. O deslocamento
 * medio das quatro rodas e removido em cada amostra, portanto uma pequena deriva fisica do robo
 * nao e confundida com o raio dos modulos.
 *
 * <p>Execute somente no carpete, com o robo completo e em uma area livre. O comando exige o
 * drivetrain, gira nos dois sentidos, freia sozinho e apenas publica o resultado; ele nunca altera
 * o TunerConstants automaticamente.
 */
public final class SwerveGeometryCalibrationCommand extends Command {
  private static final int MODULE_COUNT = 4;
  private static final String[] MODULE_NAMES = {
    "FrontLeft", "FrontRight", "BackLeft", "BackRight"
  };
  private static final int[] EXPECTED_X_SIGNS = {1, 1, -1, -1};
  private static final int[] EXPECTED_Y_SIGNS = {1, -1, 1, -1};
  private static final double METERS_TO_INCHES = 39.37007874015748;
  private static final double TELEMETRY_PERIOD_SECONDS = 0.10;

  private enum Phase {
    IDLE,
    RAMP_CCW,
    RECORD_CCW,
    SETTLE,
    RAMP_CW,
    RECORD_CW,
    DONE,
    FAILED,
    CANCELLED
  }

  private final CommandSwerveDrivetrain drivetrain;
  private final Consumer<ChassisSpeeds> requestedSpeedsObserver;
  private final SwerveRequest.RobotCentric rotateRequest =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final DirectionAccumulator counterClockwise = new DirectionAccumulator();
  private final DirectionAccumulator clockwise = new DirectionAccumulator();

  private Phase phase = Phase.IDLE;
  private double commandStartSeconds;
  private double phaseStartSeconds;
  private double lastTelemetrySeconds = Double.NEGATIVE_INFINITY;
  private String status = "Pronto";
  private boolean resultValid;
  private SwerveModulePosition[] previousPositions;
  private Rotation2d previousRawHeading;

  public SwerveGeometryCalibrationCommand(
      CommandSwerveDrivetrain drivetrain,
      Consumer<ChassisSpeeds> requestedSpeedsObserver) {
    this.drivetrain = drivetrain;
    this.requestedSpeedsObserver =
        requestedSpeedsObserver != null ? requestedSpeedsObserver : speeds -> {};
    addRequirements(drivetrain);
    setName("Swerve/CalibrateModuleGeometry");
  }

  @Override
  public void initialize() {
    counterClockwise.reset();
    clockwise.reset();
    clearPreviousSample();
    resultValid = false;
    commandStartSeconds = Timer.getFPGATimestamp();
    lastTelemetrySeconds = Double.NEGATIVE_INFINITY;

    if (!DriverStation.isTeleopEnabled()) {
      fail("A calibracao so pode iniciar com o Teleop habilitado.");
      return;
    }

    status = "Rampa anti-horaria";
    beginPhase(Phase.RAMP_CCW, commandStartSeconds);
    publishTelemetry(commandStartSeconds, true);
    DriverStation.reportWarning(
        "Calibracao do swerve iniciada: mantenha a area livre. POV esquerdo ou A cancela.",
        false);
  }

  @Override
  public void execute() {
    double nowSeconds = Timer.getFPGATimestamp();
    if (nowSeconds - commandStartSeconds > ConfigLocalization.GEOMETRY_CALIBRATION_TIMEOUT_SECONDS) {
      fail("Tempo limite atingido. Verifique se o Pigeon e os modulos estao respondendo.");
      return;
    }

    switch (phase) {
      case RAMP_CCW -> runRamp(nowSeconds, 1.0, Phase.RECORD_CCW);
      case RECORD_CCW -> runRecording(nowSeconds, 1.0, counterClockwise, Phase.SETTLE);
      case SETTLE -> runSettle(nowSeconds);
      case RAMP_CW -> runRamp(nowSeconds, -1.0, Phase.RECORD_CW);
      case RECORD_CW -> runRecording(nowSeconds, -1.0, clockwise, Phase.DONE);
      case DONE, FAILED, CANCELLED -> commandBrake();
      case IDLE -> fail("Estado interno invalido: a calibracao nao foi inicializada.");
    }

    publishTelemetry(nowSeconds, false);
  }

  private void runRamp(double nowSeconds, double direction, Phase nextPhase) {
    double normalizedTime =
        MathUtil.clamp(
            (nowSeconds - phaseStartSeconds)
                / ConfigLocalization.GEOMETRY_CALIBRATION_RAMP_SECONDS,
            0.0,
            1.0);
    double smoothStep = normalizedTime * normalizedTime * (3.0 - 2.0 * normalizedTime);
    commandRotation(
        direction
            * ConfigLocalization.GEOMETRY_CALIBRATION_OMEGA_RADIANS_PER_SECOND
            * smoothStep);

    if (normalizedTime >= 1.0) {
      status = direction > 0.0 ? "Medindo anti-horario" : "Medindo horario";
      beginPhase(nextPhase, nowSeconds);
    }
  }

  private void runRecording(
      double nowSeconds,
      double direction,
      DirectionAccumulator accumulator,
      Phase nextPhase) {
    commandRotation(
        direction * ConfigLocalization.GEOMETRY_CALIBRATION_OMEGA_RADIANS_PER_SECOND);

    if (!recordCurrentSample(accumulator)) {
      return;
    }

    double targetRadians = targetRadiansPerDirection();
    if (accumulator.absoluteYawRadians < targetRadians) {
      return;
    }

    if (direction > 0.0 && accumulator.signedYawRadians < targetRadians * 0.75) {
      fail("O sinal do RawHeading esta invertido durante o giro anti-horario.");
      return;
    }
    if (direction < 0.0 && accumulator.signedYawRadians > -targetRadians * 0.75) {
      fail("O sinal do RawHeading esta invertido durante o giro horario.");
      return;
    }

    if (nextPhase == Phase.SETTLE) {
      status = "Parando antes do giro horario";
      beginPhase(Phase.SETTLE, nowSeconds);
      commandBrake();
      return;
    }

    commandBrake();
    calculateAndPublishResult();
  }

  private void runSettle(double nowSeconds) {
    commandBrake();
    if (nowSeconds - phaseStartSeconds < ConfigLocalization.GEOMETRY_CALIBRATION_SETTLE_SECONDS) {
      return;
    }

    status = "Rampa horaria";
    beginPhase(Phase.RAMP_CW, nowSeconds);
  }

  private boolean recordCurrentSample(DirectionAccumulator accumulator) {
    var state = drivetrain.getStateCopy();
    if (state.ModulePositions == null || state.ModulePositions.length != MODULE_COUNT) {
      fail("Esperados quatro ModulePositions no estado do drivetrain.");
      return false;
    }
    if (state.RawHeading == null) {
      fail("RawHeading nao esta disponivel no estado do drivetrain.");
      return false;
    }

    if (previousPositions == null || previousRawHeading == null) {
      previousPositions = copyPositions(state.ModulePositions);
      previousRawHeading = state.RawHeading;
      return true;
    }

    double deltaTheta = state.RawHeading.minus(previousRawHeading).getRadians();
    double[] deltaX = new double[MODULE_COUNT];
    double[] deltaY = new double[MODULE_COUNT];
    double meanDeltaX = 0.0;
    double meanDeltaY = 0.0;

    for (int moduleIndex = 0; moduleIndex < MODULE_COUNT; moduleIndex++) {
      SwerveModulePosition current = state.ModulePositions[moduleIndex];
      SwerveModulePosition previous = previousPositions[moduleIndex];
      double deltaDistance = current.distanceMeters - previous.distanceMeters;
      double angleRadians = current.angle.getRadians();
      if (!Double.isFinite(deltaDistance) || !Double.isFinite(angleRadians)) {
        fail("Um modulo retornou distancia ou angulo nao finito.");
        return false;
      }

      deltaX[moduleIndex] = deltaDistance * Math.cos(angleRadians);
      deltaY[moduleIndex] = deltaDistance * Math.sin(angleRadians);
      meanDeltaX += deltaX[moduleIndex];
      meanDeltaY += deltaY[moduleIndex];
    }
    meanDeltaX /= MODULE_COUNT;
    meanDeltaY /= MODULE_COUNT;

    previousPositions = copyPositions(state.ModulePositions);
    previousRawHeading = state.RawHeading;

    if (!Double.isFinite(deltaTheta)) {
      fail("O Pigeon retornou um delta angular nao finito.");
      return false;
    }
    if (Math.abs(deltaTheta)
        < ConfigLocalization.GEOMETRY_CALIBRATION_MIN_DELTA_THETA_RADIANS) {
      return true;
    }

    accumulator.add(deltaTheta, deltaX, deltaY, meanDeltaX, meanDeltaY);
    return true;
  }

  private void calculateAndPublishResult() {
    if (counterClockwise.sampleCount
            < ConfigLocalization.GEOMETRY_CALIBRATION_MIN_SAMPLES_PER_DIRECTION
        || clockwise.sampleCount
            < ConfigLocalization.GEOMETRY_CALIBRATION_MIN_SAMPLES_PER_DIRECTION
        || counterClockwise.denominator <= 0.0
        || clockwise.denominator <= 0.0) {
      fail("Poucas amostras validas. Repita o teste e verifique o sinal do Pigeon.");
      return;
    }

    double[] ccwX = counterClockwise.moduleX();
    double[] ccwY = counterClockwise.moduleY();
    double[] cwX = clockwise.moduleX();
    double[] cwY = clockwise.moduleY();
    double[] combinedX = new double[MODULE_COUNT];
    double[] combinedY = new double[MODULE_COUNT];
    double combinedDenominator = counterClockwise.denominator + clockwise.denominator;

    double maximumDirectionDifference = 0.0;
    double halfWheelbaseMeters = 0.0;
    double halfTrackwidthMeters = 0.0;
    boolean signsValid = true;

    for (int moduleIndex = 0; moduleIndex < MODULE_COUNT; moduleIndex++) {
      combinedX[moduleIndex] =
          (counterClockwise.xNumerator[moduleIndex]
                  + clockwise.xNumerator[moduleIndex])
              / combinedDenominator;
      combinedY[moduleIndex] =
          (counterClockwise.yNumerator[moduleIndex]
                  + clockwise.yNumerator[moduleIndex])
              / combinedDenominator;
      maximumDirectionDifference =
          Math.max(
              maximumDirectionDifference,
              Math.hypot(ccwX[moduleIndex] - cwX[moduleIndex], ccwY[moduleIndex] - cwY[moduleIndex]));
      halfWheelbaseMeters += Math.abs(combinedX[moduleIndex]);
      halfTrackwidthMeters += Math.abs(combinedY[moduleIndex]);
      signsValid &= Math.signum(combinedX[moduleIndex]) == EXPECTED_X_SIGNS[moduleIndex];
      signsValid &= Math.signum(combinedY[moduleIndex]) == EXPECTED_Y_SIGNS[moduleIndex];
    }
    halfWheelbaseMeters /= MODULE_COUNT;
    halfTrackwidthMeters /= MODULE_COUNT;

    double maximumSymmetryError = 0.0;
    for (int moduleIndex = 0; moduleIndex < MODULE_COUNT; moduleIndex++) {
      maximumSymmetryError =
          Math.max(
              maximumSymmetryError,
              Math.abs(Math.abs(combinedX[moduleIndex]) - halfWheelbaseMeters));
      maximumSymmetryError =
          Math.max(
              maximumSymmetryError,
              Math.abs(Math.abs(combinedY[moduleIndex]) - halfTrackwidthMeters));
    }

    resultValid =
        signsValid
            && maximumDirectionDifference
                <= ConfigLocalization.GEOMETRY_CALIBRATION_MAX_DIRECTION_DIFFERENCE_METERS
            && maximumSymmetryError
                <= ConfigLocalization.GEOMETRY_CALIBRATION_MAX_SYMMETRY_ERROR_METERS;

    double moduleRadiusMeters = Math.hypot(halfWheelbaseMeters, halfTrackwidthMeters);
    String recommendedConstants =
        String.format(
            Locale.ROOT,
            "private static final Distance HALF_WHEELBASE = Inches.of(%.4f);%n"
                + "private static final Distance HALF_TRACKWIDTH = Inches.of(%.4f);",
            halfWheelbaseMeters * METERS_TO_INCHES,
            halfTrackwidthMeters * METERS_TO_INCHES);

    SmartDashboard.putBoolean("SwerveGeometry/Valid", resultValid);
    SmartDashboard.putNumber("SwerveGeometry/HalfWheelbaseMeters", halfWheelbaseMeters);
    SmartDashboard.putNumber(
        "SwerveGeometry/HalfWheelbaseInches", halfWheelbaseMeters * METERS_TO_INCHES);
    SmartDashboard.putNumber("SwerveGeometry/HalfTrackwidthMeters", halfTrackwidthMeters);
    SmartDashboard.putNumber(
        "SwerveGeometry/HalfTrackwidthInches", halfTrackwidthMeters * METERS_TO_INCHES);
    SmartDashboard.putNumber("SwerveGeometry/ModuleRadiusMeters", moduleRadiusMeters);
    SmartDashboard.putNumber(
        "SwerveGeometry/DirectionMaxDifferenceMeters", maximumDirectionDifference);
    SmartDashboard.putNumber("SwerveGeometry/SymmetryMaxErrorMeters", maximumSymmetryError);
    SmartDashboard.putNumber(
        "SwerveGeometry/CCWEstimatedDriftMeters", counterClockwise.driftMeters());
    SmartDashboard.putNumber(
        "SwerveGeometry/CWEstimatedDriftMeters", clockwise.driftMeters());
    SmartDashboard.putString("SwerveGeometry/RecommendedConstants", recommendedConstants);

    for (int moduleIndex = 0; moduleIndex < MODULE_COUNT; moduleIndex++) {
      String prefix = "SwerveGeometry/" + MODULE_NAMES[moduleIndex] + "/";
      SmartDashboard.putNumber(prefix + "XMeters", combinedX[moduleIndex]);
      SmartDashboard.putNumber(prefix + "YMeters", combinedY[moduleIndex]);
      SmartDashboard.putNumber(prefix + "XInches", combinedX[moduleIndex] * METERS_TO_INCHES);
      SmartDashboard.putNumber(prefix + "YInches", combinedY[moduleIndex] * METERS_TO_INCHES);
    }

    if (!resultValid) {
      fail(
          String.format(
              Locale.ROOT,
              "Resultado inconsistente (sentidos %.1f mm, simetria %.1f mm, sinais %s). Nao copie.",
              maximumDirectionDifference * 1000.0,
              maximumSymmetryError * 1000.0,
              signsValid ? "corretos" : "incorretos"));
      return;
    }

    phase = Phase.DONE;
    status = "Concluido: copie RecommendedConstants";
    SmartDashboard.putString("SwerveGeometry/Status", status);
    SmartDashboard.putString("SwerveGeometry/Phase", phase.name());
    DriverStation.reportWarning(
        String.format(
            Locale.ROOT,
            "Calibracao concluida: HALF_WHEELBASE=%.4f in; HALF_TRACKWIDTH=%.4f in",
            halfWheelbaseMeters * METERS_TO_INCHES,
            halfTrackwidthMeters * METERS_TO_INCHES),
        false);
  }

  private void beginPhase(Phase newPhase, double nowSeconds) {
    phase = newPhase;
    phaseStartSeconds = nowSeconds;
    clearPreviousSample();
    publishTelemetry(nowSeconds, true);
  }

  private void clearPreviousSample() {
    previousPositions = null;
    previousRawHeading = null;
  }

  private void commandRotation(double omegaRadiansPerSecond) {
    requestedSpeedsObserver.accept(new ChassisSpeeds(0.0, 0.0, omegaRadiansPerSecond));
    drivetrain.setControl(
        rotateRequest
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(omegaRadiansPerSecond));
  }

  private void commandBrake() {
    requestedSpeedsObserver.accept(new ChassisSpeeds());
    drivetrain.setControl(brake);
  }

  private void fail(String message) {
    phase = Phase.FAILED;
    status = message;
    resultValid = false;
    commandBrake();
    SmartDashboard.putBoolean("SwerveGeometry/Valid", false);
    SmartDashboard.putString("SwerveGeometry/Status", status);
    SmartDashboard.putString("SwerveGeometry/Phase", phase.name());
    DriverStation.reportError("Calibracao do swerve: " + message, false);
  }

  private void publishTelemetry(double nowSeconds, boolean force) {
    if (!force && nowSeconds - lastTelemetrySeconds < TELEMETRY_PERIOD_SECONDS) {
      return;
    }
    lastTelemetrySeconds = nowSeconds;

    double targetRadians = targetRadiansPerDirection();
    double progress;
    if (phase == Phase.RECORD_CCW) {
      progress = 0.50 * MathUtil.clamp(counterClockwise.absoluteYawRadians / targetRadians, 0.0, 1.0);
    } else if (phase == Phase.SETTLE || phase == Phase.RAMP_CW) {
      progress = 0.50;
    } else if (phase == Phase.RECORD_CW) {
      progress =
          0.50
              + 0.50
                  * MathUtil.clamp(clockwise.absoluteYawRadians / targetRadians, 0.0, 1.0);
    } else if (phase == Phase.DONE) {
      progress = 1.0;
    } else {
      progress = 0.0;
    }

    SmartDashboard.putBoolean("SwerveGeometry/Running", !isFinished());
    SmartDashboard.putBoolean("SwerveGeometry/Valid", resultValid);
    SmartDashboard.putString("SwerveGeometry/Phase", phase.name());
    SmartDashboard.putString("SwerveGeometry/Status", status);
    SmartDashboard.putNumber("SwerveGeometry/ProgressPercent", progress * 100.0);
    SmartDashboard.putNumber("SwerveGeometry/CCWSamples", counterClockwise.sampleCount);
    SmartDashboard.putNumber("SwerveGeometry/CWSamples", clockwise.sampleCount);
    SmartDashboard.putNumber(
        "SwerveGeometry/CCWTurns", counterClockwise.absoluteYawRadians / (2.0 * Math.PI));
    SmartDashboard.putNumber(
        "SwerveGeometry/CWTurns", clockwise.absoluteYawRadians / (2.0 * Math.PI));
  }

  private static SwerveModulePosition[] copyPositions(SwerveModulePosition[] source) {
    SwerveModulePosition[] copy = new SwerveModulePosition[source.length];
    for (int index = 0; index < source.length; index++) {
      copy[index] = new SwerveModulePosition(source[index].distanceMeters, source[index].angle);
    }
    return copy;
  }

  private static double targetRadiansPerDirection() {
    return ConfigLocalization.GEOMETRY_CALIBRATION_TURNS_PER_DIRECTION * 2.0 * Math.PI;
  }

  @Override
  public boolean isFinished() {
    return phase == Phase.DONE || phase == Phase.FAILED || phase == Phase.CANCELLED;
  }

  @Override
  public void end(boolean interrupted) {
    commandBrake();
    if (interrupted && phase != Phase.DONE && phase != Phase.FAILED) {
      phase = Phase.CANCELLED;
      status = "Cancelado pelo piloto";
      resultValid = false;
      SmartDashboard.putBoolean("SwerveGeometry/Valid", false);
      SmartDashboard.putString("SwerveGeometry/Phase", phase.name());
      SmartDashboard.putString("SwerveGeometry/Status", status);
      DriverStation.reportWarning("Calibracao do swerve cancelada.", false);
    }
    SmartDashboard.putBoolean("SwerveGeometry/Running", false);
  }

  private static final class DirectionAccumulator {
    final double[] xNumerator = new double[MODULE_COUNT];
    final double[] yNumerator = new double[MODULE_COUNT];
    double denominator;
    double absoluteYawRadians;
    double signedYawRadians;
    int sampleCount;
    Pose2d driftPose = new Pose2d();

    void reset() {
      for (int moduleIndex = 0; moduleIndex < MODULE_COUNT; moduleIndex++) {
        xNumerator[moduleIndex] = 0.0;
        yNumerator[moduleIndex] = 0.0;
      }
      denominator = 0.0;
      absoluteYawRadians = 0.0;
      signedYawRadians = 0.0;
      sampleCount = 0;
      driftPose = new Pose2d();
    }

    void add(
        double deltaTheta,
        double[] deltaX,
        double[] deltaY,
        double meanDeltaX,
        double meanDeltaY) {
      denominator += deltaTheta * deltaTheta;
      absoluteYawRadians += Math.abs(deltaTheta);
      signedYawRadians += deltaTheta;
      sampleCount++;
      driftPose = driftPose.exp(new Twist2d(meanDeltaX, meanDeltaY, deltaTheta));

      for (int moduleIndex = 0; moduleIndex < MODULE_COUNT; moduleIndex++) {
        // dx = -y*dtheta e dy = x*dtheta para um corpo rigido girando em torno do centro.
        xNumerator[moduleIndex] += deltaTheta * (deltaY[moduleIndex] - meanDeltaY);
        yNumerator[moduleIndex] += -deltaTheta * (deltaX[moduleIndex] - meanDeltaX);
      }
    }

    double[] moduleX() {
      double[] result = new double[MODULE_COUNT];
      for (int moduleIndex = 0; moduleIndex < MODULE_COUNT; moduleIndex++) {
        result[moduleIndex] = xNumerator[moduleIndex] / denominator;
      }
      return result;
    }

    double[] moduleY() {
      double[] result = new double[MODULE_COUNT];
      for (int moduleIndex = 0; moduleIndex < MODULE_COUNT; moduleIndex++) {
        result[moduleIndex] = yNumerator[moduleIndex] / denominator;
      }
      return result;
    }

    double driftMeters() {
      return driftPose.getTranslation().getNorm();
    }
  }
}
