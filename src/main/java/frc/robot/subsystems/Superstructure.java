package frc.robot.subsystems;

import java.util.Objects;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ConfigAutomaticAim;
import frc.robot.config.ConfigSwerve;
import frc.robot.config.ConfigSwerve.DriveProfile;
import frc.robot.config.ConfigVision;

/**
 * Coordena o estado global do robo e a selecao automatica do alvo de campo.
 *
 * <p>Ao iniciar a mira automatica, a Superstructure escolhe o HUB da propria alianca se o centro
 * do robo estiver dentro da ALLIANCE ZONE. Fora dela, escolhe o ponto de passe inferior ou
 * superior mais proximo, sempre pertencente a propria alianca.
 *
 * <p>O PID controla somente o heading. A translacao continua sob controle do piloto. Depois de
 * alinhado, o PID permanece ativo para resistir a perturbacoes; nenhuma trava X e aplicada.
 */
public class Superstructure extends SubsystemBase {

  public enum Goal {
    IDLE("Ocioso"),
    COLLECTING("Coletando"),
    HOLDING_GAME_PIECE("Com peca"),
    AIMING("Mirando"),
    SCORING("Pontuando"),
    CLIMBING("Escalando");

    private final String displayName;

    Goal(String displayName) {
      this.displayName = displayName;
    }

    public String displayName() {
      return displayName;
    }
  }

  public enum DriveMode {
    NORMAL("Normal"),
    SLOW("Lento");

    private final String displayName;

    DriveMode(String displayName) {
      this.displayName = displayName;
    }

    public String displayName() {
      return displayName;
    }
  }

  /** Acao de campo atualmente escolhida pelo seletor automatico. */
  public enum AutomaticAimAction {
    NONE("Nenhuma"),
    HUB("HUB"),
    PASS_LOWER("Passe inferior"),
    PASS_UPPER("Passe superior"),
    UNAVAILABLE("Indisponivel");

    private final String displayName;

    AutomaticAimAction(String displayName) {
      this.displayName = displayName;
    }

    public String displayName() {
      return displayName;
    }

    public boolean isPass() {
      return this == PASS_LOWER || this == PASS_UPPER;
    }
  }

  public enum AimPhase {
    INACTIVE,
    WAITING_FOR_ALLIANCE,
    INVALID_TARGET,
    TRACKING,
    ALIGNED
  }

  private final Supplier<Pose2d> poseSupplier;
  private final DoubleSupplier measuredOmegaSupplier;
  private final PIDController headingController =
      new PIDController(
          ConfigAutomaticAim.HEADING_KP,
          ConfigAutomaticAim.HEADING_KI,
          ConfigAutomaticAim.HEADING_KD);

  private Goal currentGoal = Goal.IDLE;
  private Goal goalBeforeAutomaticAim = Goal.IDLE;
  private DriveMode currentDriveMode = DriveMode.NORMAL;

  private boolean automaticAimRequested;
  private boolean targetGeometryValid;
  private boolean robotCenterInsideAllianceZone;
  private AutomaticAimAction automaticAimAction = AutomaticAimAction.NONE;
  private AimPhase aimPhase = AimPhase.INACTIVE;
  private Alliance selectedAlliance;
  private Translation2d selectedTarget = Translation2d.kZero;
  private Rotation2d selectedHeading = Rotation2d.kZero;
  private double headingErrorRadians = Double.NaN;
  private double alignmentStableSinceSeconds = Double.NaN;

  private double lastPublishedSeconds = Double.NEGATIVE_INFINITY;

  /** Construtor usado pelo RobotContainer real. */
  public Superstructure(
      Supplier<Pose2d> poseSupplier,
      DoubleSupplier measuredOmegaSupplier) {
    this.poseSupplier = Objects.requireNonNull(poseSupplier, "poseSupplier");
    this.measuredOmegaSupplier =
        Objects.requireNonNull(measuredOmegaSupplier, "measuredOmegaSupplier");
    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /** Mantido para testes unitarios ou simulacoes que ainda nao fornecem pose. */
  public Superstructure() {
    this(Pose2d::new, () -> 0.0);
  }

  public Goal getGoal() {
    return currentGoal;
  }

  public DriveMode getDriveMode() {
    return currentDriveMode;
  }

  public boolean isSlowModeEnabled() {
    return currentDriveMode == DriveMode.SLOW;
  }

  public DriveProfile getDriveProfile() {
    return switch (currentGoal) {
      case COLLECTING -> ConfigSwerve.COLLECTING_PROFILE;
      case HOLDING_GAME_PIECE -> ConfigSwerve.HOLDING_PROFILE;
      case AIMING, SCORING -> ConfigSwerve.SCORING_PROFILE;
      case CLIMBING -> ConfigSwerve.CLIMBING_PROFILE;
      case IDLE -> ConfigSwerve.NORMAL_PROFILE;
    };
  }

  public double getDriveTranslationScale() {
    double goalScale = getDriveProfile().translationScale();
    return isSlowModeEnabled()
        ? goalScale * ConfigSwerve.SLOW_MODE_TRANSLATION_MULTIPLIER
        : goalScale;
  }

  public double getDriveRotationScale() {
    double goalScale = getDriveProfile().rotationScale();
    return isSlowModeEnabled()
        ? goalScale * ConfigSwerve.SLOW_MODE_ROTATION_MULTIPLIER
        : goalScale;
  }

  public Command toggleSlowModeCommand() {
    return runOnce(
            () ->
                currentDriveMode =
                    currentDriveMode == DriveMode.NORMAL ? DriveMode.SLOW : DriveMode.NORMAL)
        .withName("Superstructure/ToggleSlowMode");
  }

  public Command setGoalCommand(Goal goal) {
    Objects.requireNonNull(goal, "goal");
    return runOnce(() -> setGoal(goal))
        .withName("Superstructure/SetGoal/" + goal.name());
  }

  public Command holdGoalCommand(Goal activeGoal, Goal endGoal) {
    Objects.requireNonNull(activeGoal, "activeGoal");
    Objects.requireNonNull(endGoal, "endGoal");

    return startEnd(
            () -> setGoal(activeGoal),
            () -> setGoal(endGoal))
        .withName("Superstructure/Hold/" + activeGoal.name());
  }

  public Command runAction(Goal activeGoal, Command mechanismCommand, Goal endGoal) {
    Objects.requireNonNull(mechanismCommand, "mechanismCommand");

    return Commands.sequence(
            setGoalCommand(activeGoal),
            mechanismCommand)
        .finallyDo(interrupted -> setGoal(endGoal))
        .withName("Superstructure/Action/" + activeGoal.name());
  }

  public Command collectCommand(Command intakeCommand) {
    return runAction(Goal.COLLECTING, intakeCommand, Goal.HOLDING_GAME_PIECE);
  }

  public Command scoreCommand(Command scoringCommand) {
    return runAction(Goal.SCORING, scoringCommand, Goal.IDLE);
  }

  public Command climbCommand(Command climbingCommand) {
    return runAction(Goal.CLIMBING, climbingCommand, Goal.CLIMBING);
  }

  /** Um clique inicia a selecao HUB/passe e a mira continua ate concluir ou cancelar. */
  public Command startAutomaticAimCommand() {
    return runOnce(this::startAutomaticAim)
        .withName("Superstructure/AutomaticAim/Start");
  }

  /** O botao B deve chamar este comando. */
  public Command cancelAutomaticAimCommand() {
    return runOnce(this::cancelAutomaticAim)
        .withName("Superstructure/AutomaticAim/Cancel");
  }

  /**
   * O shooter/passador deve chamar este comando depois que o sensor confirmar que a bola saiu.
   * Enquanto ele nao existe, a mira permanece ativa ate o piloto apertar B.
   */
  public Command completeAutomaticActionCommand() {
    return runOnce(this::completeAutomaticAction)
        .withName("Superstructure/AutomaticAim/Complete");
  }

  public void startAutomaticAim() {
    if (!automaticAimRequested) {
      goalBeforeAutomaticAim = currentGoal == Goal.AIMING ? Goal.IDLE : currentGoal;
    }

    automaticAimRequested = true;
    currentGoal = Goal.AIMING;
    resetAlignmentTracking();
    updateAutomaticTarget();
  }

  public void cancelAutomaticAim() {
    stopAutomaticAim();
    currentGoal = goalBeforeAutomaticAim == Goal.AIMING ? Goal.IDLE : goalBeforeAutomaticAim;
  }

  public void completeAutomaticAction() {
    stopAutomaticAim();
    currentGoal = Goal.IDLE;
  }

  public boolean isAutomaticAimActive() {
    return automaticAimRequested;
  }

  public AutomaticAimAction getAutomaticAimAction() {
    return automaticAimAction;
  }

  public AimPhase getAimPhase() {
    return aimPhase;
  }

  public boolean isAimAligned() {
    return automaticAimRequested && aimPhase == AimPhase.ALIGNED;
  }

  public Translation2d getSelectedTarget() {
    return selectedTarget;
  }

  public Rotation2d getSelectedHeading() {
    return selectedHeading;
  }

  /**
   * Retorna o omega que substitui o joystick direito durante a mira automatica.
   *
   * <p>Este metodo deve ser chamado uma vez por loop pelo default command do drivetrain. Mesmo
   * depois de ALIGNED, o PID continua calculando para manter o heading.
   */
  public double calculateAutomaticAimOmega(double maximumOmegaRadiansPerSecond) {
    if (!automaticAimRequested) {
      return 0.0;
    }

    updateAutomaticTarget();
    if (!targetGeometryValid) {
      return 0.0;
    }

    Pose2d pose = poseSupplier.get();
    double currentHeading = pose.getRotation().getRadians();
    double targetHeading = selectedHeading.getRadians();
    headingErrorRadians = MathUtil.angleModulus(targetHeading - currentHeading);

    double output = headingController.calculate(currentHeading, targetHeading);
    updateAlignmentState();

    double limit = Math.abs(maximumOmegaRadiansPerSecond);
    return MathUtil.clamp(output, -limit, limit);
  }

  /** Gate de seguranca que o futuro comando do shooter deve consultar antes de alimentar. */
  public boolean canShootHub() {
    updateAutomaticTarget();
    return automaticAimRequested
        && automaticAimAction == AutomaticAimAction.HUB
        && robotCenterInsideAllianceZone
        && aimPhase == AimPhase.ALIGNED;
  }

  /** Gate que o futuro comando de passe deve consultar antes de alimentar. */
  public boolean canPassToAlliance() {
    updateAutomaticTarget();
    return automaticAimRequested
        && automaticAimAction.isPass()
        && !robotCenterInsideAllianceZone
        && aimPhase == AimPhase.ALIGNED;
  }

  /** True somente quando a acao automaticamente selecionada esta pronta para liberar a bola. */
  public boolean canReleaseGamePiece() {
    return canShootHub() || canPassToAlliance();
  }

  /**
   * Verificacao conservadora: exige que o centro do robo esteja dentro da ALLIANCE ZONE.
   * Isso e mais restritivo que a regra de BUMPERS parcialmente dentro, mas evita tiro ilegal.
   */
  public boolean isRobotCenterInsideOwnAllianceZone() {
    updateAutomaticTarget();
    return robotCenterInsideAllianceZone;
  }

  private void setGoal(Goal newGoal) {
    Objects.requireNonNull(newGoal, "newGoal");
    if (newGoal != Goal.AIMING && automaticAimRequested) {
      stopAutomaticAim();
    }
    currentGoal = newGoal;
  }

  private void stopAutomaticAim() {
    automaticAimRequested = false;
    targetGeometryValid = false;
    robotCenterInsideAllianceZone = false;
    automaticAimAction = AutomaticAimAction.NONE;
    aimPhase = AimPhase.INACTIVE;
    selectedAlliance = null;
    selectedTarget = Translation2d.kZero;
    selectedHeading = Rotation2d.kZero;
    headingErrorRadians = Double.NaN;
    resetAlignmentTracking();
  }

  private void updateAutomaticTarget() {
    if (!automaticAimRequested) {
      return;
    }

    Alliance alliance = DriverStation.getAlliance().orElse(null);
    if (alliance == null) {
      selectedAlliance = null;
      targetGeometryValid = false;
      robotCenterInsideAllianceZone = false;
      automaticAimAction = AutomaticAimAction.UNAVAILABLE;
      aimPhase = AimPhase.WAITING_FOR_ALLIANCE;
      headingErrorRadians = Double.NaN;
      resetAlignmentTracking();
      return;
    }

    Pose2d pose = poseSupplier.get();
    boolean insideZone = isInsideOwnAllianceZone(pose, alliance);

    AutomaticAimAction newAction;
    Translation2d newTarget;
    if (insideZone) {
      newAction = AutomaticAimAction.HUB;
      newTarget =
          alliance == Alliance.Blue
              ? ConfigAutomaticAim.BLUE_HUB
              : ConfigAutomaticAim.RED_HUB;
    } else {
      newAction = choosePassAction(pose, alliance);
      newTarget = getPassTarget(alliance, newAction);
    }

    boolean targetChanged =
        alliance != selectedAlliance
            || newAction != automaticAimAction
            || newTarget.getDistance(selectedTarget) > 1e-9;

    selectedAlliance = alliance;
    robotCenterInsideAllianceZone = insideZone;
    automaticAimAction = newAction;
    selectedTarget = newTarget;

    Translation2d delta = selectedTarget.minus(pose.getTranslation());
    targetGeometryValid =
        Double.isFinite(delta.getX())
            && Double.isFinite(delta.getY())
            && delta.getNorm() >= ConfigAutomaticAim.MIN_TARGET_DISTANCE_METERS;

    if (!targetGeometryValid) {
      aimPhase = AimPhase.INVALID_TARGET;
      headingErrorRadians = Double.NaN;
      resetAlignmentTracking();
      return;
    }

    selectedHeading =
        Rotation2d.fromRadians(
            Math.atan2(delta.getY(), delta.getX())
                + ConfigAutomaticAim.SHOOTER_YAW_OFFSET_RADIANS);

    if (targetChanged) {
      headingController.reset();
      resetAlignmentTracking();
      aimPhase = AimPhase.TRACKING;
    } else if (aimPhase != AimPhase.ALIGNED) {
      aimPhase = AimPhase.TRACKING;
    }
  }

  private AutomaticAimAction choosePassAction(Pose2d pose, Alliance alliance) {
    Translation2d lower =
        alliance == Alliance.Blue
            ? ConfigAutomaticAim.BLUE_PASS_LOWER
            : ConfigAutomaticAim.RED_PASS_LOWER;
    Translation2d upper =
        alliance == Alliance.Blue
            ? ConfigAutomaticAim.BLUE_PASS_UPPER
            : ConfigAutomaticAim.RED_PASS_UPPER;

    double lowerDistance = pose.getTranslation().getDistance(lower);
    double upperDistance = pose.getTranslation().getDistance(upper);

    if (alliance == selectedAlliance && automaticAimAction.isPass()) {
      double currentDistance =
          automaticAimAction == AutomaticAimAction.PASS_LOWER
              ? lowerDistance
              : upperDistance;
      double otherDistance =
          automaticAimAction == AutomaticAimAction.PASS_LOWER
              ? upperDistance
              : lowerDistance;

      if (otherDistance + ConfigAutomaticAim.PASS_TARGET_SWITCH_HYSTERESIS_METERS
          >= currentDistance) {
        return automaticAimAction;
      }
    }

    return lowerDistance <= upperDistance
        ? AutomaticAimAction.PASS_LOWER
        : AutomaticAimAction.PASS_UPPER;
  }

  private static Translation2d getPassTarget(
      Alliance alliance,
      AutomaticAimAction action) {
    if (alliance == Alliance.Blue) {
      return action == AutomaticAimAction.PASS_UPPER
          ? ConfigAutomaticAim.BLUE_PASS_UPPER
          : ConfigAutomaticAim.BLUE_PASS_LOWER;
    }

    return action == AutomaticAimAction.PASS_UPPER
        ? ConfigAutomaticAim.RED_PASS_UPPER
        : ConfigAutomaticAim.RED_PASS_LOWER;
  }

  private static boolean isInsideOwnAllianceZone(Pose2d pose, Alliance alliance) {
    double x = pose.getX();
    double y = pose.getY();
    double fieldWidth = ConfigVision.FIELD_LAYOUT.getFieldWidth();
    if (!Double.isFinite(x)
        || !Double.isFinite(y)
        || y < 0.0
        || y > fieldWidth) {
      return false;
    }

    if (alliance == Alliance.Blue) {
      return x >= 0.0 && x <= ConfigAutomaticAim.ALLIANCE_ZONE_DEPTH_METERS;
    }

    double fieldLength = ConfigVision.FIELD_LAYOUT.getFieldLength();
    double redZoneMinimumX = fieldLength - ConfigAutomaticAim.ALLIANCE_ZONE_DEPTH_METERS;
    return x >= redZoneMinimumX && x <= fieldLength;
  }

  private void updateAlignmentState() {
    double measuredOmega = measuredOmegaSupplier.getAsDouble();
    boolean withinTolerance =
        Double.isFinite(headingErrorRadians)
            && Math.abs(headingErrorRadians)
                <= ConfigAutomaticAim.HEADING_TOLERANCE_RADIANS
            && Double.isFinite(measuredOmega)
            && Math.abs(measuredOmega)
                <= ConfigAutomaticAim.MAX_ALIGNED_OMEGA_RADIANS_PER_SECOND;

    double nowSeconds = Timer.getFPGATimestamp();
    if (!withinTolerance) {
      alignmentStableSinceSeconds = Double.NaN;
      aimPhase = AimPhase.TRACKING;
      return;
    }

    if (!Double.isFinite(alignmentStableSinceSeconds)) {
      alignmentStableSinceSeconds = nowSeconds;
    }

    aimPhase =
        nowSeconds - alignmentStableSinceSeconds
                >= ConfigAutomaticAim.ALIGNED_STABLE_SECONDS
            ? AimPhase.ALIGNED
            : AimPhase.TRACKING;
  }

  private void resetAlignmentTracking() {
    alignmentStableSinceSeconds = Double.NaN;
  }

  @Override
  public void periodic() {
    if (automaticAimRequested) {
      updateAutomaticTarget();
    }

    double nowSeconds = Timer.getFPGATimestamp();
    if (nowSeconds - lastPublishedSeconds < 0.10) {
      return;
    }
    lastPublishedSeconds = nowSeconds;

    DriveProfile profile = getDriveProfile();
    SmartDashboard.putString("Superstructure/Goal", currentGoal.displayName());
    SmartDashboard.putString("Superstructure/DriveProfile", profile.name());
    SmartDashboard.putString("Superstructure/DriveMode", currentDriveMode.displayName());
    SmartDashboard.putBoolean("Superstructure/SlowModeEnabled", isSlowModeEnabled());
    SmartDashboard.putNumber(
        "Superstructure/TranslationScale", getDriveTranslationScale());
    SmartDashboard.putNumber("Superstructure/RotationScale", getDriveRotationScale());

    SmartDashboard.putBoolean("Superstructure/Aim/Requested", automaticAimRequested);
    SmartDashboard.putString(
        "Superstructure/Aim/Action", automaticAimAction.displayName());
    SmartDashboard.putString("Superstructure/Aim/Phase", aimPhase.name());
    SmartDashboard.putString(
        "Superstructure/Aim/Alliance",
        selectedAlliance == null ? "UNKNOWN" : selectedAlliance.name());
    SmartDashboard.putBoolean(
        "Superstructure/Aim/InsideOwnAllianceZone", robotCenterInsideAllianceZone);
    SmartDashboard.putBoolean("Superstructure/Aim/Aligned", isAimAligned());
    SmartDashboard.putBoolean("Superstructure/Aim/CanShootHub", canShootHub());
    SmartDashboard.putBoolean("Superstructure/Aim/CanPass", canPassToAlliance());
    SmartDashboard.putBoolean(
        "Superstructure/Aim/CanReleaseGamePiece", canReleaseGamePiece());
    SmartDashboard.putNumber("Superstructure/Aim/TargetX", selectedTarget.getX());
    SmartDashboard.putNumber("Superstructure/Aim/TargetY", selectedTarget.getY());
    SmartDashboard.putNumber(
        "Superstructure/Aim/TargetHeadingDeg", selectedHeading.getDegrees());
    SmartDashboard.putNumber(
        "Superstructure/Aim/HeadingErrorDeg",
        Double.isFinite(headingErrorRadians)
            ? Math.toDegrees(headingErrorRadians)
            : Double.NaN);
  }
}
