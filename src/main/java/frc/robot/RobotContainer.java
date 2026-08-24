package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.ConfigSwerve;
import frc.robot.control.SwerveHeadingZoneLock;
import frc.robot.generated.TunerConstants;
import frc.robot.localization.OdometryHealthMonitor;
import frc.robot.localization.SwerveGeometryCalibrationCommand;
import frc.robot.localization.SwerveStateSolver;
import frc.robot.localization.SwerveStateSolver.DriveState;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Goal;
import frc.robot.telemetry.Telemetry;
import frc.robot.util.PerformanceMonitor;
import frc.robot.vision.LimelightSimulation;
import frc.robot.vision.VisionSubsystem;

public final class RobotContainer {
  private final double maxSpeedMetersPerSecond =
      ConfigSwerve.TELEOP_MAX_SPEED.in(MetersPerSecond);
  private final double maxAngularRateRadiansPerSecond =
      ConfigSwerve.TELEOP_MAX_ANGULAR_RATE.in(RadiansPerSecond);

  private final CommandXboxController driverController = new CommandXboxController(0);

  private final SwerveRequest.FieldCentric fieldCentricDrive =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.Velocity);
  private final SwerveRequest.SwerveDriveBrake brake =
      new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final Superstructure superstructure =
      new Superstructure(
          () -> drivetrain.getState().Pose,
          () -> drivetrain.getState().Speeds.omegaRadiansPerSecond);
  private final OdometryHealthMonitor odometryHealth = new OdometryHealthMonitor(drivetrain);
  private final LimelightSimulation limelightSimulation =
      new LimelightSimulation(() -> drivetrain.getState().Pose);
  private final VisionSubsystem vision = new VisionSubsystem(drivetrain);
  private final PerformanceMonitor performanceMonitor = new PerformanceMonitor();

  private final SwerveStateSolver swerveStateSolver = new SwerveStateSolver();
  private final SwerveHeadingZoneLock headingZoneLock =
      new SwerveHeadingZoneLock(drivetrain);
  private final SwerveGeometryCalibrationCommand geometryCalibration =
      new SwerveGeometryCalibrationCommand(drivetrain, odometryHealth::setRequestedSpeeds);
  private DriveState lastPublishedDriveState;
  private final Telemetry telemetry = new Telemetry();
  private final SendableChooser<Command> autonomousChooser;

  public RobotContainer() {
    drivetrain.setPathMotionBlockedSupplier(odometryHealth::shouldHoldPosition);
    drivetrain.setRequestedSpeedsObserver(odometryHealth::setRequestedSpeeds);
    configureBindings();
    autonomousChooser = configureAutonomousChooser();
    drivetrain.registerTelemetry(telemetry::telemeterize);

    SmartDashboard.putBoolean(
        "Swerve/HardwareConfigured", TunerConstants.HARDWARE_CONFIGURED);
    SmartDashboard.putString(
        "Swerve/DriveControlMode",
        TunerConstants.USE_TORQUE_CURRENT_FOC
            ? "VelocityTorqueCurrentFOC"
            : "VelocityVoltage");
    SmartDashboard.putString(
        "Superstructure/Aim/Controls",
        "LT inicia | B cancela | RS reseta heading pela visao");
  }

  private void configureBindings() {
    Trigger automaticAimActive = new Trigger(superstructure::isAutomaticAimActive);
    Trigger automaticAimInactive = automaticAimActive.negate();

    // Convencao WPILib: +X para frente, +Y para a esquerda e giro anti-horario positivo.
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () -> {
              double translationScale = superstructure.getDriveTranslationScale();
              double rotationScale = superstructure.getDriveRotationScale();
              double velocityX =
                  -MathUtil.applyDeadband(
                          driverController.getLeftY(), ConfigSwerve.JOYSTICK_DEADBAND)
                      * maxSpeedMetersPerSecond
                      * translationScale;
              double velocityY =
                  -MathUtil.applyDeadband(
                          driverController.getLeftX(), ConfigSwerve.JOYSTICK_DEADBAND)
                      * maxSpeedMetersPerSecond
                      * translationScale;
              double manualRotationalRate =
                  -MathUtil.applyDeadband(
                          driverController.getRightX(), ConfigSwerve.JOYSTICK_DEADBAND)
                      * maxAngularRateRadiansPerSecond
                      * rotationScale;

              double maximumAutomaticOmega =
                  maxAngularRateRadiansPerSecond * rotationScale;
              double rotationalRate;

              // Prioridade: mira automatica > trava de angulo antiga > joystick direito.
              if (superstructure.isAutomaticAimActive()) {
                rotationalRate =
                    superstructure.calculateAutomaticAimOmega(maximumAutomaticOmega);
              } else if (headingZoneLock.isLockActive()) {
                rotationalRate =
                    headingZoneLock.calculateLockOmega(maximumAutomaticOmega);
              } else {
                rotationalRate = manualRotationalRate;
              }

              odometryHealth.setRequestedSpeeds(velocityX, velocityY, rotationalRate);
              DriveState driveState =
                  swerveStateSolver.solve(
                      superstructure.getGoal(), odometryHealth.shouldHoldPosition());
              if (driveState != lastPublishedDriveState) {
                lastPublishedDriveState = driveState;
                SmartDashboard.putString("Swerve/SolverState", driveState.name());
              }
              if (driveState != DriveState.NORMAL) {
                return brake;
              }

              // A mira nunca retorna SwerveDriveBrake: o PID apenas mantem o heading.
              return fieldCentricDrive
                  .withVelocityX(velocityX)
                  .withVelocityY(velocityY)
                  .withRotationalRate(rotationalRate);
            }));

    RobotModeTriggers.disabled()
        .whileTrue(
            drivetrain
                .applyRequest(() -> idle)
                .ignoringDisable(true));

    // Trava X manual/emergencial. Nao e usada automaticamente durante a mira.
    driverController
        .a()
        .whileTrue(
            drivetrain.applyRequest(
                () -> {
                  odometryHealth.setRequestedSpeeds(0.0, 0.0, 0.0);
                  return brake;
                }));

    // B agora e exclusivamente o cancelamento da mira automatica.
    automaticAimActive
        .and(driverController.b())
        .onTrue(superstructure.cancelAutomaticAimCommand());

    driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    driverController.leftStick().onTrue(superstructure.toggleSlowModeCommand());

    // Clique no stick direito: corrige somente o heading com MT1/MT2, sem alterar X/Y.
    driverController.rightStick().onTrue(vision.resetHeadingFromVisionCommand());

    // Segure Start para usar o HeadingZoneLock antigo, apenas fora da mira automatica.
    automaticAimInactive
        .and(driverController.start())
        .onTrue(Commands.runOnce(() -> headingZoneLock.setOverride(true)))
        .onFalse(Commands.runOnce(() -> headingZoneLock.setOverride(false)));

    driverController
        .back()
        .onTrue(
            drivetrain.runOnce(
                () -> {
                  boolean applied = odometryHealth.applyRecommendedRecoveryPose();
                  SmartDashboard.putBoolean(
                      "OdometryHealth/LastManualRecoveryApplied", applied);
                }));

    // RT permanece como teste de coleta enquanto a mira nao estiver ativa.
    automaticAimInactive
        .and(driverController.rightTrigger())
        .whileTrue(
            superstructure.holdGoalCommand(
                Goal.COLLECTING, Goal.HOLDING_GAME_PIECE));

    /*
     * Um clique no LT inicia a mira persistente:
     * - dentro da zona da propria alianca: HUB;
     * - fora da zona: ponto de passe mais proximo da propria alianca.
     * Soltar LT nao cancela. B cancela.
     */
    driverController
        .leftTrigger()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> headingZoneLock.setOverride(false)),
                superstructure.startAutomaticAimCommand()));

    automaticAimInactive
        .and(driverController.y())
        .onTrue(superstructure.setGoalCommand(Goal.CLIMBING));
    automaticAimInactive
        .and(driverController.x())
        .onTrue(superstructure.setGoalCommand(Goal.IDLE));
    automaticAimInactive
        .and(driverController.rightBumper())
        .onTrue(superstructure.setGoalCommand(Goal.HOLDING_GAME_PIECE));

    // Zero e uma volta completa representam o mesmo heading; ambos redefinem a trava para frente.
    driverController
        .povUp()
        .or(driverController.povDown())
        .onTrue(Commands.runOnce(headingZoneLock::resetTargetHeading));

    // POV direito inicia a medicao completa. POV esquerdo cancela sem alterar constantes.
    driverController.povRight().onTrue(geometryCalibration);
    driverController
        .povLeft()
        .onTrue(Commands.runOnce(geometryCalibration::cancel));
  }

  private SendableChooser<Command> configureAutonomousChooser() {
    SendableChooser<Command> chooser =
        drivetrain.isPathPlannerConfigured()
            ? AutoBuilder.buildAutoChooser()
            : new SendableChooser<>();
    chooser.setDefaultOption("Parado", Commands.none());
    chooser.addOption("Reto - 2 segundos", createDriveStraightAuto());
    chooser.addOption("Quadrado - simulacao", createSquareAuto());
    SmartDashboard.putData("Autonomo", chooser);
    SmartDashboard.putBoolean(
        "PathPlanner/Configured", drivetrain.isPathPlannerConfigured());
    return chooser;
  }

  private Command createDriveStraightAuto() {
    return Commands.sequence(
        resetSimulatedPose(),
        driveRobotRelative(
            ConfigSwerve.AUTO_TRANSLATION_SPEED_METERS_PER_SECOND,
            0.0,
            0.0,
            ConfigSwerve.AUTO_STRAIGHT_DURATION_SECONDS),
        holdBrake(ConfigSwerve.AUTO_BRAKE_DURATION_SECONDS));
  }

  private Command createSquareAuto() {
    return Commands.sequence(
        resetSimulatedPose(),
        driveRobotRelative(
            ConfigSwerve.AUTO_TRANSLATION_SPEED_METERS_PER_SECOND,
            0.0,
            0.0,
            ConfigSwerve.AUTO_SQUARE_SIDE_DURATION_SECONDS),
        driveRobotRelative(
            0.0,
            ConfigSwerve.AUTO_TRANSLATION_SPEED_METERS_PER_SECOND,
            0.0,
            ConfigSwerve.AUTO_SQUARE_SIDE_DURATION_SECONDS),
        driveRobotRelative(
            -ConfigSwerve.AUTO_TRANSLATION_SPEED_METERS_PER_SECOND,
            0.0,
            0.0,
            ConfigSwerve.AUTO_SQUARE_SIDE_DURATION_SECONDS),
        driveRobotRelative(
            0.0,
            -ConfigSwerve.AUTO_TRANSLATION_SPEED_METERS_PER_SECOND,
            0.0,
            ConfigSwerve.AUTO_SQUARE_SIDE_DURATION_SECONDS),
        holdBrake(ConfigSwerve.AUTO_BRAKE_DURATION_SECONDS));
  }

  private Command resetSimulatedPose() {
    return drivetrain.runOnce(() -> drivetrain.resetPose(new Pose2d()));
  }

  private Command driveRobotRelative(
      double velocityX,
      double velocityY,
      double rotationalRate,
      double seconds) {
    SwerveRequest.RobotCentric request =
        new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withVelocityX(velocityX)
            .withVelocityY(velocityY)
            .withRotationalRate(rotationalRate);
    return drivetrain
        .applyRequest(
            () -> {
              odometryHealth.setRequestedSpeeds(velocityX, velocityY, rotationalRate);
              return odometryHealth.shouldHoldPosition() ? brake : request;
            })
        .finallyDo(interrupted -> odometryHealth.setRequestedSpeeds(0.0, 0.0, 0.0))
        .withTimeout(seconds);
  }

  private Command holdBrake(double seconds) {
    return drivetrain
        .applyRequest(
            () -> {
              odometryHealth.setRequestedSpeeds(0.0, 0.0, 0.0);
              return brake;
            })
        .withTimeout(seconds);
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
  }
}
