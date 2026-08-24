package frc.robot.generated;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.ConfigSwerve;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Configuracao gerada do swerve, isolada do codigo de comportamento do robo. */
public final class TunerConstants {
  private TunerConstants() {}

  public static final boolean HARDWARE_CONFIGURED = true;

  /** Requer Phoenix Pro nos quatro Talon FX/Kraken de drive. */
  public static final boolean USE_TORQUE_CURRENT_FOC = true;

  private static final Slot0Configs STEER_GAINS =
      new Slot0Configs()
          .withKP(100)
          .withKI(0)
          .withKD(0.5)
          .withKS(0.1)
          .withKV(1.91)
          .withKA(0)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

  /**
   * Ganhos iniciais para VelocityTorqueCurrentFOC.
   *
   * <p>Esses valores usam amperes como saida, por isso nao podem ser comparados aos ganhos antigos
   * de Voltage. Sao o ponto de partida publico usado pela 254; ajuste kS e kP pelos logs do seu
   * proprio robo.
   */
  private static final Slot0Configs TORQUE_CURRENT_DRIVE_GAINS =
      new Slot0Configs()
          .withKP(10.0)
          .withKI(0.0)
          .withKD(0.0)
          .withKS(3.30)
          .withKV(0.055)
          .withKA(0.56);

  /** Fallback para manter VelocityVoltage caso os motores de drive nao tenham Phoenix Pro. */
  private static final Slot0Configs VOLTAGE_DRIVE_GAINS =
      new Slot0Configs()
          .withKP(0.1)
          .withKI(0.0)
          .withKD(0.0)
          .withKS(0.0)
          .withKV(0.124)
          .withKA(0.0);

  private static final Slot0Configs DRIVE_GAINS =
      USE_TORQUE_CURRENT_FOC ? TORQUE_CURRENT_DRIVE_GAINS : VOLTAGE_DRIVE_GAINS;

  private static final Current SLIP_CURRENT = Amps.of(120);

  private static final TalonFXConfiguration DRIVE_INITIAL_CONFIGS =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(Amps.of(70))
                  .withSupplyCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

  private static final TalonFXConfiguration STEER_INITIAL_CONFIGS =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(60))
                  .withStatorCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

  private static final CANcoderConfiguration ENCODER_INITIAL_CONFIGS =
      new CANcoderConfiguration();
  private static final Pigeon2Configuration PIGEON_CONFIGS =
      new Pigeon2Configuration()
          .withMountPose(new MountPoseConfigs().withMountPoseRoll(Degrees.of(180)));

  /**
   * "*" seleciona o unico CANivore visivel e evita falha por nome alterado.
   *
   * <p>Se o robo passar a usar mais de um CANivore, substitua pelo numero de serie do CANivore do
   * swerve.
   */
  private static final String REAL_CANIVORE_SELECTOR = "*";

  /**
   * No robo real usa o CANivore; na simulacao usa o barramento virtual da roboRIO.
   *
   * <p>O construtor de dois argumentos nao e usado porque o segundo argumento carrega um arquivo
   * Hoot para replay; ele nao define o destino de gravacao de logs.
   */
  public static final CANBus CAN_BUS =
      RobotBase.isReal() ? new CANBus(REAL_CANIVORE_SELECTOR) : CANBus.roboRIO();

  private static final double COUPLING_GEAR_RATIO = 3.375;
  private static final double DRIVE_GEAR_RATIO = 5.2734375;
  private static final double STEER_GEAR_RATIO = 26.09090909090909;

  /** Raio efetivo caracterizado no carpete em 14/08/2026. */
  private static final Distance WHEEL_RADIUS = Inches.of(2.02211);

  private static final boolean INVERT_LEFT_SIDE = false;
  private static final boolean INVERT_RIGHT_SIDE = true;
  private static final int PIGEON_ID = 13;

  // Parametros usados pelo modelo de simulacao da Phoenix 6.
  private static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.01);
  private static final MomentOfInertia DRIVE_INERTIA = KilogramSquareMeters.of(0.035);
  private static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.2);
  private static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.2);

  public static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS =
      new SwerveDrivetrainConstants()
          .withCANBusName(CAN_BUS.getName())
          .withPigeon2Id(PIGEON_ID)
          .withPigeon2Configs(PIGEON_CONFIGS);

  private static final SwerveModuleConstantsFactory<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      MODULE_FACTORY =
          new SwerveModuleConstantsFactory<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
              .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
              .withSteerMotorGearRatio(STEER_GEAR_RATIO)
              .withCouplingGearRatio(COUPLING_GEAR_RATIO)
              .withWheelRadius(WHEEL_RADIUS)
              .withSteerMotorGains(STEER_GAINS)
              .withDriveMotorGains(DRIVE_GAINS)
              .withSteerMotorClosedLoopOutput(
                  SwerveModuleConstants.ClosedLoopOutputType.Voltage)
              .withDriveMotorClosedLoopOutput(
                  USE_TORQUE_CURRENT_FOC
                      ? SwerveModuleConstants.ClosedLoopOutputType.TorqueCurrentFOC
                      : SwerveModuleConstants.ClosedLoopOutputType.Voltage)
              .withSlipCurrent(SLIP_CURRENT)
              .withSpeedAt12Volts(ConfigSwerve.SPEED_AT_12_VOLTS)
              .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
              .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
              .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
              .withDriveMotorInitialConfigs(DRIVE_INITIAL_CONFIGS)
              .withSteerMotorInitialConfigs(STEER_INITIAL_CONFIGS)
              .withEncoderInitialConfigs(ENCODER_INITIAL_CONFIGS)
              .withSteerInertia(STEER_INERTIA)
              .withDriveInertia(DRIVE_INERTIA)
              .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
              .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE);

  private static final Distance HALF_WHEELBASE = Inches.of(11.4173);
  private static final Distance HALF_TRACKWIDTH = Inches.of(10.1378);
  private static final Distance NEGATIVE_HALF_WHEELBASE = Inches.of(-11.4173);
  private static final Distance NEGATIVE_HALF_TRACKWIDTH = Inches.of(-10.1378);

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FRONT_LEFT =
          createModule(11, 12, 10, 0.017578125, HALF_WHEELBASE, HALF_TRACKWIDTH, false);

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FRONT_RIGHT =
          createModule(8, 9, 7, -0.22216796875, HALF_WHEELBASE, NEGATIVE_HALF_TRACKWIDTH, true);

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BACK_LEFT =
          createModule(2, 3, 1, 0.256591796875, NEGATIVE_HALF_WHEELBASE, HALF_TRACKWIDTH, false);

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BACK_RIGHT =
          createModule(
              5,
              6,
              4,
              0.20703125,
              NEGATIVE_HALF_WHEELBASE,
              NEGATIVE_HALF_TRACKWIDTH,
              true);

  private static SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      createModule(
          int steerId,
          int driveId,
          int encoderId,
          double encoderOffsetRotations,
          Distance x,
          Distance y,
          boolean rightSide) {
    return MODULE_FACTORY.createModuleConstants(
        steerId,
        driveId,
        encoderId,
        Rotations.of(encoderOffsetRotations),
        x,
        y,
        rightSide ? INVERT_RIGHT_SIDE : INVERT_LEFT_SIDE,
        false,
        false);
  }

  public static CommandSwerveDrivetrain createDrivetrain() {
    if (RobotBase.isReal() && !HARDWARE_CONFIGURED) {
      throw new IllegalStateException(
          "Swerve bloqueado no robo real: gere e valide TunerConstants no Tuner X primeiro.");
    }
    return new CommandSwerveDrivetrain(
        DRIVETRAIN_CONSTANTS, FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT);
  }

  /** Drivetrain tipado para Talon FX + CANcoder. */
  public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules) {
      super(
          TalonFX::new,
          TalonFX::new,
          CANcoder::new,
          drivetrainConstants,
          ConfigSwerve.ODOMETRY_UPDATE_FREQUENCY_HZ,
          modules);

      getOdometryThread().setThreadPriority(ConfigSwerve.ODOMETRY_THREAD_PRIORITY);
    }
  }
}
