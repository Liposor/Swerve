package frc.robot.config;

/** Ajustes do monitor de odometria, colisao, desempenho, PathPlanner e calibracao. */
public final class ConfigLocalization {
  private ConfigLocalization() {}

  /** Historico fixo: uma amostra a cada 20 ms por dois segundos. */
  public static final double SAMPLE_PERIOD_SECONDS = 0.020;
  public static final double HISTORY_SECONDS = 2.0;
  public static final int HISTORY_CAPACITY =
      (int) Math.ceil(HISTORY_SECONDS / SAMPLE_PERIOD_SECONDS) + 1;

  /** Mantem a deteccao e a telemetria ligadas. */
  public static final boolean ENABLE_COLLISION_MONITOR = true;

  /**
   * Comeca desligado ate os limites serem validados com o robo no chao.
   *
   * <p>Quando false, o monitor continua detectando e publicando BLOCKED, mas nao manda X-lock.
   */
  public static final boolean ENABLE_COLLISION_X_LOCK = false;

  /**
   * O monitor de colisao translacional nao deve analisar giro puro. So existe uma suspeita quando
   * o piloto ou o autonomo realmente pede pelo menos esta velocidade de translacao.
   */
  public static final double MIN_COMMANDED_TRANSLATION_FOR_IMPACT_METERS_PER_SECOND = 0.60;

  /** Filtro simples para reduzir vibracao do Pigeon antes do calculo de jerk. */
  public static final double IMU_ACCELERATION_FILTER_ALPHA = 0.35;

  /** Limites iniciais mais resistentes a aceleracao normal de um swerve. */
  public static final double IMPACT_ACCELERATION_METERS_PER_SECOND_SQUARED = 8.0;
  public static final double IMPACT_JERK_METERS_PER_SECOND_CUBED = 120.0;

  /** Um impacto so vira BLOCKED se tambem houver travamento persistente. */
  public static final double COLLISION_CONFIRMATION_WINDOW_SECONDS = 0.60;
  public static final double MIN_COMMANDED_SPEED_FOR_STALL_METERS_PER_SECOND = 0.80;
  public static final double MAX_MEASURED_TO_COMMANDED_STALL_RATIO = 0.25;
  public static final double STALL_DEBOUNCE_SECONDS = 0.20;

  /**
   * Slip continua sendo calculado apenas para diagnostico. Ele nunca aciona o X-lock sozinho,
   * porque rodas suspensas em um carrinho parecem slip para o Pigeon.
   */
  public static final double MIN_WHEEL_ACCELERATION_FOR_SLIP = 4.0;
  public static final double MAX_IMU_TO_WHEEL_ACCELERATION_RATIO = 0.20;
  public static final double SLIP_DEBOUNCE_SECONDS = 0.15;

  /** O piloto precisa soltar translacao e rotacao para liberar um BLOCKED confirmado. */
  public static final double BLOCK_RELEASE_TRANSLATION_METERS_PER_SECOND = 0.20;
  public static final double BLOCK_RELEASE_OMEGA_RADIANS_PER_SECOND = 0.25;
  public static final double BLOCK_RELEASE_STABLE_SECONDS = 0.20;

  public static final double RECOVERY_STABLE_SECONDS = 0.40;
  public static final double STABLE_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.80;
  public static final int TRUSTED_POSE_LOOKBACK_SAMPLES = 5;

  /** Nao integra um atraso longo como se a ultima velocidade tivesse sido constante. */
  public static final double MAX_RECOVERY_INTEGRATION_STEP_SECONDS = 0.040;

  /*
   * Um rollback automatico pode piorar a pose se outro robo realmente deslocar o chassi.
   * A infraestrutura e o comando manual existem, mas a automacao fica desligada por seguranca.
   */
  public static final boolean ENABLE_AUTOMATIC_COLLISION_ROLLBACK = false;
  public static final double MAX_MANUAL_ROLLBACK_DISTANCE_METERS = 0.50;

  /** Trava X quando pontuando e durante bloqueio confirmado. */
  public static final boolean LOCK_X_WHILE_SCORING = true;

  /** Ganhos iniciais do controlador holonomico PathPlanner. */
  public static final double PATH_TRANSLATION_KP = 5.0;
  public static final double PATH_ROTATION_KP = 4.0;

  /**
   * Calibracao automatica da posicao XY dos modulos.
   *
   * <p>O robo faz tres voltas em cada sentido. O valor e propositalmente baixo para reduzir
   * derrapagem; aumente somente depois de validar a rotina em uma area livre.
   */
  public static final double GEOMETRY_CALIBRATION_OMEGA_RADIANS_PER_SECOND = 0.60;
  public static final double GEOMETRY_CALIBRATION_TURNS_PER_DIRECTION = 3.0;
  public static final double GEOMETRY_CALIBRATION_RAMP_SECONDS = 0.75;
  public static final double GEOMETRY_CALIBRATION_SETTLE_SECONDS = 0.75;
  public static final double GEOMETRY_CALIBRATION_TIMEOUT_SECONDS = 90.0;
  public static final double GEOMETRY_CALIBRATION_MIN_DELTA_THETA_RADIANS = 1.0e-5;
  public static final int GEOMETRY_CALIBRATION_MIN_SAMPLES_PER_DIRECTION = 500;

  /** Limites que decidem se o resultado pode ser copiado para o TunerConstants. */
  public static final double GEOMETRY_CALIBRATION_MAX_DIRECTION_DIFFERENCE_METERS = 0.025;
  public static final double GEOMETRY_CALIBRATION_MAX_SYMMETRY_ERROR_METERS = 0.020;

  /** Telemetria e alertas de desempenho. */
  public static final double PERFORMANCE_TELEMETRY_PERIOD_SECONDS = 0.20;
  public static final double LOOP_WARNING_SECONDS = 0.030;
  public static final double CAN_WARNING_UTILIZATION = 0.85;
  public static final double CPU_TEMPERATURE_WARNING_CELSIUS = 75.0;
}
