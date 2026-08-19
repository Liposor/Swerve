package frc.robot.config;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** Constantes da selecao automatica de HUB/passe e da trava de heading. */
public final class ConfigAutomaticAim {
  private ConfigAutomaticAim() {}

  /*
   * Coordenadas no sistema oficial WPILib:
   * +X parte da parede azul em direcao a parede vermelha;
   * +Y parte da lateral direita da alianca azul em direcao a lateral esquerda.
   */
  public static final Translation2d BLUE_HUB = new Translation2d(4.647, 4.036);
  public static final Translation2d RED_HUB = new Translation2d(11.870, 4.047);

  /*
   * Pontos de passe da propria alianca. O sistema nunca seleciona os pontos da alianca adversaria.
   * LOWER corresponde ao lado de menor Y e UPPER ao lado de maior Y.
   */
  public static final Translation2d BLUE_PASS_LOWER = new Translation2d(2.324, 2.363);
  public static final Translation2d BLUE_PASS_UPPER = new Translation2d(2.347, 5.860);
  public static final Translation2d RED_PASS_LOWER = new Translation2d(14.324, 2.363);
  public static final Translation2d RED_PASS_UPPER = new Translation2d(14.346, 5.860);

  /** Profundidade oficial da ALLIANCE ZONE: 158.6 polegadas. */
  public static final double ALLIANCE_ZONE_DEPTH_METERS = Units.inchesToMeters(158.6);

  /** Evita trocar repetidamente entre os dois pontos de passe perto do meio do campo. */
  public static final double PASS_TARGET_SWITCH_HYSTERESIS_METERS = 0.40;

  /** PID de heading. A saida do controlador e interpretada como radianos por segundo. */
  public static final double HEADING_KP = 5.0;
  public static final double HEADING_KI = 0.0;
  public static final double HEADING_KD = 0.10;

  /** Condicoes que precisam permanecer verdadeiras para declarar ALIGNED. */
  public static final double HEADING_TOLERANCE_RADIANS = Units.degreesToRadians(2.0);
  public static final double MAX_ALIGNED_OMEGA_RADIANS_PER_SECOND =
      Units.degreesToRadians(10.0);
  public static final double ALIGNED_STABLE_SECONDS = 0.20;

  /** Impede atan2 instavel se a pose estiver praticamente sobre o alvo. */
  public static final double MIN_TARGET_DISTANCE_METERS = 0.05;

  /*
   * 0 graus significa que o mecanismo lanca pela frente do robo (+X do robo).
   * Use 180 se o shooter/passador lancar pela traseira.
   */
  public static final double SHOOTER_YAW_OFFSET_RADIANS = Units.degreesToRadians(0.0);
}
