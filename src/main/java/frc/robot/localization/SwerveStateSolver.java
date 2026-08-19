package frc.robot.localization;

import frc.robot.config.ConfigLocalization;
import frc.robot.subsystems.Superstructure.Goal;

/** Resolve prioridades do swerve sem misturar regras de seguranca com leitura de joystick. */
public final class SwerveStateSolver {
  public enum DriveState {
    NORMAL,
    SCORE_X_LOCK,
    COLLISION_X_LOCK
  }

  public DriveState solve(Goal superstructureGoal, boolean collisionBlocked) {
    if (collisionBlocked) {
      return DriveState.COLLISION_X_LOCK;
    }
    if (ConfigLocalization.LOCK_X_WHILE_SCORING
        && superstructureGoal == Goal.SCORING) {
      return DriveState.SCORE_X_LOCK;
    }
    return DriveState.NORMAL;
  }
}
