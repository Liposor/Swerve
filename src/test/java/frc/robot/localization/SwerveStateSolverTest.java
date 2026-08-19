package frc.robot.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.localization.SwerveStateSolver.DriveState;
import frc.robot.subsystems.Superstructure.Goal;

class SwerveStateSolverTest {
  private final SwerveStateSolver solver = new SwerveStateSolver();

  @Test
  void collisionHasPriorityOverScoring() {
    assertEquals(DriveState.COLLISION_X_LOCK, solver.solve(Goal.SCORING, true));
  }

  @Test
  void scoringLocksModulesInX() {
    assertEquals(DriveState.SCORE_X_LOCK, solver.solve(Goal.SCORING, false));
  }

  @Test
  void normalGoalsKeepDriverControl() {
    assertEquals(DriveState.NORMAL, solver.solve(Goal.COLLECTING, false));
  }
}
