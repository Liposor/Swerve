package frc.robot.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.config.ConfigAutomaticAim;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Mantem o chassi apontado para um heading escolhido enquanto o override estiver ativo. */
public final class SwerveHeadingZoneLock {
  private final CommandSwerveDrivetrain drivetrain;
  private final PIDController headingController =
      new PIDController(
          ConfigAutomaticAim.HEADING_KP,
          ConfigAutomaticAim.HEADING_KI,
          ConfigAutomaticAim.HEADING_KD);

  private Rotation2d targetHeading = Rotation2d.kZero;
  private boolean overrideActive;

  public SwerveHeadingZoneLock(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void setOverride(boolean active) {
    overrideActive = active;
  }

  public boolean isLockActive() {
    return overrideActive;
  }

  public void setTargetHeading(Rotation2d targetHeading) {
    this.targetHeading = targetHeading;
  }

  public void resetTargetHeading() {
    targetHeading = Rotation2d.kZero;
  }

  public double calculateLockOmega(double maximumRotationRadiansPerSecond) {
    if (!overrideActive) {
      return 0.0;
    }

    double currentHeadingRadians =
        drivetrain.getState().Pose.getRotation().getRadians();
    double output =
        headingController.calculate(currentHeadingRadians, targetHeading.getRadians());
    return MathUtil.clamp(
        output,
        -maximumRotationRadiansPerSecond,
        maximumRotationRadiansPerSecond);
  }
}
