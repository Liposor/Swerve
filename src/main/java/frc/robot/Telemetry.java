package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

/** Publica pose, velocidades e estados dos modulos para AdvantageScope/NetworkTables. */
public class Telemetry {
  private static final double PUBLISH_PERIOD_SECONDS = 0.05;
  private final NetworkTable driveTable =
      NetworkTableInstance.getDefault().getTable("Swerve");
  private final StructPublisher<Pose2d> posePublisher =
      driveTable.getStructTopic("Pose", Pose2d.struct).publish();
  private final StructPublisher<ChassisSpeeds> speedsPublisher =
      driveTable.getStructTopic("ChassisSpeeds", ChassisSpeeds.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> statesPublisher =
      driveTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> targetsPublisher =
      driveTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
  private final StructArrayPublisher<SwerveModulePosition> positionsPublisher =
      driveTable
          .getStructArrayTopic("ModulePositions", SwerveModulePosition.struct)
          .publish();
  private final DoublePublisher odometryFrequencyPublisher =
      driveTable.getDoubleTopic("OdometryFrequencyHz").publish();
  private final DoubleArrayPublisher moduleSpeedErrorsPublisher =
      driveTable.getDoubleArrayTopic("ModuleSpeedErrorsMetersPerSecond").publish();
  private final DoublePublisher moduleSpeedErrorRmsPublisher =
      driveTable.getDoubleTopic("ModuleSpeedErrorRmsMetersPerSecond").publish();
  private final DoublePublisher moduleSpeedErrorMaximumPublisher =
      driveTable.getDoubleTopic("ModuleSpeedErrorMaximumMetersPerSecond").publish();
  private double lastPublishSeconds = Double.NEGATIVE_INFINITY;

  public Telemetry() {
    SignalLogger.start();
  }

  public void telemeterize(SwerveDriveState state) {
    double nowSeconds = Utils.getCurrentTimeSeconds();
    if (nowSeconds - lastPublishSeconds < PUBLISH_PERIOD_SECONDS) {
      return;
    }
    lastPublishSeconds = nowSeconds;

    posePublisher.set(state.Pose);
    speedsPublisher.set(state.Speeds);
    statesPublisher.set(state.ModuleStates);
    targetsPublisher.set(state.ModuleTargets);
    positionsPublisher.set(state.ModulePositions);
    odometryFrequencyPublisher.set(1.0 / state.OdometryPeriod);

    int moduleCount = Math.min(state.ModuleStates.length, state.ModuleTargets.length);
    double[] speedErrors = new double[moduleCount];
    double squaredErrorSum = 0.0;
    double maximumAbsoluteError = 0.0;
    for (int moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
      double speedError =
          state.ModuleTargets[moduleIndex].speedMetersPerSecond
              - state.ModuleStates[moduleIndex].speedMetersPerSecond;
      speedErrors[moduleIndex] = speedError;
      squaredErrorSum += speedError * speedError;
      maximumAbsoluteError = Math.max(maximumAbsoluteError, Math.abs(speedError));
    }
    moduleSpeedErrorsPublisher.set(speedErrors);
    moduleSpeedErrorRmsPublisher.set(
        moduleCount > 0 ? Math.sqrt(squaredErrorSum / moduleCount) : 0.0);
    moduleSpeedErrorMaximumPublisher.set(maximumAbsoluteError);

    SignalLogger.writeStruct("Swerve/Pose", Pose2d.struct, state.Pose);
    SignalLogger.writeStruct("Swerve/ChassisSpeeds", ChassisSpeeds.struct, state.Speeds);
    SignalLogger.writeStructArray(
        "Swerve/ModuleStates", SwerveModuleState.struct, state.ModuleStates);
    SignalLogger.writeStructArray(
        "Swerve/ModuleTargets", SwerveModuleState.struct, state.ModuleTargets);
  }
}