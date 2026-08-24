package frc.robot.util;

import java.util.concurrent.atomic.AtomicLong;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ConfigLocalization;
import frc.robot.generated.TunerConstants;

/** Monitora a saude da roboRIO sem executar telemetria lenta no CommandScheduler. */
public final class PerformanceMonitor extends SubsystemBase {
  private final AtomicLong maximumLoopIntervalMicros = new AtomicLong();
  private final Notifier telemetryNotifier;
  private long previousPeriodicMicros = -1;
  private double lastWarningSeconds = Double.NEGATIVE_INFINITY;

  public PerformanceMonitor() {
    telemetryNotifier = new Notifier(this::sampleAndPublish);
    telemetryNotifier.setName("PerformanceTelemetry");
    telemetryNotifier.startPeriodic(ConfigLocalization.PERFORMANCE_TELEMETRY_PERIOD_SECONDS);
  }

  @Override
  public void periodic() {
    long nowMicros = RobotController.getFPGATime();
    if (previousPeriodicMicros >= 0) {
      updateMaximum(maximumLoopIntervalMicros, nowMicros - previousPeriodicMicros);
    }
    previousPeriodicMicros = nowMicros;
  }

  private void sampleAndPublish() {
    double nowSeconds = RobotController.getFPGATime() / 1_000_000.0;
    double maximumLoopIntervalMilliseconds = maximumLoopIntervalMicros.getAndSet(0L) / 1000.0;

    var canStatus = TunerConstants.CAN_BUS.getStatus();
    boolean canHealthy = canStatus.Status.isOK();
    double cpuTemperature = RobotController.getCPUTemp();
    Runtime runtime = Runtime.getRuntime();
    double usedMemoryMiB =
        (runtime.totalMemory() - runtime.freeMemory()) / (1024.0 * 1024.0);

    SmartDashboard.putNumber("Performance/MaxLoopIntervalMs", maximumLoopIntervalMilliseconds);
    SmartDashboard.putString("Performance/CanBus", TunerConstants.CAN_BUS.getName());
    SmartDashboard.putBoolean("Performance/CanHealthy", canHealthy);
    SmartDashboard.putString("Performance/CanStatus", canStatus.Status.getName());
    SmartDashboard.putNumber("Performance/CanUtilizationPercent", canStatus.BusUtilization * 100.0);
    SmartDashboard.putNumber("Performance/CanBusOffCount", canStatus.BusOffCount);
    SmartDashboard.putNumber("Performance/CanTxFullCount", canStatus.TxFullCount);
    SmartDashboard.putNumber("Performance/CanReceiveErrors", canStatus.REC);
    SmartDashboard.putNumber("Performance/CanTransmitErrors", canStatus.TEC);
    SmartDashboard.putNumber("Performance/CpuTemperatureC", cpuTemperature);
    SmartDashboard.putNumber("Performance/JavaUsedMemoryMiB", usedMemoryMiB);

    if (nowSeconds - lastWarningSeconds < 2.0) {
      return;
    }

    if (!canHealthy) {
      DriverStation.reportWarning(
          "Barramento CAN indisponivel: "
              + TunerConstants.CAN_BUS.getName()
              + " ("
              + canStatus.Status.getName()
              + ")",
          false);
      lastWarningSeconds = nowSeconds;
    } else if (maximumLoopIntervalMilliseconds
        > ConfigLocalization.LOOP_WARNING_SECONDS * 1000.0) {
      DriverStation.reportWarning(
          String.format(
              "Loop principal atrasou: maximo recente %.1f ms",
              maximumLoopIntervalMilliseconds),
          false);
      lastWarningSeconds = nowSeconds;
    } else if (canStatus.BusUtilization > ConfigLocalization.CAN_WARNING_UTILIZATION) {
      DriverStation.reportWarning(
          String.format("CAN acima do limite saudavel: %.1f%%", canStatus.BusUtilization * 100.0),
          false);
      lastWarningSeconds = nowSeconds;
    } else if (RobotBase.isReal()
        && cpuTemperature > ConfigLocalization.CPU_TEMPERATURE_WARNING_CELSIUS) {
      DriverStation.reportWarning(
          String.format("Temperatura da roboRIO alta: %.1f C", cpuTemperature), false);
      lastWarningSeconds = nowSeconds;
    }
  }

  private static void updateMaximum(AtomicLong maximum, long candidate) {
    long current = maximum.get();
    while (candidate > current && !maximum.compareAndSet(current, candidate)) {
      current = maximum.get();
    }
  }
}
