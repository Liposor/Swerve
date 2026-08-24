package frc.robot.vision;

/** Leituras MegaTag1 e MegaTag2 produzidas pela mesma Limelight. */
public record VisionHeadingResetSample(
    String cameraName,
    VisionObservation megaTag1,
    VisionObservation megaTag2) {

  public VisionHeadingResetSample {
    if (cameraName == null || cameraName.isBlank()) {
      throw new IllegalArgumentException("A amostra precisa do nome da camera.");
    }
    if (megaTag1 == null && megaTag2 == null) {
      throw new IllegalArgumentException("A amostra precisa de MT1 ou MT2.");
    }
  }
}
