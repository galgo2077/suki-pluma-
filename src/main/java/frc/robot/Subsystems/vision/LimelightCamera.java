package frc.robot.Subsystems.vision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;

/** Registers Limelight's existing MJPEG stream without reading or re-encoding frames. */
public final class LimelightCamera {
  private static boolean registered;

  private LimelightCamera() {}

  public static synchronized void register(String name, String streamUrl) {
    if (registered) return;
    CameraServer.startAutomaticCapture(new HttpCamera(name, streamUrl));
    registered = true;
  }
}
