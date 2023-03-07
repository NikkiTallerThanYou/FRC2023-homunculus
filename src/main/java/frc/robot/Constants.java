package frc.robot;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;



public final class Constants {

  public static final int kTimoutms = 30;

  public static final double wheelRad = Units.inchesToMeters(3);
  public static final double wheelBaseWidth = Units.inchesToMeters(28); // test robot
  // public final double wheelbwidth = Units.inchesToMeters(26); // new robot
  public static final double wheelBaseLength =30;
  public static final double gearRatio = 10; // test robot
  // public final double gearratio = 9.8; // new robot
  public static final double maxSpeed = 4.267; //m/s
  public static final double maxTurn = 2.0; //rad/s

  public static final double dKf = 1023.0 / 18252;
  public static final double dKp = 0.05;
  public static final double dKi = 0.000;
  public static final double dKd = 0;
  public static final double dIz= 300;
  public static final int encoderCounts = 2048;
  public static final int voltageDropThreshold = 3;
  
  public static final int dkslot = 0;
  public static final int dkloop = 0;

  static class FieldConstants {
    static final double length = Units.feetToMeters(54);
    static final double width = Units.feetToMeters(27);
}

  static class VisionConstants {
    static final Transform3d robotToCam =
            new Transform3d(
                    new Translation3d(0.5, 0.0, 0.5),
                    new Rotation3d(
                            0, 0,
                            0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
    // from center.
    static final String cameraName = "OV5647";
}
  
}
