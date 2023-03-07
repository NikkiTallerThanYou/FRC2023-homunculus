package frc.robot.subsystems.drivetrain;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;


import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import java.util.Optional;
import java.util.function.BiConsumer;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;

import frc.robot.Constants;
import frc.robot.PhotonCameraWrapper;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;



public class Drivetrain extends SubsystemBase implements Loggable {
  //motors
  private final WPI_TalonFX m_leftMaster = new WPI_TalonFX(1);
  private final WPI_TalonFX m_leftDrone = new WPI_TalonFX(2);
  private final WPI_TalonFX m_rightMaster = new WPI_TalonFX(3);
  private final WPI_TalonFX m_rightDrone = new WPI_TalonFX(4);

  //gyro
  private final WPI_Pigeon2 m_gyro = new WPI_Pigeon2(0);

  //motor groups
  @Log.MotorController(name = "Left Motors", tabName = "Drivetrain")
  private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(m_leftMaster, m_leftDrone);
  @Log.MotorController(name = "Right Motors", tabName = "Drivetrain")
  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(m_rightMaster, m_rightDrone);

  //camera
  public PhotonCameraWrapper pcw;

  
  //kinematics
  public final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(Constants.wheelBaseWidth);

  // a varible used to keep track of the current commanded speed from the joystick
  // used in smooth drive as part of a low pass filter
  static double filtspeed =0;
 
 
  //odometry
  private final DifferentialDrivePoseEstimator m_poseEstimator =
      new DifferentialDrivePoseEstimator(
              m_kinematics, new Rotation2d(0), 0.0, 0.0, new Pose2d());
  //ramsete controller
  public final RamseteController m_ramseteController = new RamseteController();

 // Simulation classes help us simulate our robot
 private final BasePigeonSimCollection m_gyroSim = m_gyro.getSimCollection();
 private final TalonFXSimCollection m_leftDriveSim = m_leftMaster.getSimCollection();
 private final TalonFXSimCollection m_rightDriveSim = m_rightMaster.getSimCollection();

 final int kCountsPerRev = Constants.encoderCounts;  //Encoder counts per revolution of the motor shaft.
 final double kSensorGearRatio = Constants.gearRatio; //Gear ratio is the ratio between the *encoder* and the wheels.  
 final double kGearRatio = Constants.gearRatio; //Switch kSensorGearRatio to this gear ratio if encoder is on the motor instead of on the gearbox.
 final double kWheelRadiusInches = 3; //Wheel radius in inches
 final int k100msPerSecond = 10;


 private final Field2d m_fieldSim = new Field2d();
 private final LinearSystem<N2, N2, N2> m_drivetrainSystem =
         LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
 private final DifferentialDrivetrainSim m_driveSim =
         new DifferentialDrivetrainSim(
                 m_drivetrainSystem,
                 DCMotor.getFalcon500(2),
                 8,
                 Constants.wheelBaseWidth,
                 Constants.wheelRad,
                 null);

import frc.robot.Constants;
import frc.robot.PhotonCameraWrapper;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;


    m_leftMaster.setNeutralMode(NeutralMode.Brake);
    m_leftDrone.setNeutralMode(NeutralMode.Brake);
    m_rightMaster.setNeutralMode(NeutralMode.Brake);
    m_rightDrone.setNeutralMode(NeutralMode.Brake);
    // m_leftMaster.setNeutralMode(NeutralMode.Coast);
    // m_leftDrone.setNeutralMode(NeutralMode.Coast);
    // m_rightMaster.setNeutralMode(NeutralMode.Coast);
    // m_rightDrone.setNeutralMode(NeutralMode.Coast);

    //left master config
    m_leftMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.dkloop, Constants.kTimoutms);
    m_leftMaster.configNominalOutputForward(0);
    m_leftMaster.configNominalOutputReverse(0);
    m_leftMaster.configPeakOutputForward(1);
    m_leftMaster.configPeakOutputReverse(-1);
    m_leftMaster.configClosedloopRamp(0);
    
    m_leftMaster.config_kF(Constants.dkslot, Constants.dKf);
    m_leftMaster.config_kP(Constants.dkslot, Constants.dKp);
    m_leftMaster.config_kI(Constants.dkslot, Constants.dKi);
    m_leftMaster.config_kD(Constants.dkslot, Constants.dKd);
    m_leftMaster.config_IntegralZone(Constants.dkslot, Constants.dIz);
    

    //right master config
    m_rightMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.dkloop, Constants.kTimoutms);
    m_rightMaster.configNominalOutputForward(0);
    m_rightMaster.configNominalOutputReverse(0);
    m_rightMaster.configPeakOutputForward(1);
    m_rightMaster.configPeakOutputReverse(-1);
    m_rightMaster.configClosedloopRamp(0);

    m_rightMaster.config_kF(Constants.dkslot, Constants.dKf);
    m_rightMaster.config_kP(Constants.dkslot, Constants.dKp);
    m_rightMaster.config_kI(Constants.dkslot, Constants.dKi);
    m_rightMaster.config_kD(Constants.dkslot, Constants.dKd);
    m_rightMaster.config_IntegralZone(Constants.dkslot, Constants.dIz);


    //Odometry
    m_rightMotors.setInverted(true);
    
    m_leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    m_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    
    m_leftMaster.setSelectedSensorPosition(0);
    m_rightMaster.setSelectedSensorPosition(0);
    m_gyro.setYaw(0);


  

   

    //Vision
    pcw = new PhotonCameraWrapper();

  }
// Assuming this method is part of a drivetrain subsystem that provides the necessary methods
public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
  return new SequentialCommandGroup(
      new InstantCommand(() -> {
        // Reset odometry for the first path you run during auto
        if(isFirstPath){
            this.resetOdometry(traj.getInitialPose());
        }
      }),
      new PPRamseteCommand(
          traj, 
          this::getPose, // Pose supplier
          new RamseteController(),
          m_kinematics, // DifferentialDriveKinematics
          this::setSpeeds, // Voltage biconsumer
          true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          this // Requires this drive subsystem
      )
  );
}
public void resetOdometry(Pose2d initialPose) {
  m_poseEstimator.resetPosition(m_gyro.getRotation2d(), nativeUnitsToDistanceMeters( m_leftMaster.getSelectedSensorPosition()), nativeUnitsToDistanceMeters( m_rightMaster.getSelectedSensorPosition()), initialPose);
}


public class Drivetrain extends SubsystemBase implements Loggable {
  //motors
  private final WPI_TalonFX m_leftMaster = new WPI_TalonFX(1);
  private final WPI_TalonFX m_leftDrone = new WPI_TalonFX(2);
  private final WPI_TalonFX m_rightMaster = new WPI_TalonFX(3);
  private final WPI_TalonFX m_rightDrone = new WPI_TalonFX(4);

  //gyro
  private final WPI_Pigeon2 m_gyro = new WPI_Pigeon2(0);

  //motor groups
  @Log.MotorController(name = "Left Motors", tabName = "Drivetrain")
  private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(m_leftMaster, m_leftDrone);
  @Log.MotorController(name = "Right Motors", tabName = "Drivetrain")
  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(m_rightMaster, m_rightDrone);

  // private final WPI_Pigeon2 m_gyro = new WPI_Pigeon2(0); //fix canID

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();

    // System.out.println(String.format("Roll: %f, Pitch: %f, Yaw: %f", m_gyro.getRoll(), m_gyro.getPitch(), m_gyro.getYaw()));
  }
 
  @Override
  public void simulationPeriodic() {
      /* Pass the robot battery voltage to the simulated Talon SRXs */
      m_leftDriveSim.setBusVoltage(RobotController.getBatteryVoltage());
      m_rightDriveSim.setBusVoltage(RobotController.getBatteryVoltage());
      updateOdometry();
      /*
       * CTRE simulation is low-level, so SimCollection inputs
       * and outputs are not affected by SetInverted(). Only
       * the regular user-level API calls are affected.
       *
       * WPILib expects +V to be forward.
       * Positive motor output lead voltage is ccw. We observe
       * on our physical robot that this is reverse for the
       * right motor, so negate it.
       *
       * We are hard-coding the negation of the values instead of
       * using getInverted() so we can catch a possible bug in the
       * robot code where the wrong value is passed to setInverted().
       */
      m_driveSim.setInputs(m_leftDriveSim.getMotorOutputLeadVoltage(),
                           -m_rightDriveSim.getMotorOutputLeadVoltage());
  
      /*
       * Advance the model by 20 ms. Note that if you are running this
       * subsystem in a separate thread or have changed the nominal
       * timestep of TimedRobot, this value needs to match it.
       */
      m_driveSim.update(0.02);
  
      /*
       * Update all of our sensors.
       *
       * Since WPILib's simulation class is assuming +V is forward,
       * but -V is forward for the right motor, we need to negate the
       * position reported by the simulation class. Basically, we
       * negated the input, so we need to negate the output.
       *
       * We also observe on our physical robot that a positive voltage
       * across the output leads results in a positive sensor velocity
       * for both the left and right motors, so we do not need to negate
       * the output any further.
       * If we had observed that a positive voltage results in a negative
       * sensor velocity, we would need to negate the output once more.
       */
      m_leftDriveSim.setIntegratedSensorRawPosition(
                      distanceToNativeUnits(
                          m_driveSim.getLeftPositionMeters()
                      ));
      m_leftDriveSim.setIntegratedSensorRawPosition(
                      velocityToNativeUnits(
                          m_driveSim.getLeftVelocityMetersPerSecond()
                      ));
      m_rightDriveSim.setIntegratedSensorRawPosition(
                      distanceToNativeUnits(
                          -m_driveSim.getRightPositionMeters()
                      ));
      m_rightDriveSim.setIntegratedSensorRawPosition(
                      velocityToNativeUnits(
                          -m_driveSim.getRightVelocityMetersPerSecond()
                      ));
      m_gyroSim.setRawHeading(m_driveSim.getHeading().getDegrees());

  }




  public void arcadeDriveV(double speed, double rotation){
    //use the differntial drive invers kinematics class to set the motor controllers directly in velocity control mode
    
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(speed, 0, rotation));


    // 2048 units per rev
    // 600 : 100ms per min

    // double metertorpm = Constants.wheelrad*2*Math.PI ; //convertion from meters per second to rev per second
    // double rpmtounit = 2048/10; //convertion from rpm to units per 100ms

    // double leftspeedtalon = wheelSpeeds.leftMetersPerSecond / metertorpm * rpmtounit; //unit/100ms
    // double rightspeedtalon = wheelSpeeds.rightMetersPerSecond / metertorpm * rpmtounit; //unit/100ms
    // System.out.print(wheelSpeeds.leftMetersPerSecond);
    // System.out.print("*");
    // System.out.println(wheelSpeeds.rightMetersPerSecond);
   

    double leftspeedtalon = velocityToNativeUnits( wheelSpeeds.leftMetersPerSecond );
    double rightspeedtalon = velocityToNativeUnits( wheelSpeeds.rightMetersPerSecond );
    // System.out.print(leftspeedtalon);
    // System.out.print("  ");
    // System.out.println(rightspeedtalon);

    m_leftMaster.set(ControlMode.Velocity, leftspeedtalon);
    m_rightMaster.set(ControlMode.Velocity, rightspeedtalon);
    m_leftDrone.set(ControlMode.Follower, 1);
    m_rightDrone.set(ControlMode.Follower, 3);
    


  }

   public void smoothDrive(double speed, double rotation){
    
    final double kFiltercoeff = 0;//filter coefficient for the low pass filter high value means more smoothing
     
    //apply a low pass filter to the speed input
    //takes a percentage of the new speed and the oposite percentage of the old speed and adds them together
    speed = speed * (1-kFiltercoeff) + filtspeed * kFiltercoeff ;
    
    //use the differntial drive invers kinematics class to set the motor controllers directly in velocity control mode
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(speed, 0, rotation));
    
    System.out.print(wheelSpeeds.leftMetersPerSecond);
    System.out.print("*");
    System.out.println(wheelSpeeds.rightMetersPerSecond);
   
    double leftspeedtalon = velocityToNativeUnits( wheelSpeeds.leftMetersPerSecond );
    double rightspeedtalon = velocityToNativeUnits( wheelSpeeds.rightMetersPerSecond );
    System.out.print(leftspeedtalon);
    System.out.print("  ");
    System.out.println(rightspeedtalon);

    m_leftMaster.set(ControlMode.Velocity, leftspeedtalon);
    m_rightMaster.set(ControlMode.Velocity, rightspeedtalon);
    m_leftDrone.set(ControlMode.Follower, 1);
    m_rightDrone.set(ControlMode.Follower, 3);
    


  }
  // public void arcadeDrive(double speed, double rotation) {
  //   m_differentialDrive.arcadeDrive(speed, rotation);
  // }

  // public void curvatureDrive(double speed, double rotation, boolean isQuickTurn) {
  //   m_differentialDrive.curvatureDrive(speed, rotation, isQuickTurn);
  // }

  // public void tankDrive(double leftSpeed, double rightSpeed) {
  //   m_differentialDrive.tankDrive(leftSpeed, rightSpeed);
  // }

  public void stop() {
    // m_differentialDrive.stopMotor();
    m_leftMotors.stopMotor();
    m_rightMotors.stopMotor();
  }

  public void updateOdometry() {
    m_poseEstimator.update(
      m_gyro.getRotation2d(),nativeUnitsToDistanceMeters( m_leftMaster.getSelectedSensorPosition()),  nativeUnitsToDistanceMeters( m_rightMaster.getSelectedSensorPosition()));
    Optional<EstimatedRobotPose> result =
            pcw.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());

    if (result.isPresent()) {
        EstimatedRobotPose camPose = result.get();
        m_poseEstimator.addVisionMeasurement(
                camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
        m_fieldSim.getObject("Cam Est Pos").setPose(camPose.estimatedPose.toPose2d());
    } else {
        // move it way off the screen to make it disappear
        m_fieldSim.getObject("Cam Est Pos").setPose(new Pose2d(-100, -100, new Rotation2d()));
    }

    m_fieldSim.getObject("Actual Pos").setPose(m_driveSim.getPose());
    m_fieldSim.setRobotPose(m_poseEstimator.getEstimatedPosition());
}
// @Log.ToString(name = "Pose")
public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
}
@Log.ToString(name = "Translation")
public Translation2d getTranslation() {
    return m_poseEstimator.getEstimatedPosition().getTranslation();
}
@Log.ToString(name = "Rotation")
public Rotation2d getRot() {
    return m_poseEstimator.getEstimatedPosition().getRotation();
}

@Log(name = "Gyro Yaw")
private double getGyroPos(){
  return m_gyro.getYaw();
}

private int distanceToNativeUnits(double positionMeters){
  double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
  double motorRotations = wheelRotations * kSensorGearRatio;
  int sensorCounts = (int)(motorRotations * kCountsPerRev);
  return sensorCounts;
}

private int velocityToNativeUnits(double velocityMetersPerSecond){
  double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
  double motorRotationsPerSecond = wheelRotationsPerSecond * kSensorGearRatio;
  double motorRotationsPer100ms = motorRotationsPerSecond / k100msPerSecond;
  int sensorCountsPer100ms = (int)(motorRotationsPer100ms * kCountsPerRev);
  return sensorCountsPer100ms;
}

private double nativeUnitsToDistanceMeters(double sensorCounts){
  double motorRotations = (double)sensorCounts / kCountsPerRev;
  double wheelRotations = motorRotations / kSensorGearRatio;
  double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
  return positionMeters;
  }

  // BiConsumer<Double, Double> setSpeeds = (left, right) -> {
  //   m_leftMaster.set(ControlMode.Velocity, left);
  //   m_rightMaster.set(ControlMode.Velocity, right);
  //   m_leftDrone.set(ControlMode.Follower, 1);
  //   m_rightDrone.set(ControlMode.Follower, 3);
  // };
public void setSpeeds(Double left, Double right){
  m_leftMaster.set(ControlMode.Velocity, left);
    m_rightMaster.set(ControlMode.Velocity, right);
    m_leftDrone.set(ControlMode.Follower, 1);
    m_rightDrone.set(ControlMode.Follower, 3);
}
private int moveTenFeet(){  //move 10 feet in native units
  double tenFeet = Units.feetToMeters(10);
  int tenFeetNativeUnits = distanceToNativeUnits(tenFeet);
  return tenFeetNativeUnits;
}                                

public CommandBase movingCommand(){
  
  return new CommandBase() {
    @Override
    public void initialize() {}

    @Override
    public void execute() {
      m_leftMaster.set(ControlMode.Position, moveTenFeet());
      m_rightMaster.set(ControlMode.Position, moveTenFeet());
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  };
}
