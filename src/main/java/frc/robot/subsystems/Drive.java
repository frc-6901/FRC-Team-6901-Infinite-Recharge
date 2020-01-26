package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase {
  private final WPI_TalonSRX mLeftMotorMaster = new WPI_TalonSRX(DriveConstants.kLeftMotor1Port);
  private final WPI_TalonSRX mRightMotorMaster = new WPI_TalonSRX(DriveConstants.kRightMotor1Port);

  private final WPI_VictorSPX mLeftMotorSlave = new WPI_VictorSPX(DriveConstants.kLeftMotor2Port);
  private final WPI_VictorSPX mRightMotorSlave = new WPI_VictorSPX(DriveConstants.kRightMotor2Port);
  // The motors on the left side of the drive.
  private final SpeedControllerGroup m_leftMotors =
      new SpeedControllerGroup(mLeftMotorMaster,mLeftMotorSlave);

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors =
      new SpeedControllerGroup(mRightMotorMaster, mRightMotorSlave);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  

  // The gyro sensor
   private final Pigeon mGyro = new Pigeon(Constants.DriveConstants.kPigeonPort);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /**
   * Creates a new DriveSubsystem.
   */
  public Drive() {

    mRightMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    mLeftMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    resetEncoders();
    //mGyro.reset();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    double[] encoderDistances = getDistances();
    double heading = getHeading(); 
    m_odometry.update(Rotation2d.fromDegrees(heading), encoderDistances[1],
                      encoderDistances[0]);
    // System.out.println(getHeading() + ", " +  getDistances()[1] + ", " + getDistances()[0]);

    SmartDashboard.putNumber("Left Encoder", encoderDistances[1]);
    SmartDashboard.putNumber("Right Encoder", encoderDistances[0]);
    SmartDashboard.putNumber("Heading", heading);
    

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getEncoderValues()[3], getEncoderValues()[1]);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    mLeftMotorMaster.setSelectedSensorPosition(0);
    mRightMotorMaster.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    double[] distances = getDistances();
    
    return (distances[0] + distances[1]) / 2.0;
  }


  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    mGyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(mGyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  // TODO maybe add a cleaner data structure
  public int[] getEncoderValues() {
      int[] encoderValues = new int[4];
      
      // position 1
      encoderValues[0] = -mRightMotorMaster.getSelectedSensorPosition();
      encoderValues[1] = -mRightMotorMaster.getSelectedSensorVelocity();
      // position 2
      encoderValues[2] = mLeftMotorMaster.getSelectedSensorPosition();
      encoderValues[3] = mLeftMotorMaster.getSelectedSensorVelocity();
      return encoderValues;
  }

  public double[] getDistances() {
      double[] distance = new double[2];
      int[] encoderValues = getEncoderValues();
      distance[0] = encoderValues[0] * DriveConstants.kEncoderDistancePerPulse;
      distance[1] = encoderValues[2] * DriveConstants.kEncoderDistancePerPulse;
      return distance;
      
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
//   public double getTurnRate() {
//     return mGyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
//   }
}