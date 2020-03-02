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

  

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(mLeftMotorMaster, mRightMotorMaster);



  /**
   * Creates a new DriveSubsystem.
   */
  public Drive() {

    mRightMotorSlave.follow(mRightMotorMaster);
    mLeftMotorSlave.follow(mLeftMotorMaster);

    mRightMotorMaster.configPeakCurrentLimit(DriveConstants.kMaxCurrent);
    mLeftMotorMaster.configPeakCurrentLimit(DriveConstants.kMaxCurrent);



  }

  @Override
  public void periodic() {

    

  }

  /**


  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }


}