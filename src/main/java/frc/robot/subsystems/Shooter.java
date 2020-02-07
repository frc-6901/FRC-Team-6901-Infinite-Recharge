package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

 
public class Shooter extends SubsystemBase {
    private CANSparkMax mBottomMotor;
    private CANSparkMax mTopMotor;
    private CANEncoder mBottomEncoder, mTopEncoder;
    private CANPIDController mBottomPID, mTopPID;
    //private double kP;

    //private Spark shooterMotor2;

  /**
   * Creates Shooter Subsystem.
   */
  public Shooter() {

    // Motor Setup
    mBottomMotor = new CANSparkMax(Constants.shooterPort1, MotorType.kBrushless);
    mTopMotor = new CANSparkMax(Constants.shooterPort2, MotorType.kBrushless);
    
    // Makes sure if it was successful
    resetMotors();
    setIdleMode(CANSparkMax.IdleMode.kBrake);

    // Sensor and PID Controller Setup
    mTopEncoder = mTopMotor.getEncoder();
    mBottomEncoder = mBottomMotor.getEncoder();

    mBottomPID = mBottomMotor.getPIDController();
    mTopPID = mBottomMotor.getPIDController();


    // Sets the PID Values
    mBottomPID.setP(ShooterConstants.kP);
    mBottomPID.setI(ShooterConstants.kI);
    mBottomPID.setD(ShooterConstants.kD);
    mBottomPID.setFF(ShooterConstants.kF);
    mBottomPID.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);

    mTopPID.setP(ShooterConstants.kP);
    mTopPID.setI(ShooterConstants.kI);
    mTopPID.setD(ShooterConstants.kD);
    mTopPID.setFF(ShooterConstants.kF);
    mTopPID.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput, 0);
    


    SmartDashboard.putNumber("SetPoint", 0);
    SmartDashboard.putNumber("Max Output", ShooterConstants.kMaxOutput);
    SmartDashboard.putNumber("Min Output", ShooterConstants.kMinOutput);
    SmartDashboard.putNumber("Velocity Setpoint", 0);

  }

  private void resetMotors() {
    CANError bottomMotorStatus = mBottomMotor.restoreFactoryDefaults();
    CANError topMotorStatus = mTopMotor.restoreFactoryDefaults();

    if (bottomMotorStatus != CANError.kOk) {
      System.out.println("Bottom Shooter motor failed!" + bottomMotorStatus);
    }

    if (topMotorStatus != CANError.kOk) {
      System.out.println("Top Shooter motor failed!" + topMotorStatus);
    }
  }

  private void setIdleMode(CANSparkMax.IdleMode idleMode) {
    CANError bottomMotorStatus = mBottomMotor.setIdleMode(idleMode);
    CANError topMotorStatus = mTopMotor.setIdleMode(idleMode);

    if (bottomMotorStatus != CANError.kOk) {
      System.out.println("Bottom Shooter idle mode setup failed!" + bottomMotorStatus);
    }

    if (topMotorStatus != CANError.kOk) {
      System.out.println("Top Shooter idle mode setup failed!" + topMotorStatus);
    }
  }

  
  // OPEN LOOP METHODS
  public void runOpenLoop(double power) {
      mBottomMotor.set(power);
      mTopMotor.set(power);
  }

  public void runVariableOpenLoop(double power) {
    mBottomMotor.set(power);
    mTopMotor.set(power - .1);
  }

  public void stopShooter() {
    mBottomMotor.set(0);
    mTopMotor.set(0);

  }

  // CLOSED LOOP METHODS

  public void RPMShooter(double RPM) {
    SmartDashboard.putNumber("Velocity Setpoint", RPM);
    mBottomPID.setReference(RPM, ControlType.kVelocity, 0);
    mTopPID.setReference(RPM, ControlType.kVelocity, 0);

  }

  public void variableRPMShooter(double RPM) {

    SmartDashboard.putNumber("Velocity Setpoint", RPM);
    mBottomPID.setReference(RPM, ControlType.kVelocity);
    
    double RPM2 = Math.abs(RPM) - ShooterConstants.kRPMDifference;
    if (RPM2 < 0) {
      RPM2 *= -1;
    } 
    mTopPID.setReference(RPM2, ControlType.kVelocity);
  }

  public void tuningRPMShooter(double defaultRPM) {
    double RPMTarget = SmartDashboard.getNumber("Velocity Setpoint", defaultRPM);

    mBottomPID.setReference(RPMTarget, ControlType.kVelocity);
    mTopPID.setReference(RPMTarget, ControlType.kVelocity);
  }


  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Bottom Shooter RPM", mBottomEncoder.getVelocity());
    SmartDashboard.putNumber("Top Shooter RPM", mTopEncoder.getVelocity());
  }
}
