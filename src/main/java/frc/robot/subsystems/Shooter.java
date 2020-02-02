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
    private double kP;

    //private Spark shooterMotor2;

  /**
   * Creates Shooter Subsystem.
   */
  public Shooter() {
    mBottomMotor = new CANSparkMax(Constants.shooterPort1, MotorType.kBrushless);
    mTopMotor = new CANSparkMax(Constants.shooterPort2, MotorType.kBrushless);
    
    // Makes sure if it was successful
    CANError bottomStatus = mBottomMotor.restoreFactoryDefaults();
    CANError topStatus = mTopMotor.restoreFactoryDefaults();

    mTopEncoder = mTopMotor.getEncoder();
    mBottomEncoder = mBottomMotor.getEncoder();

    mBottomPID = mBottomMotor.getPIDController();
    mTopPID = mBottomMotor.getPIDController();

    kP = ShooterConstants.kP;

    mBottomPID.setP(kP);
    mBottomPID.setOutputRange(ShooterConstants.kMaxOutput, ShooterConstants.kMinOutput);

    mTopPID.setP(kP);
    mTopPID.setOutputRange(ShooterConstants.kMaxOutput, ShooterConstants.kMinOutput);

    if (bottomStatus == CANError.kOk && topStatus == CANError.kOk) {
      System.out.println("Shooter setup successful!");
    } else {
      System.out.println("Shooter setup Failed");
    }

    SmartDashboard.putNumber("SetPoint", 0);
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("Max Output", ShooterConstants.kMaxOutput);
    SmartDashboard.putNumber("Min Output", ShooterConstants.kMinOutput);
    SmartDashboard.putNumber("Velocity Setpoint", 0);

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
    mBottomPID.setReference(RPM, ControlType.kVelocity);
    mTopPID.setReference(RPM, ControlType.kVelocity);

  }

  public void variableRPMShooter(double RPM) {

    SmartDashboard.putNumber("Velocity Setpoint", RPM);
    mBottomPID.setReference(RPM, ControlType.kVelocity);
    mTopPID.setReference(RPM - ShooterConstants.kRPMDifference, ControlType.kVelocity);
  }

  public void tuningRPMShooter(double defaultRPM) {
    double RPMTarget = SmartDashboard.getNumber("Velocity Setpoint", defaultRPM);
    double p = SmartDashboard.getNumber("P Gain", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);


    if((p != kP)) { 
      mBottomPID.setP(p);
      mTopPID.setP(p);
    }

    if((max != ShooterConstants.kMaxOutput) || (min != ShooterConstants.kMinOutput)) { 
      mBottomPID.setOutputRange(min, max); 
      mTopPID.setOutputRange(min, max);
    }

    SmartDashboard.putNumber("SetPoint", RPMTarget);

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
