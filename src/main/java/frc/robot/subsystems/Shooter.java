package frc.robot.subsystems;

import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

 
public class Shooter extends SubsystemBase {
    private CANSparkMax mBottomMotor;
    private CANSparkMax mTopMotor;
    private SimpleMotorFeedforward mMotorFeedForward;


  /**
   * Creates Shooter Subsystem.
   */
  public Shooter() {
    SmartDashboard.putNumber("output", 0);
    // Motor Setup
    mBottomMotor = new CANSparkMax(ShooterConstants.kShooterIdBottom, MotorType.kBrushless);
    mTopMotor = new CANSparkMax(ShooterConstants.kShooterIdTop, MotorType.kBrushless);
    resetMotors();
    setIdleMode(CANSparkMax.IdleMode.kBrake);

    mMotorFeedForward = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV);

    SmartDashboard.putNumber("Velocity Setpoint", 0);

  }

  private void resetMotors() {
    CANError bottomMotorStatus = mBottomMotor.restoreFactoryDefaults();
    CANError topMotorStatus = mTopMotor.restoreFactoryDefaults();

    // Makes sure if the factory resets worked
    if (bottomMotorStatus != CANError.kOk) {
      System.out.println("Bottom Shooter motor failed!" + bottomMotorStatus);
    }

    if (topMotorStatus != CANError.kOk) {
      System.out.println("Top Shooter motor failed!" + topMotorStatus);
    }
  }


  // Makes sure if the idlemode setting worked
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

  /**
   * PID Command that uses WPILib's Simple Motor Feedforward (without accel)
   * To control motor PID 
   * @param motor the SPARKMax motor controller that you would like to control 
   * @param RPM the desired RPM
   */
  private void PIDShooterMotor(CANSparkMax motor, double RPM) {
    double setpoint = RPMToRPS(RPM);
    double error = setpoint - RPMToRPS(motor.getEncoder().getVelocity());
    double feedforward = mMotorFeedForward.calculate(setpoint);
    
    double output = feedforward + error*ShooterConstants.kP;
    motor.setVoltage(output);
  }

  /**
   * Converts an Rotations Per minute value into rotations per second
   * @param RPM the RPM Value to convert
   * @return the Rotations per second of that value
   */
  private double RPMToRPS(double RPM) {
    return RPM / 60;
  }


  /**
   * Sets the top and bottom shooter 
   * to a certain RPM
   * @param RPM the desired RPM
   */
  public void RPMShooter(double RPM) {
    PIDShooterMotor(mTopMotor, RPM);
    PIDShooterMotor(mBottomMotor, RPM);
  }

  /**
   * Sets the top shooter to be 
   * slower than the bottom shooter
   * in order to have more spin on the ball
   * @param RPM the desired bottom rpm
   */
  public void variableRPMShooter(double RPM) {

    SmartDashboard.putNumber("Velocity Setpoint", RPM);
    PIDShooterMotor(mBottomMotor, RPM);
    
    // Reduces RPM while maintaining signs
    double RPM2 = Math.abs(RPM) - ShooterConstants.kRPMDifference;
    if (RPM < 0) {
      RPM2 *= -1;
    } 
    PIDShooterMotor(mTopMotor, RPM2);
  }



  /**
   * This method sets motor speeds to a velocity 
   * specified in the Smart Dashboard in order to 
   * facilitate tuning the shooter from certain distances
   * @param defaultRPM the default Rotation per minute
   */
  public void tuningRPMShooter(double defaultRPM) {
    double RPMTarget = SmartDashboard.getNumber("Velocity Setpoint", defaultRPM);
    RPMShooter(RPMTarget);
  }


  
  
  @Override
  public void periodic() {
    // Mostly to update numbers
    SmartDashboard.putNumber("Bottom Shooter RPM", mBottomMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Top Shooter RPM", mTopMotor.getEncoder().getVelocity());
  }
}
