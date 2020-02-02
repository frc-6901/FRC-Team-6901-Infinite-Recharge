package frc.robot.subsystems;

import com.revrobotics.SparkMax;

import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

//TODO Integrate sparks onto CAN and figure out velocity 
public class Shooter extends SubsystemBase {
    private PWMSparkMax shooterMotor1;
    private PWMSparkMax shooterMotor2;
    //private Spark shooterMotor2;

  /**
   * Sets up the shooter with Spark Maxes plugged into PWM
   */
  public Shooter() {
    shooterMotor1 = new PWMSparkMax(ShooterConstants.shooterPort1);
    shooterMotor2 = new PWMSparkMax(ShooterConstants.shooterPort2);
    
  }

  public void runShooter(double power) {
    // Different speeds for spin
    shooterMotor1.set(power);  
    shooterMotor2.set(power-.1); 
    
  }

  public void stopShooer() {
    shooterMotor1.set(0);  
    shooterMotor2.set(0);

  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Shooter RPM", shooterMotor.getSpeed());
  }
}
