package frc.robot.subsystems;

import com.revrobotics.SparkMax;

import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    //private SparkMax shooterMotor1;
    private PWMSparkMax shooterMotor;
    //private Spark shooterMotor2;

  /**
   * Creates a new ExampleSubsystem.
   */
  public Shooter() {
    shooterMotor = new PWMSparkMax(Constants.shooterPort1);
    //shooterMotor2 = new Spark(Constants.shooterPort2);
  }

  public void runShooter(double power) {
      shooterMotor.set(power);
      //shooterMotor2.set(power);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter RPM", shooterMotor.getSpeed());
  }
}
