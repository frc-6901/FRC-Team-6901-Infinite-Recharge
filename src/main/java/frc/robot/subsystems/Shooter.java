package frc.robot.subsystems;

import com.revrobotics.SparkMax;

import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//TODO Integrate sparks onto CAN and figure out velocity 
public class Shooter extends SubsystemBase {
    private SpeedControllerGroup shooterGroup;
    private PWMSparkMax shooterMotor1;
    private PWMSparkMax shooterMotor2;
    //private Spark shooterMotor2;

  /**
   * Creates a new ExampleSubsystem.
   */
  public Shooter() {
    shooterMotor1 = new PWMSparkMax(Constants.shooterPort1);
    shooterMotor2 = new PWMSparkMax(Constants.shooterPort2);
    shooterGroup = new SpeedControllerGroup(shooterMotor1, shooterMotor2);
  }

  public void runShooter(double power) {
      shooterGroup.set(power);
      //shooterMotor2.set(power);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Shooter RPM", shooterMotor.getSpeed());
  }
}
