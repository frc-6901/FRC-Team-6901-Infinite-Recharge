package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private Spark shooterMotor1;
    //private Spark shooterMotor2;

  /**
   * Creates a new ExampleSubsystem.
   */
  public Shooter() {
    shooterMotor1 = new Spark(Constants.shooterPort1);
    //shooterMotor2 = new Spark(Constants.shooterPort2);
  }

  public void runShooter(double power) {
      shooterMotor1.set(power);
      //shooterMotor2.set(power);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
