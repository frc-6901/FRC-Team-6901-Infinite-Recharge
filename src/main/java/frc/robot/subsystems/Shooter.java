package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private Spark shooterMotor;
  /**
   * Creates a new ExampleSubsystem.
   */
  public Shooter() {
    shooterMotor = new Spark(Constants.shooterPort);
  }

  public void runShooter(double power) {
      shooterMotor.set(power);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
