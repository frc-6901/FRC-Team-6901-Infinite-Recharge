package frc.robot.subsystems;

import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//TODO Integrate sparks onto CAN and figure out velocity 
public class Shooter extends SubsystemBase {
    private SpeedControllerGroup shooterGroup;
    private CANSparkMax shooterMotor1;
    private CANSparkMax shooterMotor2;
    // TODO Figure out Velocity PID, Hook up to smart dash
    //private Spark shooterMotor2;

  /**
   * Creates a new ExampleSubsystem.
   */
  public Shooter() {
    shooterMotor1 = new CANSparkMax(Constants.shooterPort1, MotorType.kBrushless);
    shooterMotor2 = new CANSparkMax(Constants.shooterPort2, MotorType.kBrushless);
    CANError status = shooterMotor2.follow(shooterMotor1);
    if (status == CANError.kOk) {
      System.out.println("Shooter Follower setup successfully!");
    } else {
      System.out.println("Shooter Follower Failed");
    }
  }

  

  public void runManualShooter(double power) {
      shooterMotor1.set(power);
      //shooterMotor2.set(power);
  }


  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Shooter RPM", shooterMotor.getSpeed());
  }
}
