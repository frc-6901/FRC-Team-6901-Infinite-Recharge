package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {
    private final TalonSRX mWristMotor = new TalonSRX(WristConstants.kWristMotorPort);

    public Wrist() {
        mWristMotor.configFactoryDefault();
        mWristMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

        mWristMotor.config_kP(0, WristConstants.kP);
        mWristMotor.config_kI(0, WristConstants.kI);
        mWristMotor.config_kD(0, WristConstants.kD);
        mWristMotor.config_kF(0, WristConstants.kF);
        mWristMotor.configMotionCruiseVelocity(WristConstants.kV);
        mWristMotor.configMotionAcceleration(WristConstants.kA);

        mWristMotor.configForwardSoftLimitThreshold(WristConstants.kSoftLimitForward);
        mWristMotor.configReverseSoftLimitThreshold(WristConstants.kSoftLimitReverse);

        mWristMotor.configForwardSoftLimitEnable(true);
        mWristMotor.configReverseSoftLimitEnable(true);


    }

    // TODO Add something to reduce redundant commands

    // TODO Figure out how to use absolute value

    /** Open Loop Methods */

    public void moveWrist(double power) {
        mWristMotor.set(ControlMode.PercentOutput, power);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Position", mWristMotor.getSelectedSensorPosition());
    }


}