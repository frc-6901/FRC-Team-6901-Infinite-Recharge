package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private VictorSPX mIntakeMotor = new VictorSPX(IntakeConstants.kIntakeMotorPort);

    public Intake() {
        mIntakeMotor.configFactoryDefault();
    }

    public void runIntake(boolean intaking) {
        double power = IntakeConstants.kIntakePower;
        mIntakeMotor.set(ControlMode.PercentOutput, power * (intaking ? 1 : -1));
    }

    public void stopIntake() {
        mIntakeMotor.set(ControlMode.PercentOutput, 0);
    }
}