package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private VictorSPX mIntake;

    public Intake() {
        mIntake = new VictorSPX(IntakeConstants.kIntakePort);
    }

    public void runIntake(double power) {
        mIntake.set(ControlMode.PercentOutput, power);
    }

    public void runConstant(boolean stop) {
        double power = IntakeConstants.kIntakePower;
        if (stop) {
            power = 0;
        }
        mIntake.set(ControlMode.PercentOutput, power);
    }
}