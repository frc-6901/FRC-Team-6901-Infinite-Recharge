package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
    private Intake mIntake;
    private boolean isIn;

    public RunIntake(Intake intake, boolean in) {
        mIntake = intake;
        isIn = in;
    }

    @Override
    public void initialize() {
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double power = IntakeConstants.kIntakePower;
        if (!isIn) {
            power *= -1;

        }
        mIntake.runIntake(power);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mIntake.runIntake(0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }

}