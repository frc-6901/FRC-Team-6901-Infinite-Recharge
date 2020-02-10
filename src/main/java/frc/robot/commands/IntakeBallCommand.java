package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class IntakeBallCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake mIntake;
  private final boolean intakingState;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeBallCommand(Intake intake, boolean intaking) {
    mIntake = intake;
    intakingState = intaking;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      mIntake.runIntake(intakingState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      mIntake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}