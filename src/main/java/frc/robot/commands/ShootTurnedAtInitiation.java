package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Superstructure;

public class ShootTurnedAtInitiation extends SequentialCommandGroup {

    public ShootTurnedAtInitiation(Drive drive, Superstructure superstructure) {
        addCommands(
            new JogShooter(superstructure.getShooter()),
            new TurnToTarget(drive, superstructure.getLimelight()),
            new AutonLimelightShoot(superstructure),
            new DriveForward(drive)
        );
    }
}