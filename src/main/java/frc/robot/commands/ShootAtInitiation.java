package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Superstructure;

public class ShootAtInitiation extends SequentialCommandGroup {

    public ShootAtInitiation(Drive drive, Superstructure superstructure) {
        addCommands(
            new JogShooter(superstructure.getShooter()),
            new TurnToTarget(drive, superstructure.getLimelight()),
            new AutonShoot(superstructure),
            new DriveForward(drive)
        );
    }
}