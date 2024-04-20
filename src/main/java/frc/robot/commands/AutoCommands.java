package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;

public class AutoCommands {
    private AutoCommands() {}

    public static void registerCommands(Drivetrain drivetrain, Shooter shooter) {
        NamedCommands.registerCommand(
                "InitialShoot",
                DriveCommands.prepareShoot(shooter, 15, false)
                        .andThen(DriveCommands.triggerShoot(shooter)));
        NamedCommands.registerCommand("Intake", DriveCommands.intakeUntilNote(shooter));
        NamedCommands.registerCommand("Shoot", DriveCommands.triggerShoot(shooter));
        NamedCommands.registerCommand("PrepareClose", DriveCommands.prepareShoot(shooter, 15, true));
    }
}
