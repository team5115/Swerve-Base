package frc.team5115.commands;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.arm.Arm;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.shooter.Shooter;

public class AutoCommands {
    private AutoCommands() {}

    public static void registerCommands(Drivetrain drivetrain, Shooter shooter, Arm arm) {
        NamedCommands.registerCommand(
                "InitialShoot",
                DriveCommands.prepareShoot(shooter, arm, 15, false)
                        .andThen(DriveCommands.triggerShoot(shooter)));
        NamedCommands.registerCommand("Intake", DriveCommands.intakeUntilNote(shooter, arm));
        NamedCommands.registerCommand("Shoot", DriveCommands.triggerShoot(shooter));
        NamedCommands.registerCommand(
                "PrepareClose", DriveCommands.prepareShoot(shooter, arm, 15, true));
    }
}
