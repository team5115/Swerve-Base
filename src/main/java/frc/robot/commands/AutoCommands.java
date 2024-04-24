package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;

public class AutoCommands {
    private AutoCommands() {}

    public static void registerCommands(Drivetrain drivetrain, Shooter shooter, Arm arm) {
        NamedCommands.registerCommand(
                "InitialShoot",
                DriveCommands.prepareShoot(shooter, arm, 15, false)
                        .andThen(DriveCommands.triggerShoot(shooter)));
        NamedCommands.registerCommand("Intake", DriveCommands.intakeUntilNote(shooter, arm));
        NamedCommands.registerCommand("Shoot", DriveCommands.triggerShoot(shooter));
        NamedCommands.registerCommand("PrepareClose", DriveCommands.prepareShoot(shooter, arm, 15, true));
    }
}
