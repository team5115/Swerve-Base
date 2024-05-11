package frc.team5115.commands;

import com.pathplanner.lib.auto.NamedCommands;
import frc.team5115.subsystems.arm.Arm;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.shooter.Shooter;
import frc.team5115.subsystems.vision.PhotonVision;

public class AutoCommands {
    private AutoCommands() {}

    /**
     * Registers commands for pathplanner to use in autos
     * @param shooter the shooter subsystem
     * @param arm the arm subsystem
     * @param drivetrain the drivetrain subsytem (not currently used)
     * @param photonVision the photonvision subsystem (not currently used)
     */
    public static void registerCommands(Shooter shooter, Arm arm, Drivetrain drivetrain, PhotonVision photonVision) {
        NamedCommands.registerCommand(
                "InitialShoot",
                DriveCommands.prepareShoot(shooter, arm, 15, false)
                        .andThen(DriveCommands.triggerShoot(shooter)));
        NamedCommands.registerCommand("Intake", DriveCommands.intakeUntilNote(shooter, arm));
        NamedCommands.registerCommand("Shoot", DriveCommands.triggerShoot(shooter));
        NamedCommands.registerCommand(
                "PrepareClose", DriveCommands.prepareShoot(shooter, arm, 15, false));
    }
}
