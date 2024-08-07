package frc.team5115.commands;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.team5115.subsystems.amper.Amper;
import frc.team5115.subsystems.arm.Arm;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.feeder.Feeder;
import frc.team5115.subsystems.intake.Intake;
import frc.team5115.subsystems.shooter.Shooter;
import frc.team5115.subsystems.vision.PhotonVision;

public class AutoCommands {
    /**
     * Registers commands for pathplanner to use in autos
     *
     * @param shooter the shooter subsystem
     * @param arm the arm subsystem
     * @param drivetrain the drivetrain subsytem (not currently used)
     * @param photonVision the photonvision subsystem (not currently used)
     */
    public static void registerCommands(
            Drivetrain drivetrain,
            PhotonVision vision,
            Arm arm,
            Amper amper,
            Intake intake,
            Feeder feeder,
            Shooter shooter) {
        NamedCommands.registerCommand(
                "InitialShoot",
                arm.goToAngle(Rotation2d.fromDegrees(15), 1)
                        .alongWith(new SpinUpShooter(shooter, 5000, false))
                        .andThen(feeder.feed()));
        NamedCommands.registerCommand("Intake", DriveCommands.intakeUntilNote(arm, intake, feeder));
        NamedCommands.registerCommand("Shoot", feeder.feed());
        NamedCommands.registerCommand(
                "PrepareClose", DriveCommands.prepareShoot(arm, intake, feeder, shooter, 15, false));
    }
}
