// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.team5115.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team5115.Constants.SwerveConstants;
import frc.team5115.subsystems.amper.Amper;
import frc.team5115.subsystems.arm.Arm;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.feeder.Feeder;
import frc.team5115.subsystems.intake.Intake;
import frc.team5115.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class DriveCommands {
    private static final double DEADBAND = 0.1;

    private DriveCommands() {}

    public static Command intakeUntilNote(Arm arm, Intake intake, Feeder feeder) {
        return Commands.sequence(
                        arm.goToAngle(Rotation2d.fromDegrees(0), 1),
                        intake.intake(),
                        feeder.centerNote(),
                        feeder.waitForDetectionState(true, 20),
                        Commands.waitSeconds(0.25),
                        intake.stop(),
                        feeder.stop())
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public static Command prepareAmp(Arm arm, Amper amper, Intake intake, Feeder feeder) {
        return Commands.sequence(
                        arm.goToAngle(Rotation2d.fromDegrees(103.5), 1),
                        new SpinAmper(amper, new Rotation2d(3.25)).withTimeout(5),
                        intake.setSpeed(1),
                        feeder.setSpeeds(0.25),
                        Commands.waitSeconds(0.8),
                        feeder.stop(),
                        intake.setSpeed(-0.9))
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public static Command triggerAmp(Arm arm, Amper amper, Intake intake, Feeder feeder) {
        return Commands.sequence(
                        intake.setSpeed(-0.9),
                        feeder.setSpeeds(-1),
                        feeder.waitForDetectionState(true, 1),
                        feeder.waitForDetectionState(false, 1),
                        Commands.waitSeconds(0.22),
                        intake.stop(),
                        feeder.stop(),
                        Commands.waitSeconds(0.5),
                        new SpinAmper(amper, new Rotation2d(0.2)).alongWith(arm.stow()).withTimeout(5))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public static Command prepareShoot(
            Arm arm, Intake intake, Feeder feeder, Shooter shooter, double angle, boolean neverExit) {
        return Commands.parallel(
                        intake.stop(),
                        feeder.stop(),
                        arm.goToAngle(Rotation2d.fromDegrees(angle), 1),
                        new SpinUpShooter(shooter, 5000, neverExit))
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public static Command vomit(Intake intake, Feeder feeder, Shooter shooter) {
        return Commands.runOnce(
                () -> {
                    intake.vomit();
                    feeder.vomit();
                    shooter.vomit();
                });
    }

    public static Command forceStop(Intake intake, Feeder feeder, Shooter shooter) {
        return Commands.runOnce(
                () -> {
                    intake.forceStop();
                    feeder.forceStop();
                    shooter.forceStop();
                });
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    public static Command joystickDrive(
            Drivetrain drivetrain,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {
        return Commands.run(
                () -> {
                    // Apply deadband
                    double linearMagnitude =
                            MathUtil.applyDeadband(
                                    Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
                    Rotation2d linearDirection =
                            new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                    // Square values
                    linearMagnitude = linearMagnitude * linearMagnitude;
                    omega = Math.copySign(omega * omega, omega);

                    // Calcaulate new linear velocity
                    Translation2d linearVelocity =
                            new Pose2d(new Translation2d(), linearDirection)
                                    .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                                    .getTranslation();

                    // Convert to field relative speeds & send command
                    drivetrain.runVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    linearVelocity.getX() * SwerveConstants.MAX_LINEAR_SPEED,
                                    linearVelocity.getY() * SwerveConstants.MAX_LINEAR_SPEED,
                                    omega * SwerveConstants.MAX_ANGULAR_SPEED,
                                    drivetrain.isRedAlliance()
                                            ? drivetrain.getRotation().plus(new Rotation2d(Math.PI))
                                            : drivetrain.getRotation()));
                },
                drivetrain);
    }
}
