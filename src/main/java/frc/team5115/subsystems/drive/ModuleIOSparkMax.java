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

package frc.team5115.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.team5115.Constants.SwerveConstants;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMax implements ModuleIO {
    private final CANSparkMax driveSparkMax;
    private final CANSparkMax turnSparkMax;

    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder turnEncoder;

    private final Rotation2d absoluteEncoderOffset;

    public ModuleIOSparkMax(int index) {
        int driveId = -1;
        int turnId = -1;

        switch (index) {
            case 0: // Front Left
                driveId = SwerveConstants.FRONT_LEFT_DRIVE_ID;
                turnId = SwerveConstants.FRONT_LEFT_TURN_ID;
                absoluteEncoderOffset = SwerveConstants.FRONT_LEFT_ANGULAR_OFFSET;
                break;
            case 1: // Front Right
                driveId = SwerveConstants.FRONT_RIGHT_DRIVE_ID;
                turnId = SwerveConstants.FRONT_RIGHT_TURN_ID;
                absoluteEncoderOffset = SwerveConstants.FRONT_RIGHT_ANGULAR_OFFSET;
                break;
            case 2: // Back Left
                driveId = SwerveConstants.BACK_LEFT_DRIVE_ID;
                turnId = SwerveConstants.BACK_LEFT_TURN_ID;
                absoluteEncoderOffset = SwerveConstants.BACK_LEFT_ANGULAR_OFFSET;
                break;
            case 3: // Back Right
                driveId = SwerveConstants.BACK_RIGHT_DRIVE_ID;
                turnId = SwerveConstants.BACK_RIGHT_TURN_ID;
                absoluteEncoderOffset = SwerveConstants.BACK_RIGHT_ANGULAR_OFFSET;
                break;
            default:
                throw new RuntimeException("Invalid module index");
        }

        driveSparkMax = new CANSparkMax(driveId, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(turnId, MotorType.kBrushless);

        driveSparkMax.restoreFactoryDefaults();
        turnSparkMax.restoreFactoryDefaults();

        driveSparkMax.setCANTimeout(250);
        turnSparkMax.setCANTimeout(250);

        driveEncoder = driveSparkMax.getEncoder();
        turnEncoder = turnSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        turnSparkMax.setInverted(false);

        driveSparkMax.setSmartCurrentLimit(SwerveConstants.DrivingMotorCurrentLimit);
        turnSparkMax.setSmartCurrentLimit(SwerveConstants.TurningMotorCurrentLimit);
        driveSparkMax.enableVoltageCompensation(12.0);
        turnSparkMax.enableVoltageCompensation(12.0);

        driveEncoder.setPosition(0.0);
        driveEncoder.setMeasurementPeriod(10);
        driveEncoder.setAverageDepth(2);

        turnEncoder.setAverageDepth(2);

        driveSparkMax.setIdleMode(IdleMode.kBrake);
        turnSparkMax.setIdleMode(IdleMode.kBrake);

        driveSparkMax.burnFlash();
        turnSparkMax.burnFlash();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePositionRad =
                Units.rotationsToRadians(
                        driveEncoder.getPosition() / SwerveConstants.DrivingMotorReduction);
        inputs.driveVelocityRadPerSec =
                Units.rotationsPerMinuteToRadiansPerSecond(
                        driveEncoder.getVelocity() / SwerveConstants.DrivingMotorReduction);
        inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
        inputs.driveCurrentAmps = driveSparkMax.getOutputCurrent();
        inputs.turnAbsolutePosition =
                Rotation2d.fromRotations(-turnEncoder.getPosition()).minus(absoluteEncoderOffset);
        inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnEncoder.getVelocity());
        inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
        inputs.turnCurrentAmps = turnSparkMax.getOutputCurrent();
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveSparkMax.setVoltage(volts);
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnSparkMax.setVoltage(volts);
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setTurnBrakeMode(boolean enable) {
        turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }
}
