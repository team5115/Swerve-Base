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

package frc.team5115;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final Mode currentMode = Mode.REAL;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final byte INTAKE_MOTOR_ID = 32;
    public static final byte SNOWBLOWER_MOTOR_ID = 21;
    public static final byte ARM_LEFT_MOTOR_ID = 3;
    public static final byte ARM_RIGHT_MOTOR_ID = 33;
    public static final byte SHOOTER_RIGHT_MOTOR_ID = 27;
    public static final byte SHOOTER_LEFT_MOTOR_ID = 20;
    public static final byte SHOTER_AUX_MOTOR_ID = 35;
    public static final byte CLIMBER_LEFT_MOTOR_ID = 30;
    public static final byte CLIMBER_RIGHT_MOTOR_ID = 31;

    public static final byte SHOOTER_SENSOR_ID = 0;
    public static final byte CLIMB_LEFT_SENSOR_ID = 8;
    public static final byte CLIMB_RIGHT_SENSOR_ID = 9;

    public static final double LOOP_PERIOD_SECS = 0.02;

    public static class SwerveConstants {
        public static final byte FRONT_LEFT_DRIVE_ID = 4;
        public static final byte FRONT_RIGHT_DRIVE_ID = 22;
        public static final byte BACK_LEFT_DRIVE_ID = 24;
        public static final byte BACK_RIGHT_DRIVE_ID = 26;

        public static final byte FRONT_LEFT_TURN_ID = 29;
        public static final byte FRONT_RIGHT_TURN_ID = 28;
        public static final byte BACK_LEFT_TURN_ID = 23;
        public static final byte BACK_RIGHT_TURN_ID = 25;

        public static final Rotation2d FRONT_LEFT_ANGULAR_OFFSET = Rotation2d.fromDegrees(90);
        public static final Rotation2d FRONT_RIGHT_ANGULAR_OFFSET = Rotation2d.fromDegrees(0);
        public static final Rotation2d BACK_LEFT_ANGULAR_OFFSET = Rotation2d.fromDegrees(180);
        public static final Rotation2d BACK_RIGHT_ANGULAR_OFFSET = Rotation2d.fromDegrees(270);

        public static final double MAX_LINEAR_SPEED = 4.8; // meters per second
        public static final double TRACK_WIDTH_X = Units.inchesToMeters(23.75);
        public static final double TRACK_WIDTH_Y = Units.inchesToMeters(23.75);
        public static final double DRIVE_BASE_RADIUS =
                Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
        public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.5);

        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear
        // 15 teeth on the bevel pinion, 13 teeth on the driving motor
        public static final double DrivingMotorReduction = (45.0 * 22.0) / (13.0 * 15.0);

        public static final int DrivingMotorCurrentLimit = 40; // amps
        public static final int TurningMotorCurrentLimit = 20; // amps
    }
}
