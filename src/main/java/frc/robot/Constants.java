package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import SushiFrcLib.Swerve.SwerveModuleConstants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.util.SwerveKinematics;
import frc.robot.util.Vector;

public final class Constants {
    public static enum RobotName {
        NEO_SWERVE,FALCON_SWERVE;
    }

    public static final RobotName NAME = RobotName.FALCON_SWERVE;

    public static class kPorts {
        public static final String CANIVORE_NAME = "Sussy Squad";
        public static final int PIGEON_ID = 13;
    }

    public static final class kOI {
        public static final int DRIVE_TRANSLATION_Y = XboxController.Axis.kLeftY.value;
        public static final int DRIVE_TRANSLATION_X = XboxController.Axis.kLeftX.value;
        public static final int DRIVE_ROTATE = XboxController.Axis.kRightX.value;

        public static final int UPDATE_ENCODER = XboxController.Button.kY.value;

        public static final int DRIVE_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }

    public static final class kSwerve {
        public static final boolean OPEN_LOOP = false;
        public static final boolean FEILD_RELATIVE = false;

        public static final double SPEED_MULTIPLER = 1.0;

        public static final boolean GYRO_INVERSION = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(NAME == RobotName.NEO_SWERVE ? 19.5 : 21.73); // Falcon swerve: 21.73
        public static final double WHEEL_BASE = Units.inchesToMeters(NAME == RobotName.NEO_SWERVE ? 19.5 : 21.73);
        public static final double WHEEL_DIAMATER = Units.inchesToMeters(4);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMATER * Math.PI;

        public static final double OPEN_LOOP_RAMP = 0.25;

        public static final double DRIVE_GEAR_RATIO = 6.75 / 1.0; // 6.86:1
        public static final double ANGLE_GEAR_RATIO = (150.0 / 7.0); // 12.8:1

        // Neo Swerve Constants
        public static final double DRIVE_ROTATIONS_TO_METERS = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;
        public static final double DRIVE_RPM_TO_METERS_PER_SECOND = DRIVE_ROTATIONS_TO_METERS / 60.0;
        public static final double ANGLE_ROTATIONS_TO_RADIANS = (2 * Math.PI) / ANGLE_GEAR_RATIO;
        public static final double ANGLE_RPM_TO_RADIANS_PER_SECOND = DRIVE_ROTATIONS_TO_METERS / 60.0;

        public static final SwerveKinematics SWERVE_KINEMATICS = new SwerveKinematics(
                new Vector(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Vector(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Vector(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Vector(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Swerve Current Limiting */
        public static final int ANGLE_CURRENT_LIMIT = 20;
        public static final int DRIVE_CURRENT_LIMIT = 40;

        /* Angle Motor PID Values */
        public static final double ANGLE_P = 0.3;
        public static final double ANGLE_I = 0.0;
        public static final double ANGLE_D = 12.0;
        public static final double ANGLE_F = 0.0;

        /* Drive Motor PID Values */
        public static final double DRIVE_P = 0.009000;
        public static final double DRIVE_I = 0.0;
        public static final double DRIVE_D = 0.0;
        public static final double DRIVE_F = 0.046;

        /* Swerve Profiling Values */
        public static final double MAX_SPEED = 10; // 4.5 meters per second
        public static final double MAX_ACCELERATION = 4; // 2
        public static final double MAX_ANGULAR_VELOCITY = 10; // 11.5
        public static final double MAX_ANGULAR_ACCELERATION = 20; // 11.5

        /* Neutral Modes */
        public static final NeutralMode ANGLE_NEUTRAL_MODE = NeutralMode.Coast;
        public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

        public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kCoast;
        public static final IdleMode DRIVE_IDEL_MODE = IdleMode.kBrake;

        /* Motor Inverts */
        public static final boolean DRIVE_INVERSION = false;
        public static final boolean ANGLE_INVERSION = true; // make false if we have a stroke

        /* Angle Encoder Invert */
        public static final boolean CANCODER_INVERSION = true;

        /** Front Left Module - Module 0. */
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 1;
            public static final int ANGLE_MOTOR_ID = 3;
            public static final int CAN_CODER_ID = 2;
            public static final double ANGLE_OFFSET = 2.109375 + 180;

            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(
                DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET
            );
        }

        /** Front Right Module - Module 1. */
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 10;
            public static final int ANGLE_MOTOR_ID = 12;
            public static final int CAN_CODER_ID = 11;
            public static final double ANGLE_OFFSET = 229.130859 - 180;

            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(
                DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET
            );
        }

        /** Back Left Module - Module 2. */
        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 4;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final int CAN_CODER_ID = 5;
            public static final double ANGLE_OFFSET= 16.083984 + 180;
            
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(
                DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET
            );
        }

        /** Back Right Module - Module 3. */
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 7;
            public static final int ANGLE_MOTOR_ID = 9;
            public static final int CAN_CODER_ID = 8;
            public static final double ANGLE_OFFSET = 303.574219 - 180;

            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(
                DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET
            );
        }

        public static final class kNeoSwerve {
            public static final SwerveModuleConstants MOD_0_Constants = new SwerveModuleConstants(
            1,
            2,
            3,
            330.8203
            );

            public static final SwerveModuleConstants MOD_1_Constants = new SwerveModuleConstants(
            4,
            5,
            6,
            353.32  
            );

            public static final SwerveModuleConstants MOD_2_Constants = new SwerveModuleConstants(
            7,
            8,
            9,
            338.115
            );

            public static final SwerveModuleConstants MOD_3_Constants = new SwerveModuleConstants(
            10,
            11,
            12,
            27.949
            );
        }
    }

    public static final double STICK_DEADBAND = 0.1;
}