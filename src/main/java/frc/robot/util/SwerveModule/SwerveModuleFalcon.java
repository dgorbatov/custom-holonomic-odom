package frc.robot.util.SwerveModule;

import SushiFrcLib.Math.Conversion;
import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.Swerve.SwerveModuleConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.kPorts;
import frc.robot.Constants.kSwerve;

/**
 * Falcon Swerve Module.
 */
public class SwerveModuleFalcon extends SwerveModule {
    private TalonFX angleMotor;
    private TalonFX driveMotor;
    private CANCoder angleEncoder;
    private double lastAngle;

    /**
     * Instantiate a swerve module with a number from 1-4.
     */
    public SwerveModuleFalcon(int moduleNumber, SwerveModuleConstants moduleConstants) {
        super(moduleNumber, moduleConstants);

        angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new WPI_CANCoder(moduleConstants.cancoderId, kPorts.CANIVORE_NAME);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = MotorHelper.createFalconMotor(moduleConstants.angleMotorId, 
                kSwerve.ANGLE_CURRENT_LIMIT,
                kSwerve.ANGLE_INVERSION, kSwerve.ANGLE_NEUTRAL_MODE, 
                kSwerve.ANGLE_P, kSwerve.ANGLE_I, kSwerve.ANGLE_D,
                kSwerve.ANGLE_F,
                kPorts.CANIVORE_NAME, SensorInitializationStrategy.BootToZero
        );
        resetToAbsolute();

        /* Drive Motor Config */
        driveMotor = MotorHelper.createFalconMotor(moduleConstants.driveMotorId, 
                kSwerve.DRIVE_CURRENT_LIMIT,
                kSwerve.DRIVE_INVERSION, kSwerve.DRIVE_NEUTRAL_MODE, 
                kSwerve.DRIVE_P, kSwerve.DRIVE_I, kSwerve.DRIVE_D,
                kSwerve.DRIVE_F,
                kPorts.CANIVORE_NAME, SensorInitializationStrategy.BootToZero, 
                kSwerve.OPEN_LOOP_RAMP
        );
        driveMotor.setSelectedSensorPosition(0);

        lastAngle = getState().angle.getDegrees();
    }

    /**
     * Set the state of the swerve module.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // TODO: test optimization code
        double targetAngle = placeInAppropriate0To360Scope(
            lastAngle, desiredState.angle.getDegrees()
        );

        double targetSpeed = desiredState.speedMetersPerSecond;

        double delta = targetAngle - lastAngle;

        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);        
        }

        driveMotor.set(ControlMode.Velocity, Conversion.MPSToFalcon(
            targetSpeed, 
            kSwerve.WHEEL_CIRCUMFERENCE,
            kSwerve.DRIVE_GEAR_RATIO
        ));
       
        // TODO: Test anti jitter code
        // anit jitter code
        // if (targetSpeed < kSwerve.MAX_SPEED * 0.02) {
        //     targetAngle = lastAngle;
        // }

        angleMotor.set(
            ControlMode.Position, 
            Conversion.degreesToFalcon(
                targetAngle, 
                kSwerve.ANGLE_GEAR_RATIO
            )
        );

        lastAngle = targetAngle;
    }

    // Stolen from CTREModuleState.java in SushiFrcLib
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

    @Override
    public void resetToAbsolute() {
        double absolutePosition = Conversion.degreesToFalcon(getAngle(), kSwerve.ANGLE_GEAR_RATIO);
        angleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        angleEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        angleEncoder.configSensorDirection(kSwerve.CANCODER_INVERSION);
        angleEncoder.configSensorInitializationStrategy(
            SensorInitializationStrategy.BootToAbsolutePosition
        );
    }

    public SwerveModulePosition getPose() {
        double distance = Conversion.falconToM(
            driveMotor.getSelectedSensorPosition(), kSwerve.WHEEL_CIRCUMFERENCE,
            kSwerve.DRIVE_GEAR_RATIO
        );
        Rotation2d angle = Rotation2d.fromDegrees(
            Conversion.falconToDegrees(
                angleMotor.getSelectedSensorPosition(), kSwerve.ANGLE_GEAR_RATIO
            )
        );
        return new SwerveModulePosition(distance, angle);
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public double getAngle() {
        return -getCanCoder().getDegrees() + angleOffset;
    }

    /**
     * Gets current state of the swerve module.
     */
    public SwerveModulePosition getState() {
        double velocity = Conversion.falconToM(
            driveMotor.getSelectedSensorPosition(), kSwerve.WHEEL_CIRCUMFERENCE,
            kSwerve.DRIVE_GEAR_RATIO
        );

        Rotation2d angle = Rotation2d.fromDegrees(
            Conversion.falconToDegrees(
                angleMotor.getSelectedSensorPosition(), kSwerve.ANGLE_GEAR_RATIO
            )
        );

        return new SwerveModulePosition(velocity, angle);
    }

}
