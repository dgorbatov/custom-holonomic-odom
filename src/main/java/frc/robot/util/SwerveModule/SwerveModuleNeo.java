package frc.robot.util.SwerveModule;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import SushiFrcLib.Swerve.SwerveModuleConstants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.kSwerve;
import frc.robot.util.SwerveModulePosition;
import frc.robot.util.SwerveModuleState;

public class SwerveModuleNeo extends SwerveModule {
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    private final SparkMaxPIDController drivePID;

    private final CANSparkMax angleMotor;
    private final RelativeEncoder angleEncoder;
    private final SparkMaxPIDController anglePID;

    private final CANCoder canCoder;

    private double lastAngle;

    public SwerveModuleNeo(int moduleNumber, SwerveModuleConstants constants) {
        super(moduleNumber, constants);

        driveMotor = new CANSparkMax(constants.driveMotorId, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        drivePID = driveMotor.getPIDController();

        angleMotor = new CANSparkMax(constants.angleMotorId, MotorType.kBrushless);
        angleEncoder = angleMotor.getEncoder();
        anglePID = angleMotor.getPIDController();

        canCoder = new CANCoder(constants.cancoderId);

        configureDevices();
        lastAngle = getState().angle.getRadians();
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {
        // Prevents angle motor from turning further than it needs to. 
        // E.G. rotating from 10 to 270 degrees CW vs CCW.
        // state = SwerveModuleState.optimize(state, getState().angle);

        double targetAngle = state.angle.getRadians();

        // double targetSpeed = state.velocity;

        // double delta = targetAngle - lastAngle;

        // if (Math.abs(delta) > 90) {
        //     targetSpeed = -targetSpeed;
        //     targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);        
        // }

        drivePID.setReference(state.velocity, CANSparkMax.ControlType.kVelocity);

        double angle = Math.abs(state.velocity) <= Constants.kSwerve.MAX_SPEED * 0.01
            ? lastAngle
            :  targetAngle;

        anglePID.setReference(angle, CANSparkMax.ControlType.kPosition);
        lastAngle = angle;

        SmartDashboard.putNumber("Current Mod Encoder Angle: " + moduleNumber, Rotation2d.fromRadians(angleEncoder.getPosition()).getDegrees());
    }

    public SwerveModuleState getState() {
        double velocity = driveEncoder.getVelocity();
        Rotation2d rot = Rotation2d.fromRadians(angleEncoder.getPosition());
        return new SwerveModuleState(velocity, rot);
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(canCoder.getAbsolutePosition());
    }

    public double getAngle() {
        return -getCanCoder().getDegrees() + angleOffset;
    }

    @Override
    public SwerveModulePosition getPose() {
        double distance = driveEncoder.getPosition();
        Rotation2d rot = new Rotation2d(angleEncoder.getPosition());
        return new SwerveModulePosition(distance, rot);
    }

    private void configureDevices() {
        // Drive motor configuration.
        driveMotor.restoreFactoryDefaults();
        driveMotor.setInverted(Constants.kSwerve.DRIVE_INVERSION);
        driveMotor.setIdleMode(Constants.kSwerve.DRIVE_IDEL_MODE);
        driveMotor.setSmartCurrentLimit(Constants.kSwerve.DRIVE_CURRENT_LIMIT);

        drivePID.setP(Constants.kSwerve.DRIVE_P);
        drivePID.setI(Constants.kSwerve.DRIVE_I);
        drivePID.setD(Constants.kSwerve.DRIVE_D);
        drivePID.setFF(Constants.kSwerve.DRIVE_F);

        driveEncoder.setPositionConversionFactor(Constants.kSwerve.DRIVE_ROTATIONS_TO_METERS);
        driveEncoder.setVelocityConversionFactor(Constants.kSwerve.DRIVE_RPM_TO_METERS_PER_SECOND);
        driveEncoder.setPosition(0);

        // Angle motor configuration.
        angleMotor.restoreFactoryDefaults();
        angleMotor.setInverted(Constants.kSwerve.ANGLE_INVERSION);
        angleMotor.setIdleMode(Constants.kSwerve.ANGLE_IDLE_MODE);
        angleMotor.setSmartCurrentLimit(Constants.kSwerve.ANGLE_CURRENT_LIMIT);

        anglePID.setP(Constants.kSwerve.ANGLE_P);
        anglePID.setI(Constants.kSwerve.ANGLE_I);
        anglePID.setD(Constants.kSwerve.ANGLE_D);
        anglePID.setFF(Constants.kSwerve.ANGLE_F);

        anglePID.setPositionPIDWrappingEnabled(true);
        anglePID.setPositionPIDWrappingMaxInput(2 * Math.PI);
        anglePID.setPositionPIDWrappingMinInput(0);

        angleEncoder.setPositionConversionFactor(Constants.kSwerve.ANGLE_ROTATIONS_TO_RADIANS);
        angleEncoder.setVelocityConversionFactor(Constants.kSwerve.ANGLE_RPM_TO_RADIANS_PER_SECOND);
        resetToAbsolute();

        // CanCoder configuration.
        canCoder.configFactoryDefault();
        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        canCoder.configSensorDirection(kSwerve.CANCODER_INVERSION);
        canCoder.configSensorInitializationStrategy(
            SensorInitializationStrategy.BootToAbsolutePosition
        );
    }

    @Override
    public void resetToAbsolute() {
        angleEncoder.setPosition(Units.degreesToRadians(getAngle())); 
    }
}
