package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;

public class SwerveOdom {
    private RobotPose currPos;
    private SwerveKinematics kinematics;

    private SwerveModulePosition[] prevSwerveModulePositions;

    public SwerveOdom(SwerveKinematics kinematics) {
        this.kinematics = kinematics;

        currPos = new RobotPose(new Vector(0, 0), new Rotation2d(0));

        prevSwerveModulePositions = new SwerveModulePosition[]{null, null, null, null};

        for (int i=0; i < 4; ++i) {
            prevSwerveModulePositions[i] = new SwerveModulePosition(0, Rotation2d.fromDegrees(0));
        }
    }

    public void setPose(RobotPose newPos) {
        currPos = newPos;
    }

    public void updatePoseWithGyro(SwerveModulePosition[] swerveModulePositions, Rotation2d gyroAngle) {
        Vector robotDelta = kinematics.getSpeedFromDelta(getSwerveModuleDeltas(swerveModulePositions)).translationVector;
        Rotation2d angleDelta = gyroAngle.minus(currPos.rot);

        double sin_theta = Math.sin(angleDelta.getRadians());
        double cos_theta = Math.cos(angleDelta.getRadians());
        double s, c;

        if (Math.abs(angleDelta.getRadians()) < 1E-9) {
            s = 1.0 - 1.0 / 6.0 * angleDelta.getRadians() * angleDelta.getRadians(); // Use taylor series as u r dividing by 0
            c = .5 *angleDelta.getRadians();
        } else {
            s = sin_theta / angleDelta.getRadians();
            c = (1.0 - cos_theta) / angleDelta.getRadians();
        }

        currPos = new RobotPose(new Vector(robotDelta.x * s - robotDelta.y * c, robotDelta.x * c + robotDelta.y * s), gyroAngle);
    }

    private SwerveModulePosition[] getSwerveModuleDeltas(SwerveModulePosition[] swerveModulePositions) {
        SwerveModulePosition[] states = new SwerveModulePosition[]{null, null, null, null};

        for (int i=0; i < 4; ++i) {
            states[i] = new SwerveModulePosition(
                swerveModulePositions[i].distance - prevSwerveModulePositions[i].distance, 
                swerveModulePositions[i].angle
            );
        }

        prevSwerveModulePositions = swerveModulePositions;

        return states;
    }

    public RobotPose getPose() {
        return currPos;
    }
}
