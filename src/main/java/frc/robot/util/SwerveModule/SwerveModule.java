package frc.robot.util.SwerveModule;

import SushiFrcLib.Swerve.SwerveModuleConstants;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class SwerveModule {
    public int moduleNumber;
    protected double angleOffset;

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;
    }

    abstract public void resetToAbsolute();

    abstract public double getAngle();

    abstract public SwerveModulePosition getPose();

    abstract public void setDesiredState(SwerveModuleState desiredState);
}
