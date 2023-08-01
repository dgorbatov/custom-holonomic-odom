package frc.robot.util.SwerveModule;

import SushiFrcLib.Swerve.SwerveModuleConstants;
import frc.robot.util.SwerveModulePosition;
import frc.robot.util.SwerveModuleState;

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
