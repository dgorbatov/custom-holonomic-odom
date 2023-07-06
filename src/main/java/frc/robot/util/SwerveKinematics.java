package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveKinematics {
    private Vector[] modVectors;

    // Modules are expected to move in a clockwise order, ex: mod0 -> mod1 -> mod2 -> mod3
    public SwerveKinematics(Vector mod0, Vector mod1, Vector mod2, Vector mod3) {
        modVectors = new Vector[]{mod0, mod1, mod2, mod3};
    }
    
    public SwerveModuleState[] getStates(Vector translation, double rot) {
        return new SwerveModuleState[] {
            getModuleState(translation, rot, modVectors[0]),
            getModuleState(translation, rot, modVectors[1]),
            getModuleState(translation, rot, modVectors[2]),
            getModuleState(translation, rot, modVectors[3]),
        };
    }

    private SwerveModuleState getModuleState(Vector translation, double rot, Vector r) {
        double x = translation.x - (rot * r.y);
        double y = translation.y + (rot * r.x);

        return new SwerveModuleState(Math.sqrt(x * x + y * y), Rotation2d.fromRadians(Math.atan(y/x)));
    }
}
