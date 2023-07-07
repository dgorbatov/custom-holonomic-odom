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

    // Math taken from https://file.tavsys.net/control/controls-engineering-in-frc.pdf page 211
    private SwerveModuleState getModuleState(Vector translation, double rot, Vector r) {
        double x = translation.x - (rot * r.x); // Module velocity sub x = robot velocity sub x - (angular velo) * (translation vectory y)
        double y = translation.y + (rot * r.y); // Module velocity sub y = robot velocity sub y + (angular velo) * (translation vectory x)

        return new SwerveModuleState(Math.sqrt(x * x + y * y), Rotation2d.fromRadians(Math.atan(y/x) + ((Math.atan(y/x)+ (x>0?0:Math.PI))<0?(Math.PI*2):0) + (x>0?0:Math.PI)));
    }
}
