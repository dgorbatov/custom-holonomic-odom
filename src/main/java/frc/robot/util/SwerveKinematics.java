package frc.robot.util;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveKinematics {
    private Translation2d[] modTranslation2ds;
    private SimpleMatrix forwardKinematics;


    // Modules are expected to move in a clockwise order, ex: mod0 -> mod1 -> mod2 -> mod3
    public SwerveKinematics(Translation2d mod0, Translation2d mod1, Translation2d mod2, Translation2d mod3) {
        modTranslation2ds = new Translation2d[]{mod0, mod1, mod2, mod3};

        forwardKinematics = new SimpleMatrix(8, 3);

        for (int i = 0; i < 4; ++i) {
            forwardKinematics.setRow(i * 2 + 0, 0, /* Start Data */ 1, 0, -modTranslation2ds[i].getY());
            forwardKinematics.setRow(i * 2 + 1, 0, /* Start Data */ 0, 1, +modTranslation2ds[i].getX());
        }

        forwardKinematics = forwardKinematics.pseudoInverse();
    }
    
    public SwerveModuleState[] getStates(Translation2d translation, double rot) {
        return new SwerveModuleState[] {
            getModuleState(translation, rot, modTranslation2ds[0]),
            getModuleState(translation, rot, modTranslation2ds[1]),
            getModuleState(translation, rot, modTranslation2ds[2]),
            getModuleState(translation, rot, modTranslation2ds[3]),
        };
    }

    public Twist2d getTwistFromDeltra(SwerveModulePosition[] modDelta) {
        SimpleMatrix modStatesMatrix = new SimpleMatrix(8, 1);

        for (int i = 0; i < 4; ++i) {
            modStatesMatrix.set(i * 2, 0, modDelta[i].distanceMeters * modDelta[i].angle.getCos());
            modStatesMatrix.set(i * 2 + 1,0, modDelta[i].distanceMeters * modDelta[i].angle.getSin());
        }
      
        SimpleMatrix chassisSpeedMatrix = forwardKinematics.mult(modStatesMatrix);

        return new Twist2d(
            chassisSpeedMatrix.get(0,0), 
            chassisSpeedMatrix.get(1,0),
            chassisSpeedMatrix.get(2,0)
        );
    }

    // Math taken from https://file.tavsys.net/control/controls-engineering-in-frc.pdf page 211
    private SwerveModuleState getModuleState(Translation2d translation, double rot, Translation2d r) {
        double x = translation.getX() - (rot * r.getX()); // Module velocity sub x = robot velocity sub x - (angular velo) * (translation Translation2dy y)
        double y = translation.getY() + (rot * r.getY()); // Module velocity sub y = robot velocity sub y + (angular velo) * (translation Translation2dy x)


        return new SwerveModuleState(
            Math.sqrt(x * x + y * y), 
            Rotation2d.fromRadians(
                Math.atan2(y, x)
            )
        );
    }
}
