package frc.robot.subsytems;

import SushiFrcLib.Sensors.gyro.Pigeon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kPorts;
import frc.robot.Constants.kSwerve;
import frc.robot.util.SwerveModule;
import frc.robot.util.SwerveModuleState;
import frc.robot.util.Vector;

public class Swerve extends SubsystemBase {
    private final SwerveModule[] swerveMods;
    private final Pigeon gyro;

    private static Swerve instance;

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }

        return instance;
    }

    private Swerve() {
        gyro = new Pigeon(kPorts.PIGEON_ID, kSwerve.GYRO_INVERSION, kPorts.CANIVORE_NAME);
        gyro.zeroGyro();

        swerveMods = new SwerveModule[]{
            new SwerveModule(0, kSwerve.Mod0.CONSTANTS),
            new SwerveModule(1, kSwerve.Mod1.CONSTANTS),
            new SwerveModule(2, kSwerve.Mod2.CONSTANTS),
            new SwerveModule(3, kSwerve.Mod3.CONSTANTS),
        };
    }

    // Vector is in mps, and rot is in radians per sec
    public void drive(Vector vector, double rot) {
        vector.rotate(gyro.getAngle());

        SwerveModuleState[] states = kSwerve.SWERVE_KINEMATICS.getStates(vector, rot);

        for (SwerveModuleState i : states) {
            if (i.velocity > kSwerve.MAX_SPEED) {
                i.velocity = kSwerve.MAX_SPEED;
            }
        }

        for (SwerveModule i : swerveMods) {
            i.setDesiredState(states[i.moduleNumber]);
        }
    }

    @Override
    public void periodic() { }
}
