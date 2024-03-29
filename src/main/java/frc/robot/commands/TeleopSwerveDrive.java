package frc.robot.commands;

import SushiFrcLib.Math.Normalization;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.kSwerve;
import frc.robot.subsytems.Swerve;
import frc.robot.util.Vector;

import java.util.function.Supplier;

/**
 * Command that controls teleop swerve.
 */
public class TeleopSwerveDrive extends CommandBase {
    private final Swerve swerve;
    private final Supplier<Double> xaxisSupplier; 
    private final Supplier<Double> yaxisSupplier;
    private final Supplier<Double> rotSupplier;

    private Boolean isRedAlliance;
    private NetworkTable table;

    /**
     * Set swerve subsytem, controlers, axis's, and other swerve paramaters.
     */
    public TeleopSwerveDrive(Swerve swerve,
        Supplier<Double> xaxisSupplier, 
        Supplier<Double> yaxisSupplier, 
        Supplier<Double> rotSupplier
    ) {
        this.swerve = swerve;

        this.xaxisSupplier = xaxisSupplier;
        this.yaxisSupplier = yaxisSupplier;
        this.rotSupplier = rotSupplier;

        table = NetworkTableInstance.getDefault().getTable("FMSInfo");
        isRedAlliance = table.getEntry("IsRedAlliance").getBoolean(true);

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        isRedAlliance = table.getEntry("IsRedAlliance").getBoolean(true);
        double forwardBack = yaxisSupplier.get() * (isRedAlliance ? -1 : 1);
        double leftRight = -xaxisSupplier.get() * (isRedAlliance ? -1 : 1);
        double rot = rotSupplier.get();

        forwardBack = Normalization.cube(applyDeadband(forwardBack));

        leftRight = Normalization.cube(applyDeadband(leftRight));

        Translation2d translation = new Translation2d(forwardBack, leftRight)
                .times(kSwerve.MAX_SPEED).times(kSwerve.SPEED_MULTIPLER);

        rot = Normalization.cube(rot);
        rot *= kSwerve.MAX_ANGULAR_VELOCITY * kSwerve.SPEED_MULTIPLER;

        swerve.drive(new Vector(translation.getX(), translation.getY()), rot);
    }

    private double applyDeadband(double initalVal) {
        return Math.abs(initalVal) <  Constants.STICK_DEADBAND ? 0 : (
            (initalVal - ((initalVal < 0 ? -1 : 1) * Constants.STICK_DEADBAND)) 
            / (1 - Constants.STICK_DEADBAND));
    }
}
