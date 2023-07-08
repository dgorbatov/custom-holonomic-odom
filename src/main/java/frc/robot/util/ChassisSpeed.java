package frc.robot.util;

public class ChassisSpeed {
    public Vector translationVector; // meters per second
    public double rot; // radians per second

    public ChassisSpeed(Vector translationVector, double rot) {
        this.translationVector = translationVector;
        this.rot = rot;
    }
}
