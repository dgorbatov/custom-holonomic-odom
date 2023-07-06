// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TeleopSwerveDrive;
import frc.robot.subsytems.Swerve;

/**
 * This class is where the bulk of the robot (subsytems, commands, etc.) should be declared. 
 */
public class RobotContainer {
    private final OI oi;

    /**
     * Instaite subsystems and commands.
     */
    public RobotContainer() {
        oi = OI.getInstance();

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        Swerve.getInstance().setDefaultCommand(
            new TeleopSwerveDrive(
                Swerve.getInstance(), 
                () -> oi.getDriveTrainTranslationX(),
                () -> oi.getDriveTrainTranslationY(),
                () -> oi.getDriveTrainRotation()
            )
        );
    }

    public Command getAutonomousCommand() {
        return null;    
    }
}
