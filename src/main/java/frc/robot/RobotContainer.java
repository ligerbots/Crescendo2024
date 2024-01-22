// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.commands.Drive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    private final CommandXboxController m_controller = new CommandXboxController(0);
    private final Joystick m_farm = new Joystick(1);

    // private final Vision m_vision = new Vision();
    private final DriveTrain m_driveTrain = new DriveTrain();

    public RobotContainer() {
        configureBindings();
        m_driveTrain.setDefaultCommand(getDriveCommand());
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public Command getDriveCommand() {
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        return new Drive(
                m_driveTrain,
                () -> -modifyAxis(m_controller.getLeftY()),
                () -> -modifyAxis(m_controller.getLeftX()),
                () -> -modifyAxis(m_controller.getRightX()));
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }

    public DriveTrain getDriveTrain() {
        return m_driveTrain;
    }
}
