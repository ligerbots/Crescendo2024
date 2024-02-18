/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveTrain;

public class ActiveTurnToHeadingWithDriving extends Command {

    private final DriveTrain m_driveTrain;
    private final Supplier<Rotation2d> m_wantedHeadingSupplier;
    private final DoubleSupplier m_joystickXSupplier;
    private final DoubleSupplier m_joystickYSupplier;

    private final static double KP = 0.2; // TODO pick correct values
    private final static double KI = 0.0;
    private final static double KD = 0.0;

    private final PIDController m_turnHeadingPID;
    private Double m_wantedDegrees;

    /**
     * Creates a new autoAim.
     */
    public ActiveTurnToHeadingWithDriving(DriveTrain driveTrain, Supplier<Rotation2d> wantedHeading,
            DoubleSupplier joystickXSupplier, DoubleSupplier joystickYSupplier) {
        m_driveTrain = driveTrain;
        m_wantedHeadingSupplier = wantedHeading;
        m_joystickXSupplier = joystickXSupplier;
        m_joystickYSupplier = joystickYSupplier;

        m_turnHeadingPID = new PIDController(KP, KI, KD);
        addRequirements(m_driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_turnHeadingPID.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // auto aiming using PID
        m_wantedDegrees = m_wantedHeadingSupplier.get().getDegrees();
        double speed = m_turnHeadingPID.calculate(m_driveTrain.getHeading().getDegrees(), m_wantedDegrees);

        m_driveTrain.joystickDrive(m_joystickXSupplier.getAsDouble(), m_joystickYSupplier.getAsDouble(), -speed);

        // Record whether at the right heading, so that other commands can check
        m_driveTrain.setOnGoalForActiveTurn(
            Math.abs(m_driveTrain.getHeading().getDegrees() - m_wantedDegrees) < DriveTrain.ANGLE_TOLERANCE_DEGREES);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}