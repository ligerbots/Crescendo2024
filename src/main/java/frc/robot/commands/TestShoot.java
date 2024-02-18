// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;

// For testing. Needs to get separate left and right RPMs
public class TestShoot extends SequentialCommandGroup {
    private final DoubleSupplier m_leftRpm;
    private final DoubleSupplier m_rightRpm;

    /** Creates a new TestShoot. */
    public TestShoot(Shooter shooter, DoubleSupplier leftRpm, DoubleSupplier rightRpm) {
        m_leftRpm = leftRpm;
        m_rightRpm = rightRpm;

        addCommands(
                new SetShooterRpmsAndWait(shooter, this::getShootValues),
                new InstantCommand(shooter::turnOnFeeder),
                new WaitCommand(2),
                new InstantCommand(shooter::turnOffShooter));

        addRequirements(shooter);
    }

    private Shooter.ShooterValues getShootValues() {
        return new Shooter.ShooterValues(m_leftRpm.getAsDouble(), m_rightRpm.getAsDouble(), 0);
    }
}
