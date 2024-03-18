// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class Rumble extends Command {
    static final double RUMBLING_WAIT_TIME = 0.3;
    private final XboxController m_xbox;
    private final Timer m_timer = new Timer();

    /** Creates a new Rumble. */
    public Rumble(XboxController xbox) {
        m_xbox = xbox;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_timer.restart();

        // start rumbling
        m_xbox.setRumble(RumbleType.kBothRumble, 1);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // stop rumbling
        m_xbox.setRumble(RumbleType.kBothRumble, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(RUMBLING_WAIT_TIME);
    }
}
