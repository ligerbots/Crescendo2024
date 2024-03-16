// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Intake;

public class RumbleOnIntake extends Command {
    static final double RUMBLING_WAIT_TIME = 0.3;
    private final XboxController m_xbox;
    private final Intake m_intake;
    
    private final Timer m_timer = new Timer();
    private boolean m_prevState;
    private boolean m_pastFirst;

    /** Creates a new RumbleOnIntake. */
    public RumbleOnIntake(Intake intake, XboxController xbox) {
        m_intake = intake;
        m_xbox = xbox;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_prevState = false;
        m_pastFirst = false;
        m_timer.reset();
        m_timer.stop();
    }

    @Override
    public void execute() {
        // check status of Note in the Intake
        // If it transitions from True to False, Note has passed through, so Rumble
        boolean currState = m_intake.noteInCentering();

        if (m_prevState && !currState) {
            if (!m_pastFirst) {
                m_pastFirst = true;
            }
            else {
                // note has passed through the Centering Wheels
                // start rumbling
                m_xbox.setRumble(RumbleType.kBothRumble, 1);
                m_timer.restart();
            }
        }

        m_prevState = currState;
    }

    @Override
    public void end(boolean interrupted) {
        // stop rumbling
        m_xbox.setRumble(RumbleType.kBothRumble, 0);
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(RUMBLING_WAIT_TIME);
    }
}
