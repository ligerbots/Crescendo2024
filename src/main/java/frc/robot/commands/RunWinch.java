package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

import java.util.function.DoubleSupplier;

public class RunWinch extends Command {
    private final Climber m_climber;

    private final DoubleSupplier m_leftSpeedSupplier;
    private final DoubleSupplier m_rightSpeedSupplier;


    public RunWinch(Climber climber,
            DoubleSupplier leftSpeedSupplier,DoubleSupplier rightSpeedSupplier) {
        m_climber = climber;
        m_leftSpeedSupplier = leftSpeedSupplier;
        m_rightSpeedSupplier = rightSpeedSupplier;

        addRequirements(m_climber);
    }

    @Override
    public void execute() {
        m_climber.run(m_leftSpeedSupplier.getAsDouble(), m_rightSpeedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.run(0,0);
    }
}
