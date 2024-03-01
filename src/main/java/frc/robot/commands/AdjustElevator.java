package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Elevator;

public class AdjustElevator extends Command {
    public static final double ADJUSTMENT_STEP = Units.inchesToMeters(1.0);

    private final Elevator m_elevator;
    private final DoubleSupplier m_joystickSupplier;

    private SlewRateLimiter m_limiter = new SlewRateLimiter(3);

    public AdjustElevator(Elevator elevator, DoubleSupplier joystickSupplier) {
        m_elevator = elevator;
        m_joystickSupplier = joystickSupplier;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        double newX = m_limiter.calculate(m_joystickSupplier.getAsDouble());

        double adjustment = ADJUSTMENT_STEP * newX;
        m_elevator.setLength(m_elevator.getLength() + adjustment);
    }
}
