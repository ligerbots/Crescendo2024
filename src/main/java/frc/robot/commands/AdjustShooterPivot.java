package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPivot;

public class AdjustShooterPivot extends Command {
    public static final double ADJUSTMENT_STEP = Math.toRadians(2.0);
    
    private final ShooterPivot m_shooterPivot;
    private final DoubleSupplier m_joystickSupplier;

    private SlewRateLimiter m_limiter = new SlewRateLimiter(3);

    public AdjustShooterPivot(ShooterPivot shooterPivot, DoubleSupplier joystickSupplier) {
        m_shooterPivot = shooterPivot;
        m_joystickSupplier = joystickSupplier;

        addRequirements(shooterPivot);
    }

    @Override
    public void execute() {
        double newX = m_limiter.calculate(m_joystickSupplier.getAsDouble());

        double adjustment = ADJUSTMENT_STEP * newX;
        m_shooterPivot.setAngle(m_shooterPivot.getAngleRadians() + adjustment);
    }
}
