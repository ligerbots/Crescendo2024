package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterPivot;

import java.util.function.DoubleSupplier;

public class AdjustShooterPivot extends InstantCommand {
    private final ShooterPivot m_ShooterPivot;
    private final DoubleSupplier m_YSupplier;

    public AdjustShooterPivot(ShooterPivot shooterPivot,
            DoubleSupplier YSupplier) {
        this.m_ShooterPivot = shooterPivot;
        this.m_YSupplier = YSupplier;

        addRequirements(shooterPivot);
    }

    @Override
    public void execute() {
        m_ShooterPivot.overrideShooterPivot(Math.copySign(ShooterPivot.OVERRIDE_RADIANS, m_YSupplier.getAsDouble()));
    }
}
