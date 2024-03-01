package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator;

import java.util.function.DoubleSupplier;

public class AdjustElevator extends InstantCommand {
    private final Elevator m_elevator;
    private final DoubleSupplier m_YSupplier;

    public AdjustElevator(Elevator elevator,
            DoubleSupplier YSupplier) {
        this.m_elevator = elevator;
        this.m_YSupplier = YSupplier;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        m_elevator.overrideLength(Math.copySign(Elevator.OVERRIDE_METERS, m_YSupplier.getAsDouble()));
    }
}
