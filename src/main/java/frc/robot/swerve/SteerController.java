package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public interface SteerController {
    // get the reference angle
    public double getReferenceAngle();
    // set the angle we want for the wheel (radians)
    public void setReferenceAngle(double referenceAngleRadians);

    // synchronize the angle encoder offsets
    public void syncAngleEncoders(boolean dontCheckTimer);
    // get the current module angle in radians
    public Rotation2d getStateAngle();
    
    // update the smart dashboard
    public void updateSmartDashboard(String sdPrefix);
}
