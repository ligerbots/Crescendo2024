package frc.robot.swerve;

public interface DriveController {
    // set the drive voltage
    public void setReferenceVoltage(double voltage);
    // get the drive velocity
    public double getStateVelocity();
    // get wheel distance
    public double getWheelDistance();
        
    // update the smart dashboard
    public void updateSmartDashboard(String sdPrefix);
}
