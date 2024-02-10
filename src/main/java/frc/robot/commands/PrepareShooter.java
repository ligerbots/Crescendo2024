// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class PrepareShooter extends Command {
  /** Creates a new PrepareShooter. */
  private final Shooter m_shooter;
  private final ShooterPivot m_shooterPivot;
  private final Pose2d m_robotPosition;
  private final SetShooterSpeedAndWait m_setShooterSpeedAndWaitCommand;
  private final DoubleSupplier distance;

  public PrepareShooter(Shooter shooter, ShooterPivot shooterPivot, Pose2d robotPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_shooterPivot = shooterPivot;
    m_robotPosition = robotPosition;
    distance = () -> m_robotPosition.getTranslation().getDistance(FieldConstants.flipPose(FieldConstants.SPEAKER).getTranslation());
    DoubleSupplier leftSpeedSupplier = () -> Shooter.calculateShooterSpeeds(distance.getAsDouble()).leftSpeed;
    DoubleSupplier rightSpeedSupplier = () -> Shooter.calculateShooterSpeeds(distance.getAsDouble()).rightSpeed;
    m_setShooterSpeedAndWaitCommand = new SetShooterSpeedAndWait(m_shooter,leftSpeedSupplier,rightSpeedSupplier);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // MATH:
  //Please do not delete

    /**
     * Calculates the angle in radians at which a shooter must aim in order to hit a target and the angle the robot must face.
     * *While still
     * @return first, the robot angle, second the shooter angle
     */
    static List<Double> calculateShootingWhileStillVarables(double shooterX, double shooterY, double shooterZ, double targetX, double targetY, double targetZ, double shooterVectorMagnitude/*aka speed*/) {
        //When blue is on left, orgin is bottom left corner. Zero faces positive X
        final List<Double> inputVectorWithShooterMagnitude = setMagnitude(Arrays.asList(targetX - shooterX, targetY - shooterY, targetZ - shooterZ),shooterVectorMagnitude);                
        double shootAngle = calculateShootingAngle(shooterVectorMagnitude, getMagnitude(setZValue(inputVectorWithShooterMagnitude, 0)), inputVectorWithShooterMagnitude.get(2));
        double robotAngle = getFlatAngleOfVector(inputVectorWithShooterMagnitude);
        if (Double.isNaN(robotAngle) || Double.isNaN(shootAngle)) {
            System.out.println("Too far, get closer. Robot angle: "+robotAngle+" Shooter angle: "+shootAngle);
            return Arrays.asList(getFlatAngleOfVector(inputVectorWithShooterMagnitude), getVerticalTheta(inputVectorWithShooterMagnitude));
        }

        return Arrays.asList(robotAngle, shootAngle);
    }
    
    //Calc-subFunctions

    /**
     * Calculates the angle in radians at which a shooter must aim in order to hit a target.
     * @Note ALL UNITS METRIC
     * 
     * @param targetVectorWithShooterMagnitude The target vector to hit with the magnitude being equal to the shooter speed.
     * @param robotVelocityVector The velocity of the robot in 2D as a vector.
     * @return first, the robot angle, second the note speed.
     */
    //Only needs shooter vector in terms of magnitude and angle up and down. 
    static List<Double> getRobotAngleAndNoteVelocity(List<Double> targetVectorWithShooterMagnitude, List<Double> robotVelocityVector) {
        //Next two lines get "fowards" and "sideways" velocity seperated. I put them in quotes because they are relitive to the target vector
        List<Double> flatProjectedRobotVelocityVector = projectV1ontoV2(robotVelocityVector,setZValue(targetVectorWithShooterMagnitude, 0));
        List<Double> perpendicularRobotVelocityComponent = V1MinusV2(robotVelocityVector,
                flatProjectedRobotVelocityVector);
        //Calculate shooter vector:
        final double tolerance = Math.toRadians(0.01);
        boolean notDoneWithShootCalc = true;
        double angleOfError = 0; //Starts off at zero because we do not know how much to adjust
        List<Double> shootVector = Arrays.asList(0.0, 0.0, 0.0); //Setup so can be accessed at this level, had to add some crap values
        final int maxIterations = 100; // Set a maximum for iterations to prevent infinite loops
        int iterationCount = 0; // Counter to track the number of iterations
        double shooterAdjustmentOffset = 0;
        double adjustmentIncrment = 1;
        while (notDoneWithShootCalc && iterationCount < maxIterations) {
            final double theata = getVerticalTheta(targetVectorWithShooterMagnitude) + shooterAdjustmentOffset;//Math.toRadians(0.1);
            final List<Double> shootFlatPerpendicular = multiplyVector(perpendicularRobotVelocityComponent, -1);
            final List<Double> shootFlatParalell = setMagnitude(flatProjectedRobotVelocityVector, Math.sqrt(Math.pow(Math.cos(theata)*getMagnitude(targetVectorWithShooterMagnitude), 2) - getMagnitude(shootFlatPerpendicular)));

            shootVector = setZValue(addVectors(shootFlatParalell, shootFlatPerpendicular),Math.sin(theata)*getMagnitude(targetVectorWithShooterMagnitude));
            angleOfError = getVerticalTheta(targetVectorWithShooterMagnitude) - getVerticalTheta(addVectors(shootVector, robotVelocityVector));
            System.out.println("adjusted angle of error: "+Math.toDegrees(angleOfError)+" adjustmentIncrement: "+adjustmentIncrment);
            if (Math.abs(angleOfError) < tolerance) {
                notDoneWithShootCalc = false;
                System.out.println("Angle of error when ending = " + Math.toDegrees(angleOfError) + " only took "
                        + iterationCount);
            }
            if (Math.signum(angleOfError) != Math.signum(adjustmentIncrment)) {
                adjustmentIncrment = adjustmentIncrment / 2;
            }
            shooterAdjustmentOffset += adjustmentIncrment * Math.signum(angleOfError);
            iterationCount++;
        }

        double noteVelocity = getMagnitude(shootVector);
        double angle = getFlatAngleOfVector(shootVector);

        return Arrays.asList(angle, noteVelocity);
    }

    /**
     * Calculates the angle in radians at which a shooter must aim in order to hit a target.
     *
     * @param shooterSpeed The speed of the projectile when it is shot (in meters per second).
     * @param distance The horizontal distance to the target (in meters).
     * @param height The vertical distance to the target (in meters), can be negative if target is lower.
     * @return The angle in radians at which to aim to hit the target.
     */
    public static double calculateShootingAngle(double shooterSpeed, double distance, double height) {
        final double GRAVITY = 9.81; // Acceleration due to gravity (m/s^2)
        double speedSquared = shooterSpeed * shooterSpeed;
        double speedFourth = speedSquared * speedSquared;
        // The term under the square root in the quadratic formula
        double discriminant = speedFourth - GRAVITY * (GRAVITY * distance * distance + 2 * height * speedSquared);
        // No real solution exists if the discriminant is negative, because you can't take the square root of a negative number
        if (discriminant < 0) {
            //Returns NaN if too close
            return Double.NaN;
        }
        double rootDiscriminant = Math.sqrt(discriminant);
        // First possible solution for the angle
        double angle1 = Math.atan2(speedSquared - rootDiscriminant, GRAVITY * distance);
        // Second possible solution for the angle
        double angle2 = Math.atan2(speedSquared + rootDiscriminant, GRAVITY * distance);
        // Return the non-negative angle that is less than 90 degrees (in radians)
        return Math.max(0, Math.min(angle1, angle2));
    }

    //General helper functions:
    private static List<Double> setZValue(List<Double> vector, double newZValue) {
        return Arrays.asList(vector.get(0), vector.get(1), newZValue);
    }

    private static List<Double> setMagnitude(List<Double> vector, double magnitude) {
        return multiplyVector(normaliseVector(vector), magnitude);
    }
    
    private static double getVerticalTheta(List<Double> vector) {
        double adjacent = getMagnitude(setZValue(vector, 0));
        return Math.atan(vector.get(2)/adjacent);
    }

    private static double getFlatAngleOfVector(List<Double> vector) {
        double angle = getAngleFromCordinates(vector.get(0),vector.get(1)); // Calculate the angle
        angle = angle < 0 ? angle + 2 * Math.PI : angle; // Adjust if negative
        return angle; // Return the adjusted angle
    }

    private static double getAngleFromCordinates (double x, double y) {
        //Zero/360 are facing positive X
        double angle = Math.atan2(y, x);
        return Math.signum(angle) == -1.0 ? angle + Math.PI*2 : angle;
    }

    public static List<Double> adjustVectorVerticalAngle(List<Double> vector, double newAngleRad) {
        double magnitude = getMagnitude(vector);
        double horizontalAngle = getVerticalTheta(vector);

        // Calculate new vector components based on the new vertical angle and original magnitude
        double newX = magnitude * Math.cos(newAngleRad) * Math.cos(horizontalAngle);
        double newY = magnitude * Math.cos(newAngleRad) * Math.sin(horizontalAngle);
        double newZ = magnitude * Math.sin(newAngleRad);

        return Arrays.asList(newX, newY, newZ);
    }


    public static double metersToFeet(double meters) {
    return meters * 3.28084;
    }

    public static double feetToMeters(double meters) {
    return meters/3.28084;
    }

    // Reapplyable helper functions:
    private static List<Double> projectV1ontoV2(List<Double> vector1, List<Double> vector2) {
        final List<Double> vector2Normalised = normaliseVector(vector2);
        final double projectionMagnitude = dotProduct(vector1, vector2Normalised);
        return multiplyVector(vector2Normalised, projectionMagnitude);
    }

    private static double dotProduct(List<Double> vector1, List<Double> vector2) {
        return vector1.get(0) * vector2.get(0) + vector1.get(1) * vector2.get(1) + vector1.get(2) * vector2.get(2);
    }

    private static List<Double> V1MinusV2(List<Double> vector1, List<Double> vector2) {
        return Arrays.asList(vector1.get(0) - vector2.get(0), vector1.get(1) - vector2.get(1),
                vector1.get(2) - vector2.get(2));
    }

    private static List<Double> addVectors(List<Double> vector1, List<Double> vector2) {
        return Arrays.asList(vector1.get(0) + vector2.get(0), vector1.get(1) + vector2.get(1),
                vector1.get(2) + vector2.get(2));
    }

    private static List<Double> multiplyVector(List<Double> vector, double multiplyer) {
        return vector.stream().map((i) -> {
            return i * multiplyer;
        }).toList();
    }

    private static List<Double> normaliseVector(List<Double> vector) {
        double magnitude = getMagnitude(vector);
        return vector.stream().map((i) -> {
            return i / magnitude;
        }).toList();
    }

    private static double getMagnitude(List<Double> vector) {
        final double c = Math
                .sqrt(Math.pow(vector.get(0), 2) + Math.pow(vector.get(1), 2) + Math.pow(vector.get(2), 2));
        return c;
    }
}
