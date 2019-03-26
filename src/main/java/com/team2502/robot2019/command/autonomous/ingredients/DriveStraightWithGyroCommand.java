package com.team2502.robot2019.command.autonomous.ingredients;

import com.team2502.robot2019.Robot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Drive straight by using basic PD control on the heading
 */
public class DriveStraightWithGyroCommand extends Command
{
    private static final double defaultKPgain = 0.2;
    private static final double defaultKDgain = 0;


    private final double speed;

    private double targetAngle;

    private double kPgain;
    private double kDgain;

    /**
     * Construct a Drive Straight command
     *
     * @param speed How fast to go (ft/s)
     * @param timeout How long to go for (seconds)
     */
    public DriveStraightWithGyroCommand(double speed, double timeout) {
        super(timeout);
        this.speed = speed;

        SmartDashboard.putNumber("drivestraight_kP", defaultKPgain);
        SmartDashboard.putNumber("drivestraight_kD", defaultKDgain);
    }

    @Override
    protected void initialize()
    {
        this.targetAngle = Robot.DRIVE_TRAIN.getRotEstimator().estimateHeading();

        // Allow for tuning of PID without redeployment.
        kPgain = SmartDashboard.getNumber("drivestraight_kP", defaultKPgain);
        kDgain = SmartDashboard.getNumber("drivestraight_kD", defaultKDgain);
    }

    @Override
    protected void execute()
    {
        double currentAngle = Robot.DRIVE_TRAIN.getRotEstimator().estimateHeading();

        // Angluar velocity is the change in error and also the change in absolute angle because taking the derivative eliminates constants
        // and the initial angle is a constant
        // Learn calculus for more information
        double currentAngularRate = Robot.DRIVE_TRAIN.getAngularVelocity();

        double desiredWheelDifferential = (targetAngle - currentAngle) * kPgain - (currentAngularRate) * kDgain;
        SmartDashboard.putNumber("drivestraight_desiredWheelDifferential", desiredWheelDifferential);

        Robot.DRIVE_TRAIN.runMotorsVelocity(speed - desiredWheelDifferential, speed + desiredWheelDifferential);
    }

    @Override
    protected boolean isFinished()
    {
        return isTimedOut();
    }
}
