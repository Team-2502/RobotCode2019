package com.team2502.robot2019.command.autonomous.ingredients;

import com.github.ezauton.core.action.TimedPeriodicAction;
import com.github.ezauton.core.utils.MathUtils;
import com.team2502.robot2019.Robot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.concurrent.TimeUnit;

import static com.team2502.robot2019.command.autonomous.ingredients.DriveStraightWithGyroCommand.defaultKDgain;
import static com.team2502.robot2019.command.autonomous.ingredients.DriveStraightWithGyroCommand.defaultKPgain;

/**
 * Drive straight by using basic PD control on the heading
 */
public class DriveStraightWithGyroAction extends TimedPeriodicAction
{


    private final double speed;

    private double targetAngle;

    private double kPgain;
    private double kDgain;

    /**
     * Construct a Drive Straight command
     *
     * @param speed How fast to go (ft/s)
     * @param durationUnit How long to go for (seconds)
     */
    public DriveStraightWithGyroAction(double speed, double duration, TimeUnit durationUnit) {
        super((long) duration, durationUnit);
        this.speed = speed;

        SmartDashboard.putNumber("drivestraight_kP", defaultKPgain);
        SmartDashboard.putNumber("drivestraight_kD", defaultKDgain);
    }

    @Override
    protected void init() throws InterruptedException
    {
        Robot.DRIVE_TRAIN.take();
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
        double currentAngularRate = MathUtils.deg2Rad(Robot.DRIVE_TRAIN.getAngularVelocity());

        double desiredWheelDifferential = (targetAngle - currentAngle) * kPgain - (currentAngularRate) * kDgain;
        SmartDashboard.putNumber("drivestraight_desiredWheelDifferential", desiredWheelDifferential);

        Robot.DRIVE_TRAIN.runMotorsVelocity(speed - desiredWheelDifferential, speed + desiredWheelDifferential);
    }

    @Override
    public void end() throws Exception
    {
        Robot.DRIVE_TRAIN.giveBack();
    }
}
