package com.team2502.robot2019.command.autonomous.ingredients;

import com.github.ezauton.core.action.PeriodicAction;
import com.github.ezauton.core.action.TimedPeriodicAction;
import com.github.ezauton.core.utils.MathUtils;
import com.team2502.robot2019.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.concurrent.TimeUnit;

/**
 * Drive straight by using basic PD control on the heading
 */
public class TurnToAnglePDAction extends PeriodicAction
{


    private final double speed;

    private final double defaultKPgain = 10;
    private final double defaultKDgain = 0;

    private double targetAngle;

    private double kPgain;
    private double kDgain;
    private static final double velErrThresh = 0.05;
    private static final double posErrThresh = 0.05;

    /**
     * Construct a Drive Straight command
     *
     * @param speed How fast to go (ft/s)
     * */
    public TurnToAnglePDAction(double speed, double targetAngle) {
        super(20, TimeUnit.MILLISECONDS);
        this.speed = speed;

        SmartDashboard.putNumber("turntoangle_kP", defaultKPgain);
        SmartDashboard.putNumber("turntoangle_kD", defaultKDgain);
        this.targetAngle = targetAngle;
    }

    @Override
    protected void init() throws InterruptedException
    {
        Robot.DRIVE_TRAIN.take();
                // Allow for tuning of PID without redeployment.
        kPgain = SmartDashboard.getNumber("turntoangle_kP", defaultKPgain);
        kDgain = SmartDashboard.getNumber("turntoangle_kD", defaultKDgain);
    }

    @Override
    protected void execute()
    {
        double currentAngle = Robot.DRIVE_TRAIN.getRotEstimator().estimateHeading();

        // Angluar velocity is the change in error and also the change in absolute angle because taking the derivative eliminates constants
        // and the initial angle is a constant
        // Learn calculus for more information
        double currentAngularRate = MathUtils.deg2Rad(Robot.DRIVE_TRAIN.getAngularVelocity());

        double desiredWheelDifferential = ((targetAngle - currentAngle) * kPgain - (currentAngularRate) * kDgain);

        SmartDashboard.putNumber("drivestraight_desiredWheelDifferential", desiredWheelDifferential);

        Robot.DRIVE_TRAIN.runMotorsVelocity( - desiredWheelDifferential,  desiredWheelDifferential);
    }

    @Override
    protected boolean isFinished() throws Exception
    {
        return Math.abs(Robot.DRIVE_TRAIN.getRotEstimator().estimateHeading() - targetAngle) < posErrThresh &&
                Math.abs(MathUtils.deg2Rad(Robot.DRIVE_TRAIN.getAngularVelocity())) < velErrThresh;
    }

    @Override
    public void end() throws Exception
    {
        Robot.DRIVE_TRAIN.giveBack();
    }
}
