package com.team2502.robot2019.command.autonomous.ingredients;

import com.github.ezauton.core.action.TimedPeriodicAction;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.Robot;
import com.team2502.robot2019.subsystem.interfaces.DriveTrain;

import java.util.concurrent.TimeUnit;

public class AccelVelocityDriveAction extends TimedPeriodicAction
{
    private final double leftVel;
    private final double rightVel;

    private final double maxAccelFPS;

    private final boolean brake;

    private DriveTrain dt;

    /**
     * @param leftVel Velocity of left wheel (ft/s)
     * @param rightVel Velocity of right wheel (ft/s)
     * @param maxAccelFPS Maximum acceleration of the robot (ft/s^2)
     * @param time       Amount of time to run for (milliseconds)
     */
    public AccelVelocityDriveAction(double leftVel, double rightVel, double maxAccelFPS, long time)
    {this(leftVel, rightVel, maxAccelFPS, time, false, Robot.DRIVE_TRAIN); }


    /**
     * @param leftVel
     * @param rightVel
     * @param time       Amount of time to run for (milliseconds)
     */
    public AccelVelocityDriveAction(double leftVel, double rightVel, double maxAccelFPS, long time, boolean brake, DriveTrain dt)
    {
        super(Constants.DEFAULT_ACTION_PERIOD, Constants.DEFAULT_ACTION_PERIOD_UNIT, time, TimeUnit.MILLISECONDS);

        this.leftVel = leftVel;
        this.rightVel = rightVel;
        this.maxAccelFPS = maxAccelFPS;
        this.brake = brake;

        this.dt = dt;


    }

    @Override
    protected void init()
    {
        try
        {
            dt.take();
        }
        catch(InterruptedException e)
        {
            e.printStackTrace();
        }
        Robot.DRIVE_TRAIN.applyAutonomousPID();
        Robot.DRIVE_TRAIN.resetForAccelDrive();
        stopwatch.init();
    }

    @Override
    public void execute()
    {
        Robot.DRIVE_TRAIN.runAccelVelocity(leftVel, rightVel, maxAccelFPS);
    }

    @Override
    public void end() throws Exception
    {
        if(!brake) {
            dt.runMotorsVoltage(0, 0);
        }
        else {
            dt.runMotorsVelocity(0, 0);
        }
        dt.giveBack();
    }
}
