package com.team2502.robot2019.command.autonomous.ingredients;

import com.github.ezauton.core.action.PeriodicAction;
import com.github.ezauton.core.action.TimedPeriodicAction;
import com.github.ezauton.core.utils.Clock;
import com.github.ezauton.core.utils.RealClock;
import com.github.ezauton.core.utils.Stopwatch;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.Robot;
import com.team2502.robot2019.subsystem.interfaces.DriveTrain;

import java.util.concurrent.TimeUnit;

public class VelocityDriveAction extends TimedPeriodicAction
{
    private final double leftVel;
    private final double rightVel;
    private final boolean brake;

    private DriveTrain dt;

    /**
     * @param leftVel
     * @param rightVel
     * @param time       Amount of time to run for (milliseconds)
     */
    public VelocityDriveAction(double leftVel, double rightVel, long time)
    {this(leftVel, rightVel, time, false, Robot.DRIVE_TRAIN); }


    /**
     * @param leftVel
     * @param rightVel
     * @param time       Amount of time to run for (milliseconds)
     */
    public VelocityDriveAction(double leftVel, double rightVel, long time, boolean brake, DriveTrain dt)
    {
        super(Constants.DEFAULT_ACTION_PERIOD, Constants.DEFAULT_ACTION_PERIOD_UNIT, time, TimeUnit.MILLISECONDS);

        this.leftVel = leftVel;
        this.rightVel = rightVel;
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
        stopwatch.init();
    }

    @Override
    public void execute()
    {
        dt.runMotorsVelocity(leftVel, rightVel);
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
