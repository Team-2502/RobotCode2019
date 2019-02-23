package com.team2502.robot2019.command.autonomous.ingredients;

import com.github.ezauton.core.action.PeriodicAction;
import com.github.ezauton.core.trajectory.geometry.ImmutableVector;
import com.github.ezauton.core.utils.IClock;
import com.github.ezauton.core.utils.RealClock;
import com.github.ezauton.core.utils.Stopwatch;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.Robot;
import com.team2502.robot2019.subsystem.interfaces.IDriveTrain;

import java.util.concurrent.TimeUnit;

public class PointDriveAction extends PeriodicAction implements Runnable
{
    private final ImmutableVector loc;
    private final Stopwatch stopwatch;
    private final double speed;
    private final double time;
    private final TimeUnit timeUnit;
    private IDriveTrain dt;

    /**
     * @param timeout       Amount of timeout to run for (seconds)
     */
    public PointDriveAction(double speed, ImmutableVector loc, double timeout)
    {this(speed, loc, timeout, TimeUnit.SECONDS, true, RealClock.CLOCK, Robot.DRIVE_TRAIN); }


    /**
     * @param leftVel
     * @param rightVel
     * @param time       Amount of time to run for (seconds)
     */
    public PointDriveAction(double speed, ImmutableVector loc, double time, TimeUnit timeUnit, boolean brake, IClock clock, IDriveTrain dt)
    {
        super(Constants.DEFAULT_ACTION_PERIOD, Constants.DEFAULT_ACTION_PERIOD_UNIT);
        this.speed = speed;
        this.time = time;
        this.timeUnit = timeUnit;

        this.loc = loc;

        stopwatch = new Stopwatch(clock);
        this.dt = dt;

        this.onFinish(() -> {
            if(brake) {
                dt.runMotorsVoltage(0, 0);
            }
            dt.giveBack();
        });

        addRunnable(this);

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
        stopwatch.init();
    }


    @Override
    protected boolean isFinished()
    {
        return stopwatch.read(timeUnit) >= time || dt.getLocEstimator().estimateLocation().dist(loc) < 0.5;
    }

    @Override
    public void run()
    {
        dt.driveTowardTransLoc(speed, loc);
    }
}
