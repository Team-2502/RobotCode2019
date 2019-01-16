package com.team2502.robot2019.command.autonomous.ingredients;

import com.github.ezauton.core.action.PeriodicAction;
import com.github.ezauton.core.utils.IClock;
import com.github.ezauton.core.utils.RealClock;
import com.github.ezauton.core.utils.Stopwatch;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.Robot;
import com.team2502.robot2019.subsystem.interfaces.IDriveTrain;

import java.util.concurrent.TimeUnit;

public class VelocityDriveAction extends PeriodicAction implements Runnable
{
    private final double leftVel;
    private final double rightVel;
    private final boolean brake;

    private final Stopwatch stopwatch;
    private final double time;
    private final TimeUnit timeUnit;
    private IDriveTrain dt;

    /**
     * @param leftVel
     * @param rightVel
     * @param time       Amount of time to run for (seconds)
     */
    public VelocityDriveAction(double leftVel, double rightVel, double time)
    {this(leftVel, rightVel, time, TimeUnit.SECONDS, true, RealClock.CLOCK, Robot.DRIVE_TRAIN); }


    /**
     * @param leftVel
     * @param rightVel
     * @param time       Amount of time to run for (seconds)
     */
    public VelocityDriveAction(double leftVel, double rightVel, double time, TimeUnit timeUnit, boolean brake, IClock clock, IDriveTrain dt)
    {
        super(Constants.DEFAULT_ACTION_PERIOD, Constants.DEFAULT_ACTION_PERIOD_UNIT);
        this.time = time;
        this.timeUnit = timeUnit;

        this.leftVel = leftVel;
        this.rightVel = rightVel;
        this.brake = brake;

        stopwatch = new Stopwatch(clock);
        this.dt = dt;

        this.onFinish(() -> {
            if(brake) {
                dt.runMotorsVoltage(0, 0);
            }
            dt.give();
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
        return stopwatch.read(timeUnit) >= time;
    }

    @Override
    public void run()
    {
        dt.runMotorsVelocity(leftVel, rightVel);
    }
}
