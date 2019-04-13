package com.team2502.robot2019.command.autonomous.ingredients;

import com.github.ezauton.core.action.TimedPeriodicAction;
import com.github.ezauton.core.utils.Clock;
import com.github.ezauton.core.utils.RealClock;
import com.github.ezauton.core.utils.Stopwatch;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.Robot;
import com.team2502.robot2019.subsystem.interfaces.DriveTrain;

import java.util.concurrent.TimeUnit;

public class VoltageDriveAction extends TimedPeriodicAction
{
    private final double leftVolts;
    private final double rightVolts;
    private final boolean brake;

    private final Stopwatch stopwatch;
    private DriveTrain dt;

    /**
     * @param leftVolts
     * @param rightVolts
     * @param time       Amount of time to run for (seconds)
     */
    public VoltageDriveAction(double leftVolts, double rightVolts, long time)
    {this(leftVolts, rightVolts, time, true, RealClock.CLOCK, Robot.DRIVE_TRAIN); }


    /**
     * @param leftVolts
     * @param rightVolts
     * @param time       Amount of time to run for (seconds)
     */
    public VoltageDriveAction(double leftVolts, double rightVolts, long time, boolean brake, Clock clock, DriveTrain dt)
    {
        super(Constants.DEFAULT_ACTION_PERIOD, Constants.DEFAULT_ACTION_PERIOD_UNIT, time, TimeUnit.MILLISECONDS);

        this.leftVolts = leftVolts;
        this.rightVolts = rightVolts;
        this.brake = brake;

        stopwatch = new Stopwatch(clock);
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
        stopwatch.init();
    }

    @Override
    public void execute()
    {
        dt.runMotorsVoltage(leftVolts, rightVolts);
    }

    @Override
    public void end() throws Exception
    {
        if(brake) {
            dt.runMotorsVoltage(0, 0);
        }
        dt.giveBack();
    }
}
