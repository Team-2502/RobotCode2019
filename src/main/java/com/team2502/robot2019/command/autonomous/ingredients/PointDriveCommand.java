package com.team2502.robot2019.command.autonomous.ingredients;

import com.github.ezauton.core.trajectory.geometry.ImmutableVector;
import com.team2502.robot2019.Robot;
import edu.wpi.first.wpilibj.command.TimedCommand;

public class PointDriveCommand extends TimedCommand
{
    private final double speed;
    private final ImmutableVector point;
    private final boolean brake;


    /**
     * @param leftVel
     * @param rightVel
     * @param time       Amount of time to run for (seconds)
     */
    public PointDriveCommand(double speed, ImmutableVector point, double timeout, boolean brake)
    {
        super(timeout);
        this.speed = speed;
        this.point = point;


        this.brake = brake;
        requires(Robot.DRIVE_TRAIN);
    }

    @Override
    protected void initialize()
    {
    }

    @Override
    protected void execute()
    {
        Robot.DRIVE_TRAIN.update();
        Robot.DRIVE_TRAIN.driveTowardTransLoc(speed, point);
    }

    @Override
    protected boolean isFinished()
    {
        return Robot.DRIVE_TRAIN.getLocEstimator().estimateLocation().dist(point) < 0.1 || super.isTimedOut();
    }

    @Override
    protected void end()
    {
        if(brake)
        {
            Robot.DRIVE_TRAIN.runMotorsVelocity(0, 0);
        }
    }
}
