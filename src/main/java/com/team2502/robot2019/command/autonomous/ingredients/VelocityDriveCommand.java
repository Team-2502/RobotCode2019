package com.team2502.robot2019.command.autonomous.ingredients;

import com.team2502.robot2019.Robot;
import edu.wpi.first.wpilibj.command.TimedCommand;

public class VelocityDriveCommand extends TimedCommand
{
    private final double leftVel;
    private final double rightVel;
    private final boolean brake;

    /**
     * @param leftVel
     * @param rightVel
     * @param time     Amount of time to run for (seconds)
     */
    public VelocityDriveCommand(double leftVel, double rightVel, double time)
    {this(leftVel, rightVel, time, true);}

    /**
     * @param leftVel
     * @param rightVel
     * @param time     Amount of time to run for (seconds)
     */
    public VelocityDriveCommand(double leftVel, double rightVel, double time, boolean brake)
    {
        super(time);

        this.leftVel = leftVel;
        this.rightVel = rightVel;
        this.brake = brake;
        requires(Robot.DRIVE_TRAIN);
    }


    @Override
    protected void execute()
    {
        Robot.DRIVE_TRAIN.runMotorsVelocity(leftVel, rightVel);
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
