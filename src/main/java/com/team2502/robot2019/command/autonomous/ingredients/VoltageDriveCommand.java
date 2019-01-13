package com.team2502.robot2019.command.autonomous.ingredients;

import com.team2502.robot2019.Robot;
import edu.wpi.first.wpilibj.command.TimedCommand;

public class VoltageDriveCommand extends TimedCommand
{
    private final double leftVolts;
    private final double rightVolts;
    private final boolean brake;

    /**
 * @param leftVolts
     * @param rightVolts
 * @param time       Amount of time to run for (seconds)
     */
    public VoltageDriveCommand(double leftVolts, double rightVolts, double time)
    {this(leftVolts, rightVolts, time, true);}

    /**
     * @param leftVolts
     * @param rightVolts
     * @param time       Amount of time to run for (seconds)
     */
    public VoltageDriveCommand(double leftVolts, double rightVolts, double time, boolean brake)
    {
        super(time);

        this.leftVolts = leftVolts;
        this.rightVolts = rightVolts;
        this.brake = brake;
        requires(Robot.DRIVE_TRAIN);
    }

    @Override
    protected void execute()
    {
        Robot.DRIVE_TRAIN.runMotorsVoltage(leftVolts, rightVolts);
    }

    @Override
    protected void end()
    {
        if(brake)
        {
            Robot.DRIVE_TRAIN.runMotorsVoltage(0, 0);
        }
    }
}
