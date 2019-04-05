package com.team2502.robot2019.command.teleop;

import com.team2502.robot2019.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class DriveCommand extends Command
{
    public DriveCommand() {
        requires(Robot.DRIVE_TRAIN);
    }

    @Override
    protected void execute()
    {
        Robot.DRIVE_TRAIN.teleopDrive();
    }

    @Override
    protected boolean isFinished()
    {
        return false;
    }
}
