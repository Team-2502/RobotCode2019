package com.team2502.robot2019.command.teleop;

import com.team2502.robot2019.Robot;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class SwitchDriveCommand extends InstantCommand
{

    public SwitchDriveCommand()
    { requires(Robot.DRIVE_TRAIN); }

    @Override
    protected void execute(){ Robot.DRIVE_TRAIN.setForward(!Robot.DRIVE_TRAIN.isForward());}
}
