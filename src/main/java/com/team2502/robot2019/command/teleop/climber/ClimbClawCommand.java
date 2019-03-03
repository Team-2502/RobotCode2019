package com.team2502.robot2019.command.teleop.climber;

import com.team2502.robot2019.Robot;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class ClimbClawCommand extends InstantCommand
{
    public ClimbClawCommand()
    {
        requires(Robot.CLIMB_CLAWS);
    }

    @Override
    protected void execute()
    {
        Robot.CLIMB_CLAWS.toggle();
    }
}
