package com.team2502.robot2019.command.teleop.cargoactive;

import com.team2502.robot2019.Robot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;

// ###############
// TODO: Make it deploy OBA

//

public class ToggleOBACommand extends Command
{

    public ToggleOBACommand()
    {
        requires(Robot.OBA);
    }

    @Override
    protected void execute() { Robot.OBA.toggleOBA(); }

    @Override
    protected boolean isFinished()
    { return true; }

}