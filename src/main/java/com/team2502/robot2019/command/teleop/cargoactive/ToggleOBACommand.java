package com.team2502.robot2019.command.teleop.cargoactive;

import com.team2502.robot2019.Robot;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class ToggleOBACommand extends InstantCommand
{
    public ToggleOBACommand() { requires(Robot.OBA); }

    @Override
    protected void execute(){ Robot.OBA.toggle(); }
}