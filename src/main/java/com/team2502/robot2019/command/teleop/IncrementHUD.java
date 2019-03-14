package com.team2502.robot2019.command.teleop;

import com.team2502.robot2019.Robot;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class IncrementHUD extends InstantCommand
{
    private final int id;
    private final boolean left;

    public IncrementHUD(int id, boolean left)
    {
        this.id = id;
        this.left = left;
    }

    @Override
    protected void execute()
    {
        Robot.SCORING_HUD.increment(id, left);
    }
}
