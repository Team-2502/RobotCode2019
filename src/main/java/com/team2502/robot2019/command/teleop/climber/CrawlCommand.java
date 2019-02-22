package com.team2502.robot2019.command.teleop.climber;

import com.team2502.robot2019.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class CrawlCommand extends Command
{
    private boolean forwards;

    public CrawlCommand(boolean forwards)
    {
        this.forwards = forwards;
    }

    @Override
    protected void execute()
    {
        Robot.CLIMBER.crawl(forwards);
    }

    @Override
    protected void end()
    {
        Robot.CLIMBER.stopCrawl();
    }

    @Override
    protected boolean isFinished()
    {
        return false;
    }
}