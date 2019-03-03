package com.team2502.robot2019.command.teleop.climber;

import com.team2502.robot2019.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class CrawlCommand extends Command
{
    public CrawlCommand()
    {
        requires(Robot.CRAWLER);
    }

    @Override
    protected void execute()
    {
        Robot.CRAWLER.crawl();
    }

    @Override
    protected void end()
    {
        Robot.CRAWLER.stopCrawl();
    }

    @Override
    protected boolean isFinished()
    {
        return false;
    }
}
