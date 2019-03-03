package com.team2502.robot2019.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;

public class CrawlerSubsystem extends Subsystem
{
    private final WPI_TalonSRX rightClaw;
    private final WPI_TalonSRX leftClaw;

    public CrawlerSubsystem()
    {
        rightClaw = new WPI_TalonSRX(RobotMap.Motor.CLIMBER_CLAW_RIGHT);
        leftClaw = new WPI_TalonSRX(RobotMap.Motor.CLIMBER_CLAW_LEFT);
    }

    /**
     * Drives the "crawling" window motors in specified direction at full speed.
     * The speed will always be 100% (-1.0 or 1.0), because there is no reasion to
     * have the robot crawl slower than its mechanical limitation.
     */
    public void crawl()
    {
        leftClaw.set(ControlMode.PercentOutput, Constants.Physical.Crawler.SPEED_FWD);
        rightClaw.set(ControlMode.PercentOutput, Constants.Physical.Crawler.SPEED_FWD);
    }

    /**
     * Stops the crawling window motors (sets PercentOutput to 0.0).
     */
    public void stopCrawl()
    {
        leftClaw.set(ControlMode.PercentOutput, 0.0D);
        rightClaw.set(ControlMode.PercentOutput, 0.0D);
    }

    public void stop() {
        stopCrawl();
    }

    @Override
    protected void initDefaultCommand()
    {

    }
}
