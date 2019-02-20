package com.team2502.robot2019.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.team2502.robot2019.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;

public class ClimberSubsystem extends Subsystem
{
    private final WPI_TalonSRX climber;
    private final WPI_TalonSRX rightClaw;
    private final WPI_TalonSRX leftClaw;

    public ClimberSubsystem()
    {
        climber = new WPI_TalonSRX(RobotMap.Motor.CLIMBER_RIGHT);

        rightClaw = new WPI_TalonSRX(RobotMap.Motor.CLIMBER_CLAW_RIGHT);
        leftClaw = new WPI_TalonSRX(RobotMap.Motor.CLIMBER_CLAW_LEFT);
    }

    public void climb(boolean forwards)
    {
        climber.set(ControlMode.PercentOutput, forwards ? 1.0D : -1.0D);
    }

    public void stopClimb()
    {
        climber.set(ControlMode.PercentOutput, 0.0D);
    }

    public void crawl(boolean forwards)
    {
        leftClaw.set(ControlMode.PercentOutput, forwards ? 1.0D : -1.0D);
        rightClaw.set(ControlMode.PercentOutput, forwards ? 1.0D : -1.0D);
    }

    public void stopCrawl()
    {
        leftClaw.set(ControlMode.PercentOutput, 0.0D);
        rightClaw.set(ControlMode.PercentOutput, 0.0D);
    }

    public void stop()
    {
        climber.set(ControlMode.PercentOutput, 0.0D);
        leftClaw.set(ControlMode.PercentOutput, 0.0D);
        rightClaw.set(ControlMode.PercentOutput, 0.0D);
    }

    @Override
    protected void initDefaultCommand()
    { }
}
