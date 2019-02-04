package com.team2502.robot2019.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.team2502.robot2019.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;

public class ClimberSubsystem extends Subsystem
{
    private final WPI_TalonSRX leftClimber;
    private final WPI_TalonSRX rightClimber;

    public ClimberSubsystem()
    {
        leftClimber = new WPI_TalonSRX(RobotMap.Motor.CLIMBER_LEFT);
        rightClimber = new WPI_TalonSRX(RobotMap.Motor.CLIMBER_RIGHT);
    }

    public void climb(boolean forwards)
    {
        leftClimber.set(ControlMode.PercentOutput, forwards ? 1.0D : -1.0D);
        rightClimber.set(ControlMode.PercentOutput, forwards ? 1.0D : -1.0D);
    }

    public void stop()
    {
        leftClimber.set(ControlMode.PercentOutput, 0.0D);
        rightClimber.set(ControlMode.PercentOutput, 0.0D);
    }

    @Override
    protected void initDefaultCommand()
    { }
}
