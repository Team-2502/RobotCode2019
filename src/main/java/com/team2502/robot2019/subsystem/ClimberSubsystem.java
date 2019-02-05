package com.team2502.robot2019.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.team2502.robot2019.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;

public class ClimberSubsystem extends Subsystem
{
    public enum ClimberSide
    {
        RIGHT,
        LEFT,
        BOTH
    }


    private final WPI_TalonSRX leftClimber;
    private final WPI_TalonSRX rightClimber;

    public ClimberSubsystem()
    {
        leftClimber = new WPI_TalonSRX(RobotMap.Motor.CLIMBER_LEFT);
        rightClimber = new WPI_TalonSRX(RobotMap.Motor.CLIMBER_RIGHT);
    }

    public void climb(ClimberSide sides, boolean forwards)
    {
        switch (sides)
        {
            case RIGHT:
                rightClimber.set(ControlMode.PercentOutput, forwards ? 1.0D : -1.0D);
                break;
            case LEFT:
                leftClimber.set(ControlMode.PercentOutput, forwards ? 1.0D : -1.0D);
                break;
            case BOTH:
                leftClimber.set(ControlMode.PercentOutput, forwards ? 1.0D : -1.0D);
                rightClimber.set(ControlMode.PercentOutput, forwards ? 1.0D : -1.0D);
                break;
        }
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
