package com.team2502.robot2019.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.team2502.robot2019.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;

public class CargoSubsystem extends Subsystem
{
    public enum Belt
    {
        TOP,
        BOTTOM,
        BOTH;
    }

    private final WPI_TalonSRX upperBelt;
    private final WPI_TalonSRX lowerBelt;

    public CargoSubsystem()
    {
        upperBelt = new WPI_TalonSRX(RobotMap.Motor.CARGO_UPPER_BELT);
        lowerBelt = new WPI_TalonSRX(RobotMap.Motor.CARGO_LOWER_BELT);
    }

    public void runIntake(Belt belt, double speed) {
        switch(belt) {
            case TOP:
                runTop(speed);
                break;
            case BOTTOM:
                runBottom(speed);
                break;
            case BOTH:
                runBoth(speed);
                break;
        }
    }
    public void runBoth(double speed)
    {
        upperBelt.set(ControlMode.PercentOutput, speed);
        lowerBelt.set(ControlMode.PercentOutput, -speed);
    }

    public void stopIntake()
    {
        runBoth(0D);
    }

    @Override
    protected void initDefaultCommand() {}

    public void runTop(double speed)
    {
        upperBelt.set(ControlMode.PercentOutput, speed);
    }

    public void runBottom(double speed)
    {
        lowerBelt.set(ControlMode.PercentOutput, -speed);

    }
}
