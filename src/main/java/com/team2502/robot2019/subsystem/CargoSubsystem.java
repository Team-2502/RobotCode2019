package com.team2502.robot2019.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.team2502.robot2019.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;

public class CargoSubsystem extends Subsystem
{

    private final WPI_TalonSRX upperBelt;
    private final WPI_TalonSRX lowerBelt;

    public CargoSubsystem()
    {
        upperBelt = new WPI_TalonSRX(RobotMap.Motor.CARGO_UPPER_BELT);
        lowerBelt = new WPI_TalonSRX(RobotMap.Motor.CARGO_LOWER_BELT);
    }

    public void runIntake(double speed)
    {
        upperBelt.set(ControlMode.PercentOutput, speed);
        lowerBelt.set(ControlMode.PercentOutput, -speed);
    }

    public void stopIntake()
    {
        runIntake(0D);
    }

    @Override
    protected void initDefaultCommand(){}
}
