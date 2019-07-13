package com.team2502.robot2019.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Robot Cargo intake/outtake system is composed of two WPI_TalonSRX motor
 * controllers that control the top and bottom conveyor belts, respectively.
 */
public class CargoSubsystem extends Subsystem
{

    private final WPI_TalonSRX tunnel;

    /**
     * Creates a CargoSubsystem with two WPI_TalonSRX motor controllers
     * (tunnel and intake).
     */
    public CargoSubsystem()
    {
        tunnel = new WPI_TalonSRX(RobotMap.Motor.CARGO_UPPER_BELT);

        tunnel.setInverted(true);
    }

    /**
     * Stops the intake (sets PercentOutput to 0.0).
     */
    public void stopIntake()
    {
        runTop(0);
    }

    /**
     * Runs only the top belt of the intake.
     * @param speed The speed (-1.0 to 1.0) at which to run the belt.
     */
    public void runTop(double speed)
    {
        tunnel.set(ControlMode.PercentOutput, speed);
    }


    @Override
    protected void initDefaultCommand() {}
}
