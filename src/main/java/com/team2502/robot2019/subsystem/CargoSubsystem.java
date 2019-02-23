package com.team2502.robot2019.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.team2502.robot2019.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Robot Cargo intake/outtake system is composed of two WPI_TalonSRX motor
 * controllers that control the top and bottom conveyor belts, respectively.
 */
public class CargoSubsystem extends Subsystem
{
    /**
     * Used to determine to activate just the top, bottom, or both belts.
     */
    public enum Belt
    {
        TOP,
        BOTTOM,
        BOTH
    }

    private final WPI_TalonSRX upperBelt;
    private final WPI_TalonSRX lowerBelt;

    /**
     * Creates a CargoSubsystem with two WPI_TalonSRX motor controllers
     * (upperBelt and lowerBelt).
     */
    public CargoSubsystem()
    {
        upperBelt = new WPI_TalonSRX(RobotMap.Motor.CARGO_UPPER_BELT);
        lowerBelt = new WPI_TalonSRX(RobotMap.Motor.CARGO_LOWER_BELT);
    }

    /**
     * Runs a specified belt of the cargo intake at a particular speed (includes direction).
     * @param belt Whether to run the TOP, BOTTOM, or BOTH belts from the Belt enum.
     * @param speed The speed (-1.0 to 1.0) at which to run the intake.
     */
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

    /**
     * Runs both the top and bottom belts at the specified speed.
     * @param speed The speed (-1.0 to 1.0) at which to run the intake.
     */
    public void runBoth(double speed)
    {
        upperBelt.set(ControlMode.PercentOutput, speed);
        lowerBelt.set(ControlMode.PercentOutput, -speed);
    }

    /**
     * Stops the intake (sets PercentOutput to 0.0).
     */
    public void stopIntake()
    {
        runBoth(0D);
    }

    /**
     * Runs only the top belt of the intake.
     * @param speed The speed (-1.0 to 1.0) at which to run the belt.
     */
    public void runTop(double speed)
    {
        upperBelt.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Runs only the bottom belt of the intake.
     * @param speed The speed (-1.0 to 1.0) at which to run the belt.
     */
    public void runBottom(double speed)
    {
        lowerBelt.set(ControlMode.PercentOutput, -speed);

    }

    @Override
    protected void initDefaultCommand() {}
}
