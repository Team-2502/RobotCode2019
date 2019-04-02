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
    /**
     * Used to determine the part of the active to run: external (OBA), internal (top pulleys), or both
     */
    public enum Section
    {
        INTERNAL,
        EXTERNAL,
        BOTH
    }

    private final WPI_TalonSRX tunnel;
    private final WPI_TalonSRX intake;

    /**
     * Creates a CargoSubsystem with two WPI_TalonSRX motor controllers
     * (tunnel and intake).
     */
    public CargoSubsystem()
    {
        tunnel = new WPI_TalonSRX(RobotMap.Motor.CARGO_UPPER_BELT);
        intake = new WPI_TalonSRX(RobotMap.Motor.CARGO_LOWER_BELT);

        tunnel.setInverted(true);
        intake.setInverted(true);
    }

    /**
     * Runs a specified section of the cargo intake at a particular speed (includes direction).
     * @param section Whether to run the INTERNAL, EXTERNAL, or BOTH motors from the Section enum.
     * @param speed The speed (-1.0 to 1.0) at which to run the intake.
     */
    public void runIntake(Section section, double speed) {
        switch(section) {
            case INTERNAL:
                runTop(speed);
                break;
            case EXTERNAL:
                runBottom(speed);
                break;
            case BOTH:
                if (speed > 0)
                {
                    runTop(Constants.Physical.CargoActive.SPEED_FWD);
                    runBottom(Constants.Physical.OverBumperActive.SPEED_FWD);
                }

                else
                {
                    runTop(Constants.Physical.CargoActive.SPEED_BWD);
                    runBottom(Constants.Physical.OverBumperActive.SPEED_BWD);
                }
                break;
        }
    }


    public void runIntake(boolean forward)
    {
        runIntake(Section.BOTH, forward ? 1.0D : -1.0D);
    }

    /**
     * Runs both the top and bottom belts at the specified speed.
     * @param speed The speed (-1.0 to 1.0) at which to run the intake.
     */
    public void runBoth(double speed)
    {
        tunnel.set(ControlMode.PercentOutput, speed);
        intake.set(ControlMode.PercentOutput, -speed);
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
        tunnel.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Runs only the bottom belt of the intake.
     * @param speed The speed (-1.0 to 1.0) at which to run the belt.
     */
    public void runBottom(double speed)
    {
        intake.set(ControlMode.PercentOutput, -speed);

    }

    @Override
    protected void initDefaultCommand() {}
}
