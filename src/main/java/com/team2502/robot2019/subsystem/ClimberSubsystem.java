package com.team2502.robot2019.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.team2502.robot2019.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Robot climberRight consists of a winch CIM on a gearbox and two
 * window motors that "crawl" forwards when the robot has been lifted.
 */
public class ClimberSubsystem extends Subsystem
{
    private final WPI_TalonSRX climberRight;
    private final WPI_TalonSRX climberLeft;
    private final WPI_TalonSRX rightClaw;
    private final WPI_TalonSRX leftClaw;

    /**
     * Creates a ClimberSubsystem with 3 WPI_TalonSRX motor controllers (1 winch, 2 crawl).
     */
    public ClimberSubsystem()
    {
        climberRight = new WPI_TalonSRX(RobotMap.Motor.CLIMBER_RIGHT);
        climberLeft = new WPI_TalonSRX(RobotMap.Motor.CLIMBER_LEFT);

        rightClaw = new WPI_TalonSRX(RobotMap.Motor.CLIMBER_CLAW_RIGHT);
        leftClaw = new WPI_TalonSRX(RobotMap.Motor.CLIMBER_CLAW_LEFT);

        climberRight.setInverted(true);
        climberLeft.setInverted(false);
    }

    /**
     * Drives the winch motor in specified direction at full speed.
     * The speed will always be 100% (-1.0 or 1.0), because there is no reason to
     * have the robot climb slower than its mechanical limitation.
     * @param forwards Whether or not to drive in the "forwards" direction, which
     *                 correlates to an UPWARDS lift movement (true) or
     *                 DOWNWARDS lift movement (false).
     */
    public void climb(boolean forwards)
    {
        climberRight.set(ControlMode.PercentOutput, forwards ? 1.0D : -1.0D);
        climberLeft.set(ControlMode.PercentOutput, forwards ? 1.0D : -1.0D);
    }

    public void onlyRightClimb(boolean forwards)
    {
        climberRight.set(ControlMode.PercentOutput, forwards ? 1.0D : -1.0D);
    }

    public void onlyLeftClimb(boolean forwards)
    {
        climberLeft.set(ControlMode.PercentOutput, forwards ? 1.0D : -1.0D);
    }

    /**
     * Stops the winch motor (sets PercentOutput to 0.0).
     */
    public void stopClimb()
    {
        climberRight.set(ControlMode.PercentOutput, 0.0D);
        climberLeft.set(ControlMode.PercentOutput, 0.0D);
    }

    /**
     * Drives the "crawling" window motors in specified direction at full speed.
     * The speed will always be 100% (-1.0 or 1.0), because there is no reasion to
     * have the robot crawl slower than its mechanical limitation.
     * @param forwards Whether or not to drive in the "forwards" direction, which
     *                 correlates to a
     */
    public void crawl(boolean forwards)
    {
        leftClaw.set(ControlMode.PercentOutput, forwards ? 1.0D : -1.0D);
        rightClaw.set(ControlMode.PercentOutput, forwards ? 1.0D : -1.0D);
    }

    /**
     * Stops the crawling window motors (sets PercentOutput to 0.0).
     */
    public void stopCrawl()
    {
        leftClaw.set(ControlMode.PercentOutput, 0.0D);
        rightClaw.set(ControlMode.PercentOutput, 0.0D);
    }

    /**
     * Stops all motors in the climberRight subsystem by setting
     * PercentOutput to 0.0.
     */
    public void stop()
    {
        climberRight.set(ControlMode.PercentOutput, 0.0D);
        climberLeft.set(ControlMode.PercentOutput, 0.0D);
        leftClaw.set(ControlMode.PercentOutput, 0.0D);
        rightClaw.set(ControlMode.PercentOutput, 0.0D);
    }

    @Override
    protected void initDefaultCommand() { }
}
