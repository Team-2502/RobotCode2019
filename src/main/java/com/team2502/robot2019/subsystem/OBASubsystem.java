package com.team2502.robot2019.subsystem;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.team2502.robot2019.RobotMap;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;


/**
 * TODO for ravisha
 *
 * 1. Make an oba subsystem. It needs 1 motor (the oba motor) and one solenoid (the up down guy)
 *      - Needs methods to run the motor, trigger the solenoid
 * 2.
 */
public class OBASubsystem extends Subsystem
{

    private final WPI_TalonSRX obaMotor;
    private final Solenoid obaSolenoid;
    private boolean obaOpen  = false;

    public OBASubsystem() {
        obaMotor = new WPI_TalonSRX(RobotMap.Motor.OBA_MOTOR);
        obaSolenoid = new Solenoid(RobotMap.Solenoid.OBA_SOLENOID);

        obaSolenoid.set(false);
    }

    public void toggleOBA() { obaSolenoid.set(obaOpen = !obaOpen); }

    public void runOBA(double speed)  { obaMotor.set(ControlMode.PercentOutput, speed); }

    public void stopOBA()
    {
        runOBA(0);
    }


    @Override
    protected void initDefaultCommand()
    {}
}
