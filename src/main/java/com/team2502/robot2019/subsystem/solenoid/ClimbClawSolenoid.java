package com.team2502.robot2019.subsystem.solenoid;

import com.team2502.robot2019.RobotMap;
import com.team2502.robot2019.utils.NonDefaultSubsystem;
import edu.wpi.first.wpilibj.Solenoid;

public class ClimbClawSolenoid extends NonDefaultSubsystem
{
    private final Solenoid flipOuts;

    private boolean toggleState = false;

    public ClimbClawSolenoid()
    {
        flipOuts = new Solenoid(RobotMap.Solenoid.FLIP_OUTS);

        // Starting position
        flipOuts.set(false);
    }

    public void toggle()
    {
        flipOuts.set(toggleState = !toggleState);
    }

    public void set(boolean open)
    {
        flipOuts.set(toggleState = open);
    }

    public boolean isOut() { return toggleState; }
}
