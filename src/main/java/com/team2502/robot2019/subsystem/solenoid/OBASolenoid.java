package com.team2502.robot2019.subsystem.solenoid;

import com.team2502.robot2019.RobotMap;
import com.team2502.robot2019.utils.NonDefaultSubsystem;
import edu.wpi.first.wpilibj.Solenoid;

public class OBASolenoid extends NonDefaultSubsystem
{
    private final Solenoid oba;

    private boolean toggleState = false;

    public OBASolenoid()
    {
        oba = new Solenoid(RobotMap.Solenoid.OBA);

        // Starting position
        oba.set(false);
    }

    public void toggle()
    {
        oba.set(toggleState = !toggleState);
    }

    public void set(boolean open)
    {
        oba.set(toggleState = open);
    }

    public boolean isOut() { return toggleState; }
}
