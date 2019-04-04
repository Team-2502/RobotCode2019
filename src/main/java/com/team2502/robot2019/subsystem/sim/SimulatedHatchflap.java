package com.team2502.robot2019.subsystem.sim;

import com.team2502.robot2019.subsystem.interfaces.HatchFlap;

public class SimulatedHatchflap implements HatchFlap
{
    private boolean deployed = false;

    @Override
    public void toggleHatchIntake()
    {
        deployed = ! deployed;

    }

    @Override
    public void setHatchIntake(boolean open)
    {
        deployed = open;
    }
}
