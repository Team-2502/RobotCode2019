package com.team2502.robot2019.command.autonomous.ingredients;

import com.github.ezauton.core.action.ActionRunInfo;
import com.github.ezauton.core.action.BaseAction;
import com.team2502.robot2019.Robot;

public class SetHatchIntakeAction extends BaseAction
{
    private final boolean out;

    public SetHatchIntakeAction(boolean out) {
        this.out = out;
    }

    @Override
    public void run(ActionRunInfo info)
    {
        Robot.HATCH_INTAKE.set(out);
    }
}
