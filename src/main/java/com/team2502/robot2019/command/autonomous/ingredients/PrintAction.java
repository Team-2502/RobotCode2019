package com.team2502.robot2019.command.autonomous.ingredients;

import com.github.ezauton.core.action.PeriodicAction;

public class PrintAction extends PeriodicAction {

    @Override
    protected void execute() {
        System.out.println("hmm");
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
