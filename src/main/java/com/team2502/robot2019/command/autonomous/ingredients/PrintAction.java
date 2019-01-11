package com.team2502.robot2019.command.autonomous.ingredients;

import com.github.ezauton.core.action.BackgroundAction;

import java.util.concurrent.TimeUnit;

public class PrintAction extends BackgroundAction {
    public PrintAction() {
        super(20, TimeUnit.MILLISECONDS);
    }

    @Override
    protected void execute() {
        System.out.println("print test");
    }
}
