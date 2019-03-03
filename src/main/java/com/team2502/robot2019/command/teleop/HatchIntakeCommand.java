package com.team2502.robot2019.command.teleop;
import com.team2502.robot2019.Robot;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class HatchIntakeCommand extends InstantCommand {

    public HatchIntakeCommand() { requires(Robot.HATCH_INTAKE); }

    @Override
    protected void execute(){ Robot.HATCH_INTAKE.toggleHatchIntake(); }
}

