package com.team2502.robot2019.command.vision;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class DriveToVisionTargetCommand extends CommandGroup
{
    public DriveToVisionTargetCommand() {
        addSequential(new GoToTargetLimelight());
    }
}
