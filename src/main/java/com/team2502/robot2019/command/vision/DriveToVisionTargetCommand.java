package com.team2502.robot2019.command.vision;

import com.team2502.robot2019.command.autonomous.ingredients.DriveStraightWithGyroCommand;
import com.team2502.robot2019.command.autonomous.ingredients.VelocityDriveCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class DriveToVisionTargetCommand extends CommandGroup
{
    public DriveToVisionTargetCommand() {
        addSequential(new GoToTargetLimelight());
        addSequential(new DriveStraightWithGyroCommand(2, 3));
    }
}
