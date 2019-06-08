package com.team2502.robot2019.command.vision;

import com.team2502.robot2019.Robot;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class GoToTargetLimelight extends Command {

    private boolean LimelightHasValidTarget;

    // These numbers must be tuned for your Robot!  Be careful!
    private final double STEER_K = 0.03;                    // how hard to turn toward the target
    private final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
    private final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
    private final double MAX_SPEED = 3;                     // Simple speed limit so we don't drive too fast
    private double tv;                                      // Does the Limelight see the target? Either 1 for yes or 0 for no.
    private double tx;                                      // Horizontal Offset From Cross hair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
    private double ty;                                      // Vertical Offset From Cross hair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
    private double ta;                                      // Target Area (0% of image to 100% of image)
    private double ts;                                      // Skew or rotation (-90 degrees to 0 degrees)
    private final DifferentialDrive DIFF_DRIVE = Robot.DRIVE_TRAIN.diffDrive;

    public void Update_Limelight_Tracking() {

        tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
        ts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    }

    public GoToTargetLimelight()
    {
        requires(Robot.DRIVE_TRAIN);
    }

    @Override
    protected void initialize()
    {
        Update_Limelight_Tracking();
    }

    @Override
    protected void execute()
    {

        Update_Limelight_Tracking();
        // Start with proportional steering
        double LimelightSteerCommand = tx * STEER_K;

        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_SPEED)
        {
            drive_cmd = MAX_SPEED;
        }
        double LimelightDriveCommand = drive_cmd;

        DIFF_DRIVE.arcadeDrive(LimelightDriveCommand, LimelightSteerCommand);

    }

    @Override
    protected boolean isFinished()
    {
        if(tv == 0)
        {
            return true;
        }
        else if(tv == 1)
        {
            return false;
        }
        else
        {
            DriverStation.reportWarning("tv is not equal to one or zero!", false);
            return true;
        }
    }

    @Override
    protected void end()
    {

    }
}
