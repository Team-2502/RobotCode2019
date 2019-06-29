package com.team2502.robot2019.command.vision;

import com.team2502.robot2019.Robot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GoToTargetLimelightSimple extends Command {

    private boolean LimelightHasValidTarget;

    // These numbers must be tuned for your Robot!  Be careful!
    private final double STEER_K = SmartDashboard.getNumber("LL_Steer_K", 0.03);               // how hard to turn toward the target
    private final double DRIVE_K = SmartDashboard.getNumber("LL_Drive_K", 0.26);               // how hard to drive fwd toward the target
    private final double DESIRED_TARGET_AREA = SmartDashboard.getNumber("LL_DES_T_A", 13.0);   // Area of the target when the robot reaches the wall
    private final double MAX_SPEED = SmartDashboard.getNumber("LL_MAX_SPEED", 1);              // Simple speed limit so we don't drive too fast

    private double tv;                                      // Does the Limelight see the target? Either 1 for yes or 0 for no.
    private double tx;                                      // Horizontal Offset From Cross hair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
    private double ty;                                      // Vertical Offset From Cross hair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
    private double ta;                                      // Target Area (0% of image to 100% of image)
    private double ts;                                      // Skew or rotation (-90 degrees to 0 degrees)

    private final DifferentialDrive DIFF_DRIVE = Robot.DRIVE_TRAIN.diffDrive;

    public void UpdateLimelightTracking() {

        tv = Robot.LL_TV.getDouble(0);
        tx = Robot.LL_TX.getDouble(0);
        ty = Robot.LL_TY.getDouble(0);
        ta = Robot.LL_TA.getDouble(0);
        ts = Robot.LL_TS.getDouble(0);

        SmartDashboard.putNumber("LimelightTX", tx);
        SmartDashboard.putNumber("LimelightTArea", ta);
        SmartDashboard.putNumber("LimelightTY", ty);
        SmartDashboard.putNumber("LimelightTS", ts);
        SmartDashboard.putNumber("LimelightTV", tv);

    }

    public GoToTargetLimelightSimple()
    {
        requires(Robot.DRIVE_TRAIN);
    }

    @Override
    protected void initialize()
    {
        UpdateLimelightTracking();
    }

    @Override
    protected void execute()
    {

        UpdateLimelightTracking();

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
        // Finish if can't see target
        return !(tv == 0);

    }

    @Override
    protected void end()
    {

    }
}
