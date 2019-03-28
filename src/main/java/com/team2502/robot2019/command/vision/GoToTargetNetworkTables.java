package com.team2502.robot2019.command.vision;

import com.github.ezauton.core.trajectory.geometry.ImmutableVector;
import com.google.common.util.concurrent.AtomicDouble;
import com.team2502.robot2019.Robot;
import com.team2502.robot2019.subsystem.vision.VisionData;
import com.team2502.robot2019.utils.CircularBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GoToTargetNetworkTables extends Command {

    /**
     * Name of the SmartDashboard item that lets you change speed
     */
    public static final String gttsc_speed = "gttsc_speed";

    /**
     * SPeed the robot should go at
     */
    private double totalSpeed = 3;

    /**
     * Instance of the PID Controller
     * <br>
     * Manages how the robot turns based on the offset
     */
    private PIDController pidController;

    /**
     * The desired difference between the wheel speeds. The PIDController writes to this AtomicDouble, then we read from it and drive accordingly.
     */
    private final AtomicDouble desiredWheelDifferential = new AtomicDouble();

    /**
     * The maximum possible offset that can occur. This has been measured to be 3 feet.
     */
    private final double max_offset = 3;

    /**
     * Data class that contains current offset
     */
    private VisionData visionInfo;

    /**
     * Circular buffer that keeps track of offsets in past loop iterations
     */
    private CircularBuffer errorBuffer;

    public GoToTargetNetworkTables()
    {
        requires(Robot.DRIVE_TRAIN);

        visionInfo = new VisionData(0,0,0);

        updateVisionData();

        pidController = new PIDController(8, .0005, .5, new PIDSource() {
            PIDSourceType sourceType = PIDSourceType.kDisplacement;
            @Override
            public void setPIDSourceType(PIDSourceType pidSource)
            {
                throw new RuntimeException("You can't do that! (attempted to setPIDSourceType on anon class)");
            }

            @Override
            public PIDSourceType getPIDSourceType()
            {
                return sourceType;
            }

            @Override
            public double pidGet()
            {
                double lastPidGet = Math.min(max_offset, Math.max(-max_offset, visionInfo.getPos().get(0)));
                System.out.println("lastPidGet = " + lastPidGet);
                return lastPidGet;
            }
        }, desiredWheelDifferential::set);


        SmartDashboard.putData("gototargetstupidcommand", pidController);
        SmartDashboard.putNumber(gttsc_speed, totalSpeed);

    }

    private void updateVisionData()
    {
        double tvecs1 = Robot.tvecs1Entry.getDouble(-9001);
        double tvecs2 = Robot.tvecs2Entry.getDouble(-9001);
        visionInfo.pos = new ImmutableVector(tvecs1, tvecs2);
        visionInfo.angle = Robot.angleEntry.getDouble(-9001);

        Robot.seesTarget.setBoolean(! (visionInfo.pos.get(0) == -9001 || visionInfo.angle == -9001));
    }

    @Override
    protected void initialize()
    {
        DriverStation.reportWarning("Go to target initialized", false);
        totalSpeed = SmartDashboard.getNumber(gttsc_speed, totalSpeed);
        pidController.setSetpoint(0);

        pidController.setInputRange(-max_offset, max_offset);
        pidController.setOutputRange(-totalSpeed, totalSpeed);
        pidController.setAbsoluteTolerance(.01);
        pidController.enable();

        errorBuffer = new CircularBuffer(800);
    }

    @Override
    protected void execute()
    {
        updateVisionData();

        SmartDashboard.putNumber("desiredratio", desiredWheelDifferential.get());
        SmartDashboard.putNumber("socket", visionInfo.getPos().get(0));

        if(visionInfo.isMeaningful())
        {
            double velRight = desiredWheelDifferential.get() / 2;
            double velLeft = - desiredWheelDifferential.get() / 2;
            SmartDashboard.putNumber("velLeft", velLeft);
            SmartDashboard.putNumber("velRight", velRight);
            Robot.DRIVE_TRAIN.runMotorsVelocity(velLeft, velRight);
            errorBuffer.addValue(visionInfo.pos.get(0));
        }
        else
        {
            Robot.DRIVE_TRAIN.driveSpeed(0);
            System.out.println("not meaningful");
        }

    }

    @Override
    protected boolean isFinished()
    {
        return false;
//        return Math.abs(errorBuffer.getAverage()) <= 0.002;
    }

    @Override
    protected void end()
    {
        pidController.disable();
        Robot.DRIVE_TRAIN.driveSpeed(0);
        DriverStation.reportError("ended", false);
    }
}
