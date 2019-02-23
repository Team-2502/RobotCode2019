package com.team2502.robot2019.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.github.ezauton.core.action.require.BaseResource;
import com.github.ezauton.core.actuators.IVelocityMotor;
import com.github.ezauton.core.localization.IRotationalLocationEstimator;
import com.github.ezauton.core.localization.Updateable;
import com.github.ezauton.core.localization.UpdateableGroup;
import com.github.ezauton.core.localization.estimators.EncoderRotationEstimator;
import com.github.ezauton.core.localization.sensors.Encoders;
import com.github.ezauton.core.localization.sensors.ITranslationalDistanceSensor;
import com.github.ezauton.core.localization.sensors.IVelocityEstimator;
import com.github.ezauton.core.robot.implemented.TankRobotTransLocDriveable;
import com.github.ezauton.core.trajectory.geometry.ImmutableVector;
import com.github.ezauton.core.utils.MathUtils;
import com.github.ezauton.wpilib.motors.ITypicalMotor;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.DashboardData;
import com.team2502.robot2019.RobotMap;
import com.team2502.robot2019.command.teleop.DriveCommand;
import com.team2502.robot2019.subsystem.interfaces.IDriveTrain;
import com.team2502.robot2019.utils.IPIDTunable;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DrivetrainSubsystem extends Subsystem implements IPIDTunable, IDriveTrain, Updateable, DashboardData.DashboardUpdater
{
    private final WPI_TalonSRX backLeft;
    private final WPI_TalonSRX backRight;

    public final WPI_TalonSRX frontLeft;
    public final WPI_TalonSRX frontRight;

    private final ITypicalMotor left;
    private final ITypicalMotor right;

    private final TankRobotTransLocDriveable trtls;
    private final EncoderRotationEstimator locEst;
    private final IRotationalLocationEstimator rotEst;
    private final IVelocityEstimator velEst;

    private final PigeonIMU pigeonIMU;


    private double kP = Constants.Physical.DriveTrain.DEFAULT_KP;
    private double kI = Constants.Physical.DriveTrain.DEFAULT_KI;
    private double kD = Constants.Physical.DriveTrain.DEFAULT_KD;
    private double kF = Constants.Physical.DriveTrain.DEFAULT_KF;

    private boolean forward = true;

    private final BaseResource resource = new BaseResource();
    private final UpdateableGroup updateableGroup;
    private final ITranslationalDistanceSensor leftSensor;
    private final ITranslationalDistanceSensor rightSensor;
    private int maxVel = 0;

    public DrivetrainSubsystem()
    {
        backLeft = new WPI_TalonSRX(RobotMap.Motor.DRIVE_TRAIN_BACK_LEFT);
        backRight = new WPI_TalonSRX(RobotMap.Motor.DRIVE_TRAIN_BACK_RIGHT);
        frontLeft = new WPI_TalonSRX(RobotMap.Motor.DRIVE_TRAIN_FRONT_LEFT);
        frontRight = new WPI_TalonSRX(RobotMap.Motor.DRIVE_TRAIN_FRONT_RIGHT);

        pigeonIMU = new PigeonIMU(backLeft);

        frontLeft.setSelectedSensorPosition(0);
        frontRight.setSelectedSensorPosition(0);
        frontLeft.setSensorPhase(true);

        frontLeft.configClosedloopRamp(0.05);
        frontRight.configClosedloopRamp(0.05);

        frontLeft.configOpenloopRamp(0.05);
        frontRight.configOpenloopRamp(0.05);

        right = new ITypicalMotor()
        {
            public void runVelocity(double targetVelocity)
            {
                this.makeSlavesFollowMaster();
                frontRight.set(ControlMode.Velocity, targetVelocity / Constants.Physical.DriveTrain.ENC_UNITS_TO_FPS);
            }

            public void runVoltage(double targetVoltage)
            {
                this.makeSlavesFollowMaster();
                frontRight.set(ControlMode.PercentOutput, targetVoltage);
            }

            public double getPosition()
            {
                return (double) frontRight.getSelectedSensorPosition(0) * Constants.Physical.DriveTrain.ENC_UNITS_TO_FEET;
            }

            public double getVelocity()
            {
                return (double) frontRight.getSelectedSensorVelocity(0) * Constants.Physical.DriveTrain.ENC_UNITS_TO_FPS;
            }

            private void makeSlavesFollowMaster()
            {
                backRight.follow(frontRight);
            }
        };


        left = new ITypicalMotor()
        {
            public void runVelocity(double targetVelocity)
            {
                this.makeSlavesFollowMaster();
                frontLeft.set(ControlMode.Velocity, targetVelocity / Constants.Physical.DriveTrain.ENC_UNITS_TO_FPS);
            }

            public void runVoltage(double targetVoltage)
            {
                this.makeSlavesFollowMaster();
                frontLeft.set(ControlMode.PercentOutput, targetVoltage);
            }

            public double getPosition()
            {
                return (double) frontLeft.getSelectedSensorPosition(0) * Constants.Physical.DriveTrain.ENC_UNITS_TO_FEET;
            }

            public double getVelocity()
            {
                return (double) frontLeft.getSelectedSensorVelocity(0) * Constants.Physical.DriveTrain.ENC_UNITS_TO_FPS;
            }

            private void makeSlavesFollowMaster()
            {
                backLeft.follow(frontLeft);
            }
        };

        leftSensor = Encoders.toTranslationalDistanceSensor(
                1,
                1,
                left
                                                           );
        rightSensor = Encoders.toTranslationalDistanceSensor(
                1,
                1,
                right
                                                            );

        pigeonIMU.setYaw(0);
        rotEst = () -> {
            double[] ypr = new double[3];
            pigeonIMU.getYawPitchRoll(ypr);
            return -MathUtils.Kinematics.navXToRad(ypr[0]);
        };
        velEst = () -> (leftSensor.getVelocity() + rightSensor.getVelocity()) / 2;

        locEst = new EncoderRotationEstimator(rotEst, new ITranslationalDistanceSensor()
        {
            @Override
            public double getPosition()
            {
                return (leftSensor.getPosition() + rightSensor.getPosition()) / 2;
            }

            @Override
            public double getVelocity()
            {
                return (leftSensor.getVelocity() + rightSensor.getVelocity()) / 2;
            }
        });
//        locEst = new TankRobotEncoderEncoderEstimator(leftSensor, rightSensor, Constants.Physical.DriveTrain.TANK_ROBOT_CONSTANTS);
        locEst.reset();


        trtls = new TankRobotTransLocDriveable(left, right, locEst, rotEst, Constants.Physical.DriveTrain.TANK_ROBOT_CONSTANTS);

        // DO NOT CHANGE
        backLeft.follow(frontLeft);
        backRight.follow(frontRight);

        updateableGroup = new UpdateableGroup(locEst);

        DashboardData.addUpdater(this);

        applyPID();
    }

    /**
     * Runs the drive train motors with the specified control mode, at the specified speeds for each side.
     *
     * @param controlMode the control mode to use.
     * @param leftVal     value/speed for left side of DT.
     * @param rightVal    value/speed for right side of DT.
     * @see com.ctre.phoenix.motorcontrol
     */
    public void runMotors(ControlMode controlMode, double leftVal, double rightVal)
    {
        frontLeft.set(controlMode, leftVal);
        frontRight.set(controlMode, rightVal);
    }

    /**
     * Runs the drive train in velocity mode (values are native encoder units per 100ms)
     *
     * @param leftVal  speed of left side of DT in FT/SEC.
     * @param rightVal speed of right side of DT in FT/SEC.
     */
    public void runMotorsVelocity(double leftVal, double rightVal)
    {
        left.runVelocity(leftVal);
        right.runVelocity(rightVal);
    }

    @Deprecated
    public void runMotorsVoltage(double leftVolts, double rightVolts)
    {
        left.runVoltage(leftVolts);
        right.runVoltage(rightVolts);
    }

    @Override
    public IVelocityMotor getLeft()
    {
        return left;
    }

    @Override
    public IVelocityMotor getRight()
    {
        return right;
    }

    @Override
    public EncoderRotationEstimator getLocEstimator()
    {
        return locEst;
    }

    @Override
    public IRotationalLocationEstimator getRotEstimator()
    {
        return rotEst;
    }

    @Override
    public IVelocityEstimator getVelocityEstimator()
    {
        return velEst;
    }

    public WPI_TalonSRX getFrontLeft()
    {
        return frontLeft;
    }

    public WPI_TalonSRX getFrontRight()
    {
        return frontRight;
    }

    @Override
    public double getkP()
    {
        return kP;
    }

    @Override
    public void setkP(double kP)
    {
        this.kP = kP;
        applyPID();
    }

    @Override
    public double getkI()
    {
        return kI;
    }

    @Override
    public void setkI(double kI)
    {
        this.kI = kI;
        applyPID();
    }

    @Override
    public double getkD()
    {
        return kD;
    }

    @Override
    public void setkD(double kD)
    {
        this.kD = kD;
        applyPID();
    }

    @Override
    public double getkF()
    {
        return kF;
    }

    @Override
    public void setkF(double kF)
    {
        this.kF = kF;
        applyPID();
    }

    @Override
    public void setPID(double kP, double kI, double kD)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        applyPID();
    }

    public void applyPID()
    {
        frontLeft.selectProfileSlot(0, 0);
        frontRight.selectProfileSlot(0, 0);

        frontLeft.config_kP(0, kP, Constants.INIT_TIMEOUT);
        frontLeft.config_kI(0, kI, Constants.INIT_TIMEOUT);
        frontLeft.config_kD(0, kD, Constants.INIT_TIMEOUT);
        frontLeft.config_kF(0, kF, Constants.INIT_TIMEOUT);

        frontRight.config_kP(0, kP, Constants.INIT_TIMEOUT);
        frontRight.config_kI(0, kI, Constants.INIT_TIMEOUT);
        frontRight.config_kD(0, kD, Constants.INIT_TIMEOUT);
        frontRight.config_kF(0, kF, Constants.INIT_TIMEOUT);
    }

    @Override
    public boolean driveTowardTransLoc(double speed, ImmutableVector loc)
    {
        return trtls.driveTowardTransLoc(speed, loc);
    }

    @Override
    public boolean driveSpeed(double speed)
    {
        return trtls.driveSpeed(speed);
    }


    @Override
    public BaseResource getResource()
    {
        return resource;
    }

    @Override
    public boolean update()
    {
        return updateableGroup.update();
    }

    public boolean isForward()
    {
        return forward;
    }

    public void setForward(boolean forward)
    {
        this.forward = forward;
    }

    @Override
    public void updateDashboard()
    {
        update();
        SmartDashboard.putNumber("rot", rotEst.estimateHeading());
        SmartDashboard.putNumber("left vel", leftSensor.getVelocity());
        SmartDashboard.putNumber("right vel", rightSensor.getVelocity());
        SmartDashboard.putNumber("left pos", frontLeft.getSelectedSensorPosition());
        SmartDashboard.putNumber("right pos", frontRight.getSelectedSensorPosition());
        SmartDashboard.putString("loc", locEst.estimateLocation().toString());
    }

    protected void initDefaultCommand()
    {
        setDefaultCommand(new DriveCommand());
    }
}
