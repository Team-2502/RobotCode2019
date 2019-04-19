package com.team2502.robot2019.subsystem.solenoid;
import com.team2502.robot2019.RobotMap;
import com.team2502.robot2019.subsystem.interfaces.HatchFlap;
import com.team2502.robot2019.utils.NonDefaultSubsystem;
import edu.wpi.first.wpilibj.Solenoid;

public class HatchIntakeSolenoid extends NonDefaultSubsystem implements HatchFlap
{

    private final Solenoid hatchPusher;
    private boolean hatchPusherEnabled = false;

    public HatchIntakeSolenoid() {
        hatchPusher = new Solenoid(RobotMap.Solenoid.HATCH_INTAKE);

        // Starting State
        hatchPusher.set(false);
    }

    public void toggleHatchIntake()
    {
        hatchPusher.set(hatchPusherEnabled = !hatchPusherEnabled);
    }

    public void set(boolean open)
    {
        hatchPusher.set(hatchPusherEnabled = open);
    }
}
