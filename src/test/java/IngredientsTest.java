import com.github.ezauton.core.action.BackgroundAction;
import com.github.ezauton.core.action.TimedPeriodicAction;
import com.github.ezauton.core.simulation.TimeWarpedSimulation;
import com.github.ezauton.core.trajectory.geometry.ImmutableVector;
import com.team2502.robot2019.command.autonomous.ingredients.VoltageDriveAction;
import com.team2502.robot2019.subsystem.sim.SimulatedDrivetrain;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import java.util.concurrent.TimeUnit;

public class IngredientsTest
{
    private SimulatedDrivetrain driveTrain;
    private TimeWarpedSimulation sim;

    @Before
    public void init() {
        sim = new TimeWarpedSimulation();
        driveTrain = new SimulatedDrivetrain(sim.getClock());
    }

    @Test
    public void testVoltageDrive() {
        sim.add(new VoltageDriveAction(1, 1, 1, TimeUnit.SECONDS, true, sim.getClock(), driveTrain));
        sim.add(new BackgroundAction(10, TimeUnit.MILLISECONDS, driveTrain::update));
        sim.runSimulation(10, TimeUnit.SECONDS);

        ImmutableVector lastLocation = driveTrain.getLocEstimator().estimateLocation();
        System.out.println("driveTrain.getLocEstimator().estimateLocation() = " + lastLocation);
        Assert.assertNotEquals("The simulated robot did not move", new ImmutableVector(0, 0), lastLocation);
    }
}
