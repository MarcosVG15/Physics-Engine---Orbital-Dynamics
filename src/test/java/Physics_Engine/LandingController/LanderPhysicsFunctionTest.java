package Physics_Engine.LandingController;

import Physics_Engine.GeneralComponents.AstralObject;
import Physics_Engine.GeneralComponents.Interfaces.SpaceObject;
import Physics_Engine.GeneralComponents.Interfaces.vectorInterface;
import Physics_Engine.GeneralComponents.Vector;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

public class LanderPhysicsFunctionTest {

    @Test
    public void testComputeDerivative(){
        LanderController controller = new LanderController();
        WindModel model = new WindModel(1000);
        LanderPhysicsFunction f = new LanderPhysicsFunction(controller, model);
        ArrayList<SpaceObject> solarSystem = new ArrayList<SpaceObject>();
        vectorInterface v = new Vector(10, -5, 0);
        solarSystem.add(new LanderObject(new Vector(0,0, 0), v, 0, "test", 0, 0));
        solarSystem.add(new AstralObject(new Vector(0, 0, 0), (Vector) v, 0));

        Exception e = assertThrows(
                IllegalArgumentException.class,
                () -> {
                    f.computeDerivative(1, new Vector(0, 0, 0), solarSystem);
                }
        );

        vectorInterface result = f.computeDerivative(0, new Vector(0, 0, 0), solarSystem);

        double dvx = 0; // sin(0) = 0
        double dvy = controller.calculateControlInputs(
                new LanderState(0, 0, 10, -5, 0.0, 0.0)
        ).thrust - 1.352;

        assertEquals(dvx, result.getX(), 0);
        assertEquals(dvy, result.getY(), 0);
        assertEquals(0, result.getZ(), 0);
    }
}
