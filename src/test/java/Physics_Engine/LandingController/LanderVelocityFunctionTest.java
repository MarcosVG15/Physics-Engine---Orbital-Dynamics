package Physics_Engine.LandingController;

import Physics_Engine.GeneralComponents.AstralObject;
import Physics_Engine.GeneralComponents.Interfaces.SpaceObject;
import Physics_Engine.GeneralComponents.Interfaces.vectorInterface;
import Physics_Engine.GeneralComponents.Vector;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

public class LanderVelocityFunctionTest {

    @Test
    public void testComputeDerivative(){
        LanderVelocityFunction f = new LanderVelocityFunction();
        ArrayList<SpaceObject> solarSystem = new ArrayList<SpaceObject>();
        vectorInterface v = new Vector(12, 893, -23.5);
        solarSystem.add(new LanderObject(new Vector(0,0, 0), v, 0, "test", 0, 0));
        solarSystem.add(new AstralObject(new Vector(0, 0, 0), (Vector) v, 0));

        Exception e = assertThrows(
                IllegalArgumentException.class,
                () -> {
                    f.computeDerivative(1, new Vector(0, 0, 0), solarSystem);
                }
        );

        vectorInterface result = f.computeDerivative(0, new Vector(0, 0, 0), solarSystem);

        assertEquals(result.getX(), v.getX());
        assertEquals(result.getY(), v.getY());
        assertEquals(result.getZ(), v.getZ());

    }

}
