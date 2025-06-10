package Physics_Engine.ODESolverRK4;

import Physics_Engine.GeneralComponents.AstralObject;
import Physics_Engine.GeneralComponents.Interfaces.*;
import Physics_Engine.GeneralComponents.Vector;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class AccelerationFunctionTest {

    @Test
    public void testComputeDerivative(){
        function f = new AccelerationFunction();

        vectorInterface v1 = new Vector(1, 0, 0);
        vectorInterface v2 = new Vector(0, 0, 0);

        SpaceObject a = new AstralObject((Vector) v1, (Vector)v1, 1);
        SpaceObject b = new AstralObject((Vector)v2, (Vector)v2, 1);

        ArrayList<SpaceObject> system = new ArrayList<>();
        system.add(b);
        system.add(a);

        Vector result = (Vector) f.computeDerivative(1, v1, system);

        assertEquals(-6.6743e-20, result.getX());
        assertEquals(0, result.getY(), 1e-6);
        assertEquals(0, result.getZ(), 1e-6);


    }
}
