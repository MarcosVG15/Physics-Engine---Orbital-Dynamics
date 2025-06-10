package Physics_Engine.ODESolverRK4;

import Physics_Engine.GeneralComponents.Interfaces.*;
import Physics_Engine.GeneralComponents.SolarSystem;
import Physics_Engine.GeneralComponents.Vector;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class VelocityFunctionTest {

    @Test
    public void testComputeDerivative(){
        vectorInterface v = new Vector(0, 0, 0);
        function f = new VelocityFunction();
        SolarSystem system = new SolarSystem();
        vectorInterface result = f.computeDerivative(0, v, system.getSolarSystem());
        assertEquals(v, result);
    }
}
