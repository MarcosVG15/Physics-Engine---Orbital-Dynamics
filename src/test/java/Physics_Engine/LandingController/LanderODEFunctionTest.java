package Physics_Engine.LandingController;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class LanderODEFunctionTest {

    @Test
    public void testComputeDerivative(){
        double time = 1000;
        double[] state = {100, 200, 5, -3, 0.0, 0.1};

        LanderController controller = new LanderController();
        WindModel model = new WindModel(1000);
        LanderODEFunction f = new LanderODEFunction(controller, model);

        double[] result = f.computeDerivative(state, time, null);

        double x = model.getWindX(time);
        double y = model.getWindY(time);
        ControlInputs inputs = controller.calculateControlInputs(new LanderState(
                state[0], state[1], state[2], state[3], state[4], state[5]
        ));

        double dvxdt = inputs.thrust * Math.sin(state[4]) + x;
        double dvydt = inputs.thrust * Math.cos(state[4]) - 1.352 + y;
        double dthetadt = state[5];
        double domegadt = inputs.torque;

        assertEquals(5.0, result[0], 1e-6);
        assertEquals(-3.0, result[1], 1e-6);
        assertEquals(dvxdt, result[2], 2000);
        assertEquals(dvydt, result[3], 2000);
        assertEquals(dthetadt, result[4], 1e-6);
        assertEquals(domegadt, result[5], 1e-6);
    }
}
