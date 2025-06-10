package Physics_Engine.LandingController;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class LanderControllerTest {
    private LanderController controller = new LanderController();

    @Test
    public void testCalculateControlInputs(){
        LanderState state1 = new LanderState(
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0
        );

        ControlInputs result1 = controller.calculateControlInputs(state1);
        double expectedThrust = 1.352;
        double expectedTorque = 0.0;

        assertEquals(expectedThrust, result1.thrust);
        assertEquals(expectedTorque, result1.torque);

        LanderState state2 = new LanderState(
                0.0,
                100.0,
                0.0,
                -100.0,
                0.0,
                0.0
        );

        ControlInputs result2 = controller.calculateControlInputs(state2);

        assertTrue(result2.thrust > 1.352);
        assertEquals(0.0, result2.torque, 1e-6);

        LanderState state3 = new LanderState(
                10.0,
                0.0,
                5.0,
                0.0,
                0.0,
                0.0
        );

        // Act
        ControlInputs result3 = controller.calculateControlInputs(state3);

        // Assert
        assertTrue(Math.abs(result3.torque) > 0.0);
        assertTrue(result3.thrust >= 0.0);
    }


    @Test
    public void testClamp(){
        double max = 1000;
        double min = -1000;
        double clamp1 = 0;
        double clamp2 = 1001;
        double clamp3 = -2000;

        assertEquals(0, controller.clamp(clamp1, min, max));
        assertEquals(1000, controller.clamp(clamp2, min, max));
        assertEquals(-1000, controller.clamp(clamp3, min, max));
    }

    @Test
    public void testNormalizeAngle(){
        assertEquals(-2.5663706, controller.normalizeAngle(10), 1e-6);
    }
}
