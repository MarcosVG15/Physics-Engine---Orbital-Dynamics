package Physics_Engine.LandingController;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class LanderStateTest {

    @Test
    public void testClone(){
        LanderState state = new LanderState(1, 2, 3, 4.6, -345, -50200.12848);
        LanderState newState = state.clone();

        assertEquals(state.getOrientation(), newState.getOrientation());
        assertEquals(state.getVelocityVector().getX(), newState.getVelocityVector().getX());
        assertEquals(state.getVelocityVector().getY(), newState.getVelocityVector().getY());
        assertEquals(state.getVelocityVector().getZ(), newState.getVelocityVector().getZ());
        assertEquals(state.getPositionVector().getX(), newState.getPositionVector().getX());
        assertEquals(state.getPositionVector().getY(), newState.getPositionVector().getY());
        assertEquals(state.getPositionVector().getZ(), newState.getPositionVector().getZ());
        assertEquals(state.getAngularVelocity(), newState.getAngularVelocity());

    }


}
