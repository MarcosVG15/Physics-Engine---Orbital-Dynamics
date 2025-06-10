package Physics_Engine.GeneralComponents;

import Physics_Engine.GeneralComponents.Interfaces.SpaceObject;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class SpaceShipTest {
    private Vector v = new Vector(0, 0, 0);
    private Vector x = new Vector(0, 0, 0);
    private SpaceShip object = new SpaceShip(v, x);

    @Test
    public void testClone(){
        SpaceShip newObject = object.clone();

        assertEquals(object.getMass(), newObject.getMass(), 0);
        assertEquals(object.getName(), newObject.getName());
        assertEquals(object.getPositionVector().getX(), newObject.getPositionVector().getX(), 0);
        assertEquals(object.getPositionVector().getY(), newObject.getPositionVector().getY(), 0);
        assertEquals(object.getPositionVector().getZ(), newObject.getPositionVector().getZ(), 0);
        assertEquals(object.getVelocityVector().getX(), newObject.getVelocityVector().getX(), 0);
        assertEquals(object.getVelocityVector().getY(), newObject.getVelocityVector().getY(), 0);
        assertEquals(object.getVelocityVector().getZ(), newObject.getVelocityVector().getZ(), 0);

        object.setFuel(1234);

        assertEquals(object.getFuel(), 1234);
        assertNotEquals(object.getFuel(), newObject.getFuel());
    }

}
