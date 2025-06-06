package src.test.java.Physics_Engine.GeneralComponents;

import org.junit.Test;
import src.main.java.Physics_Engine.GeneralComponents.AstralObject;
import src.main.java.Physics_Engine.GeneralComponents.Interfaces.SpaceObject;
import src.main.java.Physics_Engine.GeneralComponents.Vector;

import static org.junit.Assert.assertEquals;

public class AstralObjectTest {

    @Test
    public void testGetMass(){
        Vector v = new Vector(0, 0, 0);
        Vector x = new Vector(0, 0, 0);
        double m = 5;
        SpaceObject object = new AstralObject(v, x, m);

        assertEquals(m, object.getMass(), 0);
    }

}
