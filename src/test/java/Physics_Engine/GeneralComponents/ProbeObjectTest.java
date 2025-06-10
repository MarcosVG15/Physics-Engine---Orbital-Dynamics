package Physics_Engine.GeneralComponents;

import Physics_Engine.GeneralComponents.Interfaces.SpaceObject;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;

import static org.junit.jupiter.api.Assertions.*;


public class ProbeObjectTest {


        private final ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
        private final PrintStream originalOut = System.out;

        @BeforeEach
        public void setUp() {
            // Redirect System.out to a ByteArrayOutputStream
            System.setOut(new PrintStream(outputStream));
        }

        @AfterEach
        public void restoreSystemOut() {
            // Restore the original System.out after each test
            System.setOut(originalOut);
        }



        private Vector v = new Vector(0, 0, 0);
        private Vector x = new Vector(0, 0, 0);
        private SpaceObject object = new ProbeObject(v, x);

    @Test
    public void testPrint(){
        object.print();

        String expectedOutput = object.getName() + ", Position , 0.000000,0.000000,0.000000 \n" + object.getName() + "  Velocity , 0.000000,0.000000,0.000000 " + System.lineSeparator();

        assertEquals(expectedOutput, outputStream.toString());
    }

    @Test
    public void testClone(){
        SpaceObject newObject = object.clone();

        assertEquals(object.getMass(), newObject.getMass(), 0);
        assertEquals(object.getName(), newObject.getName());
        assertEquals(object.getPositionVector().getX(), newObject.getPositionVector().getX(), 0);
        assertEquals(object.getPositionVector().getY(), newObject.getPositionVector().getY(), 0);
        assertEquals(object.getPositionVector().getZ(), newObject.getPositionVector().getZ(), 0);
        assertEquals(object.getVelocityVector().getX(), newObject.getVelocityVector().getX(), 0);
        assertEquals(object.getVelocityVector().getY(), newObject.getVelocityVector().getY(), 0);
        assertEquals(object.getVelocityVector().getZ(), newObject.getVelocityVector().getZ(), 0);
    }

    @Test
    public void testHasHitPlanet(){
        double radius = 100;
        Vector xx = new Vector(0, 12, 9090);
        Vector xxx = new Vector(1200, 34, 9090);
        Vector xxxx = new Vector(56879, 123339, 900);

        SpaceObject astralObject1 = new AstralObject(v, x, 100);
        SpaceObject astralObject2 = new AstralObject(v, xx, 100);
        SpaceObject astralObject3 = new AstralObject(v, xxx, 9);
        SpaceObject astralObject4 = new AstralObject(v, xxxx, 9);

        assertTrue(object.hasHitPlanet(astralObject1, radius));
        assertFalse(object.hasHitPlanet(astralObject2, radius));
        assertFalse(object.hasHitPlanet(astralObject3, radius));
        assertFalse(object.hasHitPlanet(astralObject4, radius));
    }

}
