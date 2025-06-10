package Physics_Engine.LandingController;

import Physics_Engine.GeneralComponents.AstralObject;
import Physics_Engine.GeneralComponents.Interfaces.SpaceObject;
import Physics_Engine.GeneralComponents.Vector;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;

import static org.junit.jupiter.api.Assertions.*;

public class LanderObjectTest {
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

    @Test
    public void testClone(){
        SpaceObject lander  = new LanderObject(new Vector(1, 2, 3), new Vector(5, 6, 7), 0, "test", 34, -9);
        SpaceObject newLander = lander.clone();
        assertNull(newLander);
    }

    @Test
    public void testHasHitPlanet(){
        SpaceObject astralObject = new AstralObject(new Vector(0, 0, 0), new Vector(0, 0, 0), 0);
        SpaceObject lander  = new LanderObject(new Vector(1, 2, 3), new Vector(5, 6, 7), 0, "test", 34, -9);

        assertFalse(lander.hasHitPlanet(astralObject, 0.05));
        assertTrue(lander.hasHitPlanet(astralObject, 1000));
    }

    @Test
    public void testPrint(){
        LanderObject lander  = new LanderObject(new Vector(1, 2, 3), new Vector(5, 6, 7), 0, "test", 34, -9);
        lander.print();

        String expectedOutput = "Lander: " + lander.getName() + "\n" + "  Position: " + lander.getPositionVector().getX() + ", "
                + lander.getPositionVector().getY() + ", " + lander.getPositionVector().getZ() + "\n" + "  Velocity: "
                + lander.getVelocityVector().getX() + ", " + lander.getVelocityVector().getY() + ", " + lander.getVelocityVector().getZ() + "\n"
                + "  Mass: " + lander.getMass() + "\n" + "  Orientation (radians): " + lander.getOrientation() + "\n"
                + "  Angular Velocity (radians/s): " + lander.getAngularVelocity() + "\n";

        assertEquals(expectedOutput, outputStream.toString());
    }
}
