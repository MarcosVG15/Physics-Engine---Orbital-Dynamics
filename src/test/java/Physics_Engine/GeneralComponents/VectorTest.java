package Physics_Engine.GeneralComponents;

import Physics_Engine.GeneralComponents.Interfaces.vectorInterface;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;

import static org.junit.jupiter.api.Assertions.*;

public class VectorTest {
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
    public void testComputeNorm(){
        Vector v1 = new Vector(0, 0, 0);
        Vector v2 = new Vector(0, 0, 1);
        Vector v3 = new Vector(0, 1, 0);
        Vector v4 = new Vector(1, 0, 0);
        Vector v5 = new Vector(3, 2, 1);

        assertEquals(0, v1.computeNorm());
        assertEquals(1, v2.computeNorm());
        assertEquals(1, v3.computeNorm());
        assertEquals(1, v4.computeNorm());
        assertEquals(3.741657, v5.computeNorm(), 1e-6);
    }

    @Test
    public void testPrint(){
        Vector v = new Vector(0, 0 , 0);
        v.print("test");

        String expectedOutput = "test" + "0.000000,0.000000,0.000000 " + System.lineSeparator();

        assertEquals(expectedOutput, outputStream.toString());
    }

    @Test
    public void testDistance(){
        vectorInterface v1 = new Vector(0, 0, 0);
        vectorInterface v2 = new Vector(0, 1, 0);
        vectorInterface v3 = new Vector(1003, 1, 54);
        vectorInterface v = new Vector(0, 0, 0);

        assertEquals(0, v.distance(v1));
        assertEquals(1, v.distance(v2));
        assertEquals(1004.453085, v.distance(v3), 1e-6);
    }

    @Test
    public void testAdd(){
        vectorInterface v = new Vector(0, 0, 0);
        vectorInterface v1 = new Vector(1, 1, 1);
        v.add(v1);

        assertEquals(1, v.getX());
        assertEquals(1, v.getY());
        assertEquals(1, v.getZ());

        vectorInterface v2 = new Vector(-9, 5, 0);
        v.add(v2);

        assertEquals(-8, v.getX());
        assertEquals(6, v.getY());
        assertEquals(1, v.getZ());
    }

    @Test
    public void testScale(){
        vectorInterface v = new Vector(10, 10, 10);
        v.scale(2);

        assertEquals(20, v.getX());
        assertEquals(20, v.getY());
        assertEquals(20, v.getZ());

        v.scale(0.25);

        assertEquals(5, v.getX());
        assertEquals(5, v.getY());
        assertEquals(5, v.getZ());
    }
}
