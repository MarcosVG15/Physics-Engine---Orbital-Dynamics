package Physics_Engine.GeneralComponents;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class SolarSystemTest {
    private SolarSystem system = new SolarSystem();

    @Test
    public void testGetModulus(){
        Vector a = new Vector(1, 2, 3);
        Vector b = new Vector(1, 2, 3);
        double result1 = system.getModulus(a, b);

        Vector c = new Vector(0, 0, 0);
        Vector d = new Vector(3, 4, 0);
        double result2 = system.getModulus(c, d);

        Vector e = new Vector(-1, -2, -3);
        Vector f = new Vector(1, 2, 3);
        double result3 = system.getModulus(e, f);

        Vector g = new Vector(0.1, 0.2, 0.3);
        Vector h = new Vector(0.4, 0.5, 0.6);
        double result4 = system.getModulus(g, h);

        assertEquals(0.0, result1, 0);
        assertEquals(5.0, result2, 0);
        assertEquals(Math.sqrt(4 + 16 + 36), result3, 0);
        assertEquals(Math.sqrt(Math.pow(0.3, 2) + Math.pow(0.3, 2) + Math.pow(0.3, 2)), result4, 1e-9);
    }

    @Test
    public void testGetBestInitialPositionCoordinates(){

        double[] earth1 = {0.0, 0.0, 0.0};
        double[] titan1 = {10.0, 0.0, 0.0};
        double[] result1 = system.getBestInitialPositionCoordinates(titan1, earth1);

        double[] earth2 = {100, 200, 300};
        double[] titan2 = {400, 600, 800};
        double[] result2 = system.getBestInitialPositionCoordinates(titan2, earth2);

        double dx = 300, dy = 400, dz = 500;
        double norm = Math.sqrt(dx*dx + dy*dy + dz*dz);
        double[] expected2 = {
                100 + dx / norm * 6370,
                200 + dy / norm * 6370,
                300 + dz / norm * 6370
        };

        assertEquals(6370, result1[0], 0);
        assertEquals(0.0, result1[1], 0);
        assertEquals(0.0, result1[2], 0);

        assertEquals(expected2[0], result2[0], 0);
        assertEquals(expected2[1], result2[1], 0);
        assertEquals(expected2[2], result2[2], 0);
    }
}
