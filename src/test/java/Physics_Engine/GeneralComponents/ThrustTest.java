package Physics_Engine.GeneralComponents;

import Physics_Engine.GeneralComponents.Interfaces.vectorInterface;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class ThrustTest {
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
    public void testPrint(){
        Thrust thrust = new Thrust(0, 1, 2);
        thrust.setDuration(23);
        thrust.setStartTime(9);

        double[] thrustComponents = thrust.getThrustVector();

        thrust.print();

        String expectedOutput = " THRUST COMPONENTS : "+ thrustComponents[0] + " , " + thrustComponents[1]
                + " , " + thrustComponents[2] + " Starting : "+ thrustComponents[3] + " Duration : " + thrustComponents[4] + System.lineSeparator();

        assertEquals(expectedOutput, outputStream.toString());
    }
}
