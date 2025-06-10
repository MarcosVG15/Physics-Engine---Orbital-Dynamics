package Physics_Engine.LandingController;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertTrue;

public class WindModelTest {

    @Test
    public void testGetWind(){
        double max = 1000;
        double time = 1000;

        WindModel model = new WindModel(max);

        int i = 0;
        while(i < 10000) {
            double x = model.getWindX(time);
            double y = model.getWindY(time);

            assertTrue(x >= -1000 & x <= 1000);
            assertTrue(y >= -1000 & y <= 1000);
            i+=1;
        }
    }
}
