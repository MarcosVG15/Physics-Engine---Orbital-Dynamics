package src.Physics_Engine.SolarSystem;


import java.util.ArrayList;
import java.util.List;


public class SolarSystem {
    private List<AstralObject> solarSystem;
    public SolarSystem(){
        solarSystem = new ArrayList<>(
                List.of(new AstralObject(0,0,0,0,0,0,1.99E30, "Sun"),//Sun
                        new AstralObject(-5.67E7, -3.23E+07,2.58E+06, 1.39E+01,-4.03E+01 ,-4.57E+00,3.30E+23, "Mercury"),//Mercury
                        new AstralObject(-1.04E+08,-3.19E+07,5.55E+06,9.89E+00,-3.37E+01,-1.03E+00,4.87E+24, "Venus"),//Venus
                        new AstralObject(-1.47E+08,-2.97E+07,2.75E+04,5.31E+00,-2.93E+01,6.69E-04,5.97E+24, "Earth"),//Earth
                        new AstralObject(-1.47E+08,-2.95E+07,5.29E+04,4.53E+00,-2.86E+01,6.73E-02,7.35E+22, "Moon"),//Moon
                        new AstralObject(-2.15E+08,1.27E+08,7.94E+06,-1.15E+01,-1.87E+01,-1.11E-01,6.42E+23, "Mars"),//Mars
                        new AstralObject(5.54E+07,7.62E+08,-4.40E+06,-1.32E+01,1.29E+01,5.22E-02,1.90E+27, "Jupiter"),//Jupiter
                        new AstralObject(1.42E+09,-1.91E+08,-5.33E+07,7.48E-01,9.55E+00,-1.96E-01,5.68E+26, "Saturn"),//Saturn
                        new AstralObject(1.42E+09,-1.92E+08,-5.28E+07,5.95E+00,7.68E+00,2.54E-01,1.35E+23, "Titan"),//Titan
                        new AstralObject(1.62E+09,2.43E+09,-1.19E+07,-5.72E+00,3.45E+00,8.70E-02,8.68E+25, "Uranus"),//Uranus
                        new AstralObject(4.47E+09,-5.31E+07,-1.02E+08,2.87E-02,5.47E+00,-1.13E-01,1.02E+26, "Neptune")//Neptune
                )
        );

    }

    public List<AstralObject> getSolar(){
        return solarSystem;
    }

   public void setSolar(List<AstralObject> solarSystem){
        this.solarSystem = solarSystem;
   }
}
