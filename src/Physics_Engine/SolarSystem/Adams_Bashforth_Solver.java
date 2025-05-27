package src.Physics_Engine.SolarSystem;

import src.Physics_Engine.SolarSystem.ODEFunction;

import java.util.Arrays;

/**
 * DO NOT IMPLEMENT IT (FOR NOW)
 * SOLVER DOES UPDATE OBJECT(PLANET) WITHOUT WAITING FOR OTHER PLANTES IN SOLAR SYSTEM
 */
public class Adams_Bashforth_Solver {

    public Adams_Bashforth_Solver(){

    }

    // the thing is that X has Three value for each dimensions and three of each for each t . DOes this take into account ??
    public double[] AB4(double h, double t, double[] x, double[][] history, double[] v,  ODEFunction function){
        int step = (int) (t/h);

        //Bootstraping first using classical fourth order Runge-Kuntta method
        if (step < 3) {
            double[] k1 = function.computeDerivative(x, t, v);
            double[] xTemp = new double[x.length];
            // k2 = f(x + h/2 * k1, t + h/2)
            for (int j = 0; j < x.length; j++) {
                xTemp[j] = x[j] + h / 2.0 * k1[j];
            }
            double[] k2 = function.computeDerivative(xTemp, t + h / 2, v);
            // k3 = f(x + h/2 * k2, t + h/2)
            for (int j = 0; j < x.length; j++) {
                xTemp[j] = x[j] + h / 2.0 * k2[j];
            }
            double[] k3 = function.computeDerivative(xTemp, t + h / 2, v);
            // k4 = f(x + h * k3, t + h)
            for (int j = 0; j < x.length; j++) {
                xTemp[j] = x[j] + h * k3[j];
            }
            double[] k4 = function.computeDerivative(xTemp, t + h, v);
            // Update the state using the RK4 formula
            for (int j = 0; j < x.length; j++) {
                x[j] = x[j] + h / 6.0 * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]);
                //Store x values after each time step
                history[step][j] = x[j];
            }
            //System.out.println("Step : " + step + " | t = " + t + " | x = " + Arrays.toString(x));
            return x;
        }

        //Using AB4 method onward when we have previous 4 derivatives
        else{

            double[] f1 = function.computeDerivative(history[index(step - 1)], t - h, v);
            double[] f2 = function.computeDerivative(history[index(step - 2)], t - 2*h, v);
            double[] f3 = function.computeDerivative(history[index(step - 3)], t - 3*h, v);
            double[] f4 = function.computeDerivative(history[index(step - 4)], t - 4*h, v);

            for (int j = 0; j < x.length; j++) {
                //x_j = x_j-1 + h/24 * (55 * f(x_j-1, t_j-1) - 59 * f(x_j-2, t_j-2) + 37 * f(x_j-3, t_j-3- 9 * f(x_j-4, t_j-4))
                x[j] = x[j] + h / 24.0 * (55 * f1[j] - 59 * f2[j] + 37 * f3[j] -  9 * f4[j]);
                history[index(step)][j] = x[j];
            }
        }

        //Return the final stage, approximated value x at final time step
        return x;
    }

    private int index(int s){
        return ((s%4)+4)%4;
    }
}
