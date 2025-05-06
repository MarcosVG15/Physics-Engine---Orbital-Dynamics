package src.Physics_Engine.SolarSystem;

import src.Physics_Engine.ODEFunction;

import java.util.Arrays;

public class Adams_Bashforth_Solver {

    public Adams_Bashforth_Solver(){

    }

    // the thing is that X has Three value for each dimensions and three of each for each t . DOes this take into account ??
    public double[] AB4(double h, double t, double[] x, double[][] history, ODEFunction function){
        

        //Bootstraping first using classical fourth order Runge-Kuntta method
        if (Math.round(t/h) < 3) {
            double[] k1 = function.computeDerivative(x, t);
            double[] xTemp = new double[x.length];
            // k2 = f(x + h/2 * k1, t + h/2)
            for (int j = 0; j < x.length; j++) {
                xTemp[j] = x[j] + h / 2.0 * k1[j];
            }
            double[] k2 = function.computeDerivative(xTemp, t + h / 2);
            // k3 = f(x + h/2 * k2, t + h/2)
            for (int j = 0; j < x.length; j++) {
                xTemp[j] = x[j] + h / 2.0 * k2[j];
            }
            double[] k3 = function.computeDerivative(xTemp, t + h / 2);
            // k4 = f(x + h * k3, t + h)
            for (int j = 0; j < x.length; j++) {
                xTemp[j] = x[j] + h * k3[j];
            }
            double[] k4 = function.computeDerivative(xTemp, t + h);
            // Update the state using the RK4 formula
            for (int j = 0; j < x.length; j++) {
                x[j] = x[j] + h / 6.0 * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]);
                //Store x values after each time step
                history[(int)(t/h)][j] = x[j];
            }
            System.out.println("Step : " + Math.round(t/h) + " | t = " + t + " | x = " + Arrays.toString(x));
            return x;
        }

        //Using AB4 method onward when we have previous 4 derivatives
        else{
            for (int j = 0; j < x.length; j++) {
                //x_j = x_j-1 + h/24 * (55 * f(x_j-1, t_j-1) - 59 * f(x_j-2, t_j-2) + 37 * f(x_j-3, t_j-3- 9 * f(x_j-4, t_j-4))
                x[j] = x[j] + h / 24.0 * (
                        55 * function.computeDerivative(history[(int)Math.round(t/h - h)], t - h)[j]
                                - 59 * function.computeDerivative(history[(int)Math.round(t/h - 2*h)], t - 2*h)[j]
                                + 37 * function.computeDerivative(history[(int)Math.round(t/h - 3*h)], t - 3*h)[j]
                                -  9 * function.computeDerivative(history[(int)Math.round(t/h - 4*h)], t - 4*h)[j]
                );
                history[(int)Math.round(t/h)][j] = x[j];
            }
        }


        //Return the final stage, approximated value x at final time step
        return x;
    }

}
