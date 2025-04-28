package src.Physics_Engine.SolarSystem;

import src.Physics_Engine.ODEFunction;

import java.util.Arrays;

public class Adams_Bashforth_Solver {

    public Adams_Bashforth_Solver(){

    }

    // the thing is that X has Three value for each dimensions and three of each for each t . DOes this take into account ??
    public double[] AB4(double h, double[] t, double[] x, ODEFunction function){
        double[][] history = new double[t.length][x.length];

        //Bootstraping first using classical fourth order Runge-Kuntta method
        for (int i = 0; i < 3; i++) {
            double[] k1 = function.computeDerivative(x, t[i]);
            double[] xTemp = new double[x.length];
            // k2 = f(x + h/2 * k1, t + h/2)
            for (int j = 0; j < x.length; j++) {
                xTemp[j] = x[j] + h / 2.0 * k1[j];
            }
            double[] k2 = function.computeDerivative(xTemp, t[i] + h / 2);
            // k3 = f(x + h/2 * k2, t + h/2)
            for (int j = 0; j < x.length; j++) {
                xTemp[j] = x[j] + h / 2.0 * k2[j];
            }
            double[] k3 = function.computeDerivative(xTemp, t[i] + h / 2);
            // k4 = f(x + h * k3, t + h)
            for (int j = 0; j < x.length; j++) {
                xTemp[j] = x[j] + h * k3[j];
            }
            double[] k4 = function.computeDerivative(xTemp, t[i] + h);
            // Update the state using the RK4 formula
            for (int j = 0; j < x.length; j++) {
                x[j] = x[j] + h / 6.0 * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]);
                //Store x values after each time step
                history[i][j] = x[j];
            }
            System.out.println("Step : " + i + " | t = " + t[i] + " | x = " + Arrays.toString(x));
        }

        //Using AB4 method onward when we have previous 4 derivatives
        for(int k = 4; k < t.length; k++){
            for (int j = 0; j < x.length; j++) {
                //x_j = x_j-1 + h/24 * (55 * f(x_j-1, t_j-1) - 59 * f(x_j-2, t_j-2) + 37 * f(x_j-3, t_j-3- 9 * f(x_j-4, t_j-4))
                x[j] = x[j] + h / 24.0 * (
                        55 * function.computeDerivative(history[k - 1], t[k - 1])[j]
                                - 59 * function.computeDerivative(history[k - 2], t[k - 2])[j]
                                + 37 * function.computeDerivative(history[k - 3], t[k - 3])[j]
                                -  9 * function.computeDerivative(history[k - 4], t[k - 4])[j]
                );

                history[k][j] = x[j];
            }
        }


        //Return the final stage, approximated value x at final time step
        return x;
    }

}
