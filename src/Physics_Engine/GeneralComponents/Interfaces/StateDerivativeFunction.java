package src.Physics_Engine.GeneralComponents.Interfaces;

/**
 * Interface for functions that compute the derivative of a state vector.
 * This is used in ODE solvers.
 */
public interface StateDerivativeFunction {

    /**
     * Computes the derivative of the state vector at a given time.
     *
     * @param state The current state vector as a double array.
     * @param time The current time.
     * @param params Optional parameters needed for the derivative computation.
     * @return The derivative of the state vector as a double array.
     */
    double[] computeDerivative(double[] state, double time, double[] params);
}