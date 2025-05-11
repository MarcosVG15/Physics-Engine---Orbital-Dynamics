package src.Physics_Engine.SOLAR_SYSTEM_MARCOS;

import org.w3c.dom.ls.LSOutput;

import java.util.ArrayList;

public class Legendre_Polynomial {
    private ArrayList<Double> P_polynomial ;
    private ArrayList<Double> P_polynomialDerivative;
    private ArrayList<ArrayList<Double>> E_Coefficient;

    private double N_quad ;
    private static final double TOLERANCE = 1E-5  ;

    public Legendre_Polynomial(double M ){

        P_polynomial = new ArrayList<>();
        P_polynomialDerivative = new ArrayList<>();
        E_Coefficient  = new ArrayList<>();

        this.N_quad= M+1 ;





    }

    public ArrayList<ArrayList<Double>> getLegendrePolynomial(){
        int n = 1 ;
        int l = 0 ;

        for ( int k = 1 ; k<=N_quad  ;k++){
            double error = 10;


            P_polynomial.clear();
            P_polynomialDerivative.clear();

            ArrayList<Double> firstRow = new ArrayList<>();
            double angle = ((4.0 * k - 1.0) * Math.PI) / (4.0 * N_quad + 2.0);
            double E0    = Math.cos(angle);
            firstRow.add(E0);


            E_Coefficient.add(firstRow);

            P_polynomial.add((double)1);
            P_polynomial.add(E_Coefficient.get(k-1).getFirst()) ;

            n = 1 ;
            l = 0 ;

            do{
                P_polynomial.add(P_nPlus1(n , E_Coefficient.get(k-1).get(l)));
                P_polynomialDerivative.add(PD_nPlus1(n , E_Coefficient.get(k-1).get(l))) ;

                double division = P_polynomial.getLast()/P_polynomialDerivative.getLast() ;
                double E_lPlus1 = E_Coefficient.get(k-1).get(l) - division ;
                System.out.println(division);
                E_lPlus1 = Math.max(-1.0, Math.min(1.0, E_lPlus1));


                E_Coefficient.get(k-1).add(E_lPlus1);

                error= Math.abs((E_Coefficient.get(k-1).getLast() - E_Coefficient.get(k-1).get(E_Coefficient.get(k-1).size()-2)));

                l++;
                n++;
            }
            while(error >=TOLERANCE) ;

        }
        return E_Coefficient ;


    }


    /**
     *  Calculates P_(n+1) for the legendre polynomial
     *  require the current value and the Gauss Legendre Quadrature Node
     * @param n - current P
     * @param E - gauss legendre qudrature Node
     * @return - return P_n+1
     */
    private double P_nPlus1 (int n , double E){
        double num = (2*n+1)*E*P_polynomial.get(n)
                - n*P_polynomial.get(n-1);

        System.out.print("Pn  "+num / (n+1) +" | ");
        return num / (n+1);

    }

    /**
     *  Calculates the derivative of  P_(n+1) the legendre polynomial
     * @param n - current P
     * @param E - gauss legendre quadrature Node
     * @return - return P_n+1
     */
    private double PD_nPlus1 (int n , double E){
         double o = ((n+1)/(Math.pow(E,2)-1)); // first term to calculate the derivative of the Legendre Polynomial
         double DifferenceInPolynomial = ((E*P_polynomial.get(n+1) - P_polynomial.get(n)));

         System.out.println("PDn    "+o*DifferenceInPolynomial);

        return o*DifferenceInPolynomial;
    }



}
