package src.Physics_Engine.SOLAR_SYSTEM_MARCOS;

import java.util.ArrayList;

public class Main {
    public static void  main(String[] args){

           Legendre_Polynomial legendrePolynomial = new Legendre_Polynomial(1);
           ArrayList<ArrayList<Double>> arr = legendrePolynomial.getLegendrePolynomial();

           for( ArrayList<Double> subArr : arr){
               System.out.println(subArr);
           }



    }
}
