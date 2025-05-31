package src.Physics_Engine.SpectralDefferedInProgress;

public class QIJMatrix {

    private double StartT;
    private double EndT;

    private static int NUMBER_OF_INTERVALS = 11 ; // will change if it takes too much time to calculate ;

    public QIJMatrix( double StartT , double EndT){
        this.StartT = StartT;
        this.EndT = EndT;



    }


    public double[][] QijMatrix(int size){

        double[][] QIJMatrix = new double[size][size];

        for(int i = 1 ; i<10 ; i++){
            for(int j = 1 ; j<10 ; j++){
                if(j<=i){
                    QIJMatrix[i][j] = TrapezoidIntegration( getAllTs(), i , j);

                }

            }
        }
        return QIJMatrix;
    }


    /**
     * This method computes the Integral for a given interval of a specific lagrange Polynomial for a given range
     * @param AllTs - This is the array of all subsections of the interval t and t+1 which would be 1 second for our planet
     * @param I - is the current Row of the Qij Matrix that we want to compute
     * @param J - is the current Column of the Qij Matrix that we want to compute and corresponds to the lagrange Polynomial
     * @return - It should return the integral  over a given range of the specific Lagrange Polynomial.
     */
    public double TrapezoidIntegration(double[] AllTs , int I , int J ){

        double Integral = 0 ;
        double[] SegmentedArray = Segmenter(I ,  AllTs);
        double Tj = SegmentedArray[J];
        double h = (SegmentedArray[I]- SegmentedArray[0])/SegmentedArray.length;

        if(SegmentedArray.length < 2) {
            throw new IllegalArgumentException(" The Size Must be bigger than 2");
        }

        Integral+= LagrangePolynomial(SegmentedArray[0] ,Tj , SegmentedArray );

        Integral+= LagrangePolynomial(SegmentedArray[I] , Tj, SegmentedArray );


        for (int i = 1 ; i< I ; i++){
            Integral += 2 * (LagrangePolynomial(SegmentedArray[i] ,Tj ,SegmentedArray));
        }


        return (h/2.0) * Integral ;
    }

    public double[] Segmenter ( int I , double[] AllTs){

        double[] segmenetedArray = new double[I+1];

        for(int i = 0 ; i<I+1 ; i++){
            segmenetedArray[i] = AllTs[i];
        }
        return segmenetedArray;
    }



    /**
     * This method compute the lagrange polynomial values such that i could integrate over these values for different S
     * it requires the S that we want to compute the polynomial for , the Tj which is the step J that is needed and an
     * array of T values which are essentially a subset of the main step interval.
     *
     * @param S - value we want to compute for
     * @param Tj - the current T value
     * @param AllTs - all T values
     * @return - returns the value of the Lagrange Polynomials for the current S
     */
    public double LagrangePolynomial(double S , double Tj , double[] AllTs){
        double Jth_LagrangeValue = 1;

        for (double allT : AllTs) {
            if(Tj == allT){
                continue;
            }
            Jth_LagrangeValue *= ((S - allT)/(Tj - allT));
            System.out.println("Numerator : For Ti : "+ allT +" We get : "+ (S - allT));
            System.out.println("Denominator : For Ti : "+ allT +" We get : "+ (Tj - allT));
        }

        return Jth_LagrangeValue;
    }


    public double[] getAllTs(){

        double h = (EndT - StartT)/(NUMBER_OF_INTERVALS-1) ;
        double[] ArrayOfTs = new double[NUMBER_OF_INTERVALS];

        double Accumulation = StartT;
        ArrayOfTs[0] = Accumulation;

        for( int i = 1 ; i<NUMBER_OF_INTERVALS ; i++){
            Accumulation+= h;
            ArrayOfTs[i] = (double) Math.round(Accumulation * 10_000) /10_000;
        }

       return ArrayOfTs;
    }

}
