package src.Physics_Engine.SpectralDefferedInProgress;

public class QIJMatrix {

    private double[][] QIJMatrix ;
    private double StartT;
    private double EndT;

    private static int NUMBER_OF_INTERVALS = 10 ; // will change if it takes too much time to calculate ;

    public QIJMatrix( double StartT , double EndT){
        this.QIJMatrix = new double[NUMBER_OF_INTERVALS][NUMBER_OF_INTERVALS];
        this.StartT = StartT;
        this.EndT = EndT;



    }

    private double[][] ComputeMatrixCoefficients(){
        double h  =  ((EndT - StartT)/(double)NUMBER_OF_INTERVALS);
        double[] ArrayAllTs = getAllTs();

        for( int  i = 0 ; i<NUMBER_OF_INTERVALS ; i++){
            for( int  j = 0 ; j<NUMBER_OF_INTERVALS ; j++) {
                QIJMatrix[i][j] = SimpsonsIntegration(j,segmentArray(i , ArrayAllTs));
            }
        }
        return  QIJMatrix ;
    }

    /**
     * Shortens the array based  how many datapoints we want to pass
     * @param data - the amount of data point
     * @param Array - the Array that we want shorten
     * @return - returns a shortened array
     */
    private double[] segmentArray(int data, double[] Array){

        double[] shortenedArray = new double[data + 1];
        for (int i = 0; i <= data; i++) {
            shortenedArray[i] = Array[i];
        }
        return shortenedArray ;
    }

    private double SimpsonsIntegration(int j, double[] ArrayAllTs){
        double h  =  ((ArrayAllTs[ArrayAllTs.length-1] - ArrayAllTs[0])/(ArrayAllTs.length));

        double Integration = LagrangePolynomial(ArrayAllTs[0], ArrayAllTs[j], ArrayAllTs)
                                              + LagrangePolynomial(ArrayAllTs[ArrayAllTs.length-1] ,ArrayAllTs[j] , ArrayAllTs) ;

        for (int c = 1; c < ArrayAllTs.length; c++) {
            if (c % 2 != 0) {
                Integration += 4*(LagrangePolynomial(c, ArrayAllTs[j], ArrayAllTs));
            }
            else {
                Integration += 2*(LagrangePolynomial(c, ArrayAllTs[j], ArrayAllTs));

            }
        }
        return (h/3 * Integration);


    }

    /**
     * This method compute the langrang polynomial values such that i could integrate over these values for different S
     * it requires the S that we want to compute the polynomial for , the Tj which is the step J that is needed and an
     * array of T values which are essentially a subset of the main step interval.
     *
     * @param S - value we want to compute for
     * @param Tj - the current T value
     * @param AllTs - all T values
     * @return - returns the value of the Lagrange Polynomials for the current S
     */
    private double LagrangePolynomial(double S , double Tj , double[] AllTs){
        double Jth_LagrangeValue = 1;

        for (double allT : AllTs) {
            if(Tj == allT){
                continue;
            }
            Jth_LagrangeValue *= (S - allT) / (Tj - allT);
        }

        return Jth_LagrangeValue;
    }


    private double[] getAllTs(){

        double h = (EndT - StartT)/NUMBER_OF_INTERVALS ;
        double[] ArrayOfTs = new double[NUMBER_OF_INTERVALS];

        double Accumulation = StartT;
        ArrayOfTs[0] = Accumulation;

        for( int i = 1 ; i<NUMBER_OF_INTERVALS ; i++){
            Accumulation+= h;
            ArrayOfTs[i] = Accumulation;
        }

       return ArrayOfTs;
    }
}
