#ifndef MYMATRIX_HPP
#define MYMATRIX_HPP

//#include <iostream>
#include <stdlib.h>
class myMatrix{
    public:
        int nbRow;
        int nbColumn;
        float** data;

        myMatrix(int inNbRow,int inNbColumn);
        
        ~myMatrix();
        
        void set(int i,int j, float value);

        float get(int i,int j);

        void fillWithInverse(myMatrix mat1);

        //WIP
        //void fillWithArray(float** array);

        void fillWithZeroes();

        void fillByCopy(myMatrix mat1);

        void fillWithTranspose(myMatrix mat1);

        void fillByMultiplication(myMatrix mat1,myMatrix mat2);

        void addition(myMatrix mat1);

        void subtraction(myMatrix mat1);

        void print();

    protected:

    private:
};

#endif // MYMATRIX_HPP
