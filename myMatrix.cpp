//#include "../include/myMatrix.hpp"
#include "myMatrix.hpp"

//using namespace std;

myMatrix::myMatrix(int inNbRow,int inNbColumn){
    int i;
    this->nbRow=inNbRow;
    this->nbColumn=inNbColumn;
    //initialise data

    //memory
    this->data= new float*[this->nbRow];
    for(i = 0; i < this->nbRow; ++i)
        this->data[i] = new float[this->nbColumn];

    //fill with 0
    this->fillWithZeroes();
}

//not working somehow
myMatrix::~myMatrix()
{
    //for (int i = 0; i < this->nbRow; i++) {
    //    delete [] (this->data[i]);
    //}
    //delete [] (this->data);
}

void myMatrix::fillWithZeroes(){
    for(int i = 0; i < this->nbRow; ++i) {
        for(int j = 0; j < this->nbColumn; ++j) {
            this->data[i][j] = 0;
        }
    }
}

void myMatrix::set(int i,int j, float value){
    this->data[i][j]=value;
}

float myMatrix::get(int i,int j){
    return this->data[i][j];
}

void myMatrix::fillByCopy(myMatrix mat1){
    for(int i = 0; i < mat1.nbRow; ++i) {
        for(int j = 0; j < mat1.nbColumn; ++j) {
            this->data[i][j] = mat1.data[i][j];
        }
    }
}

//assume the dimensions are correct
void myMatrix::fillWithTranspose(myMatrix mat1){
    for(int i = 0; i < mat1.nbRow; ++i) {
        for(int j = 0; j < mat1.nbColumn; ++j) {
            this->data[j][i] = mat1.data[i][j];
        }
    }
}

//assume the dimensions are correct, and that this is fill with 0
void myMatrix::fillByMultiplication(myMatrix mat1,myMatrix mat2){
    // Multiplying matrix firstMatrix and secondMatrix and storing in array mult.
    float sum=0;
    for(int i = 0; i < mat1.nbRow; ++i) {
        for(int j = 0; j < mat2.nbColumn; ++j) {
            for(int k=0; k < mat1.nbColumn; ++k) {
                sum += mat1.data[i][k] * mat2.data[k][j];
            }
            this->data[i][j] = sum;
            sum = 0;
        }
    }
}

void myMatrix::addition(myMatrix mat1){
    for(int i = 0; i < mat1.nbRow; ++i) {
        for(int j = 0; j < mat1.nbColumn; ++j) {
            this->data[i][j] += mat1.data[i][j];
        }
    }
}

void myMatrix::subtraction(myMatrix mat1){
    for(int i = 0; i < mat1.nbRow; ++i) {
        for(int j = 0; j < mat1.nbColumn; ++j) {
            this->data[i][j] -= mat1.data[i][j];
        }
    }
}

/*
void myMatrix::print(){
    for(int i = 0; i < this->nbRow; ++i) {
        for(int j = 0; j < this->nbColumn; ++j) {
            cout << this->data[i][j] << " ";
        }
        cout << endl;
    }
}
*/

//The rest is for inversion

// calculate the cofactor of element (row,col)
int GetMinor(float **src, float **dest, int row, int col, int order)
{
    // indicate which col and row is being copied to dest
    int colCount=0,rowCount=0;

    for(int i = 0; i < order; i++ )
    {
        if( i != row )
        {
            colCount = 0;
            for(int j = 0; j < order; j++ )
            {
                // when j is not the element
                if( j != col )
                {
                    dest[rowCount][colCount] = src[i][j];
                    colCount++;
                }
            }
            rowCount++;
        }
    }

    return 1;
}

// Calculate the determinant recursively.
double CalcDeterminant( float **mat, int order)
{
    // order must be >= 0
    // stop the recursion when matrix is a single element
    if( order == 1 )
        return mat[0][0];

    // the determinant value
    float det = 0;

    // allocate the cofactor matrix
    float **minor;
    minor = new float*[order-1];
    for(int i=0;i<order-1;i++)
        minor[i] = new float[order-1];

    for(int i = 0; i < order; i++ )
    {
        // get minor of element (0,i)
        GetMinor( mat, minor, 0, i , order);
        // the recusion is here!

        det += (i%2==1?-1.0:1.0) * mat[0][i] * CalcDeterminant(minor,order-1);
        //det += pow( -1.0, i ) * mat[0][i] * CalcDeterminant( minor,order-1 );
    }

    // release memory
    for(int i=0;i<order-1;i++)
        delete [] minor[i];
    delete [] minor;

    return det;
}

// matrix inversion
// the result is put in Y
void MatrixInversion(float **A, int order, float **Y)
{
    // get the determinant of a
    double det = 1.0/CalcDeterminant(A,order);

    // memory allocation
    float *temp = new float[(order-1)*(order-1)];
    float **minor = new float*[order-1];
    for(int i=0;i<order-1;i++)
        minor[i] = temp+(i*(order-1));

    for(int j=0;j<order;j++)
    {
        for(int i=0;i<order;i++)
        {
            // get the co-factor (matrix) of A(j,i)
            GetMinor(A,minor,j,i,order);
            Y[i][j] = det*CalcDeterminant(minor,order-1);
            if( (i+j)%2 == 1)
                Y[i][j] = -Y[i][j];
        }
    }

    // release memory
    //delete [] minor[0];
    delete [] temp;
    delete [] minor;
}


//assume the matrix is invertible and square
void myMatrix::fillWithInverse(myMatrix mat1){
    MatrixInversion(mat1.data,mat1.nbColumn,this->data);
}

/*
//assume the dimensions are correct
void myMatrix::fillWithArray(float** array){
    for(int i = 0; i < this->nbRow; ++i) {
        for(int j = 0; j < this->nbColumn; ++j) {
            this->data[j][i] = array[i][j];
        }
    }
}
*/
