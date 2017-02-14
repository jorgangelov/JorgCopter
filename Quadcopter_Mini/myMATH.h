#ifndef MYMATH_H
#define MYMATH_H
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

// Declarations
namespace jorg{
//////////////////////////////////////////////// MATRIX
template <uint8_t n, uint8_t m> class cMatrix
{
public:
    cMatrix()
    {
        memset(entries,0,sizeof(entries));
    }


    float& operator()(int i, int j);
    cMatrix<m,n> T();
    float norm();

protected:
    float entries[n][m];

};
//////////////////////////////////////////////// MATRIX


//////////////////////////////////////////////// QUATERNION
class cQuaternion
{
public:
    cQuaternion(float q1=1,float q2=0,float q3=0,float q4=0)
    {
        values[0] = q1;
        values[1] = q2;
        values[2] = q3;
        values[3] = q4;
    }

    cQuaternion operator+(cQuaternion Q);
    cQuaternion operator*(cQuaternion Q);
    cQuaternion operator*(float f);
    float& operator()(int i);
    cQuaternion conjugated();
    void norm();
    float values[4];
};


//////////////////////////////////////////////// QUATERNION


//////////////////////////////////////////////// FILTERS
class cDigitalFilter
{
  public:
  cDigitalFilter(float _T_f)
  {
    T_f = _T_f;
    upt1_k = 0;
    udt1_k = 0;
    uI_k = 0;
    u_k = 0;
  }
  void update(float _u_k, float _T_s);


  float upt1_k, udt1_k, uI_k, udot_kme;

  protected:
  float upt1_kme, udt1_kme, uI_kme, u_kme, u_k;
  float T_f;


};

template <class T> class cRingBuffer
{
public:
    cRingBuffer(int _length)
    {
        length = _length;
        currentIndex = 0;
        buffer = new T[length];
        memset(buffer,0,sizeof(buffer)*_length);
    }
    ~cRingBuffer(){delete buffer;}
    void attach(T value);
    T first();
    T last();
    T& operator[](int i);
    int size();
protected:
    T *buffer;
    int length;
    int currentIndex;
};

class cFIRFilter
{
public:
    cFIRFilter(int _order):values(_order)
    {
        order = _order;
        u_FIR_kme = 0;
    }

    void update(float u_k);
    float u_FIR_k;
protected:
    int order;
    float u_FIR_kme;
    cRingBuffer<float> values;

};
//////////////////////////////////////////////// FILTERS

class cOptimization
{
public:
cOptimization(double (*costFctPt_)(double* argArray), int argNr_)
{
    costFctPt = costFctPt_;
    argNr = argNr_;
    Result.optimalArg = new double[argNr];
    perturbationNr = 1e-3;
    initialGain = -2;
    innerLoopMax = 30;
}
~cOptimization()
{
    delete Result.optimalArg;
}
    struct
    {
        double optimalCost;
        double *optimalArg;
        int outerLoopCount;
    } Result;

    void startOptimization(double *initialInput_ = 0, int maxSteps_ = 100, double threshold_ = 1e-6);


private:
double (*costFctPt)(double* argArray);
int argNr;
double perturbationNr;
double initialGain;
int innerLoopMax;

};


///////////////////////////////////////// Dynamic Array
template <class T> class cList
{
public:
    cList(): listLength(0) // Constructor
    {

    }
    cList(cList &obj): listLength(0) // Copy Constructor
    {
        attachList(obj);
    }

    ~cList() // Destructor
    {
        if (listLength > 0) delete data;
    }
    void attach(T entry);
    void attachList(cList<T> &otherList);
    void clearList();
    T& operator[](int i);
    int length();

private:
    T *data;
    int listLength;
};


//////////////////////////////////////////////////////////////////////////Definitions
///

//////////////////////////////////////////////// MATRIX
template <uint8_t n, uint8_t m> inline float& cMatrix<n,m>::operator ()(int i, int j)
{
    return entries[i-1][j-1];
}

template <uint8_t n, uint8_t m, uint8_t l> inline cMatrix<n,m> operator*(cMatrix<n,l> left, cMatrix<l,m> right)
{
    cMatrix<n,m> result;
    for (int i=0; i<n; i++)
    {
        for (int j=0; j<m; j++)
        {
            float entry = 0;
            for (int k=0; k<l; k++)
            {
                entry += left(i+1,k+1)*right(k+1,j+1);
            }
            result(i+1,j+1) = entry;
        }
    }


    return result;
}

template <uint8_t n, uint8_t m> inline cMatrix<n,m> operator+(cMatrix<n,m> left, cMatrix<n,m> right)
{
    cMatrix<n,m> result;
    for (int i=0; i<n; i++)
    {
        for (int j=0; j<m; j++)
        {
            result(i+1,j+1) = left(i+1,j+1) + right(i+1,j+1);
        }
    }

    return result;
}

template <uint8_t n, uint8_t m> inline cMatrix<n,m> operator*(cMatrix<n,m> left, float number)
{
    cMatrix<n,m> result;
    for (int i=0; i<n; i++)
    {
        for (int j=0; j<m; j++)
        {
            result(i+1,j+1) = left(i+1,j+1)*number;
        }
    }

    return result;
}

template <uint8_t n, uint8_t m> inline cMatrix<n,m> operator*(float number, cMatrix<n,m> right)
{
    cMatrix<n,m> result;
    for (int i=0; i<n; i++)
    {
        for (int j=0; j<m; j++)
        {
            result(i+1,j+1) = number*right(i+1,j+1);
        }
    }

    return result;
}

template <uint8_t n, uint8_t m> inline cMatrix<n,m> operator-(cMatrix<n,m> left, cMatrix<n,m> right)
{
    return left+(-1)*right;
}

template <uint8_t n, uint8_t m> inline cMatrix<m,n> cMatrix<n,m>::T()
{
    cMatrix<m,n> result;
    for (int i=0; i<n; i++)
    {
        for (int j=0; j<m; j++)
        {
            result(j+1,i+1) = (*this)(i+1,j+1);
        }
    }

    return result;
}

template <uint8_t n, uint8_t m> inline float cMatrix<n,m>::norm()
{
    float result = 0;
    for (int i=0; i<n; i++)
    {
        for (int j=0; j<m; j++)
        {
            result += entries[i][j]*entries[i][j];
        }
    }

    return sqrt(result);
}

//////////////////////////////////////////////// MATRIX


//////////////////////////////////////////////// QUATERNION
inline cQuaternion cQuaternion::operator+(cQuaternion Q)
{
    cQuaternion Q_return;
    Q_return.values[0] = Q.values[0] + this->values[0];
    Q_return.values[1] = Q.values[1] + this->values[1];
    Q_return.values[2] = Q.values[2] + this->values[2];
    Q_return.values[3] = Q.values[3] + this->values[3];
    return Q_return;
}

inline cQuaternion cQuaternion::operator*(cQuaternion Q)
{
    cQuaternion Q_return;
    Q_return.values[0] = this->values[0]*Q.values[0] - this->values[1]*Q.values[1] - this->values[2]*Q.values[2] - this->values[3]*Q.values[3];
    Q_return.values[1] = this->values[1]*Q.values[0] + this->values[0]*Q.values[1] - this->values[3]*Q.values[2] + this->values[2]*Q.values[3];
    Q_return.values[2] = this->values[2]*Q.values[0] + this->values[3]*Q.values[1] + this->values[0]*Q.values[2] - this->values[1]*Q.values[3];
    Q_return.values[3] = this->values[3]*Q.values[0] - this->values[2]*Q.values[1] + this->values[1]*Q.values[2] + this->values[0]*Q.values[3];
    return Q_return;
}

inline cQuaternion cQuaternion::operator*(float f)
{
    cQuaternion Q_return;
    Q_return.values[0] = this->values[0] * f;
    Q_return.values[1] = this->values[1] * f;
    Q_return.values[2] = this->values[2] * f;
    Q_return.values[3] = this->values[3] * f;

    return Q_return;
}

inline float& cQuaternion::operator()(int i)
{
    return this->values[i-1];
}

inline void cQuaternion::norm()
{
    float betrag = sqrt(this->values[0]*this->values[0] + this->values[1]*this->values[1] + this->values[2]*this->values[2] + this->values[3]*this->values[3]);
    this->values[0] /= betrag;
    this->values[1] /= betrag;
    this->values[2] /= betrag;
    this->values[3] /= betrag;
}

inline cQuaternion cQuaternion::conjugated()
{
    cQuaternion tmp_q = *this;
    tmp_q.values[1] = -tmp_q.values[1];
    tmp_q.values[2] = -tmp_q.values[2];
    tmp_q.values[3] = -tmp_q.values[3];
    return tmp_q;
}

inline cQuaternion operator-(cQuaternion q1, cQuaternion q2)
{
    return q1 + q2*(-1);
}

inline cQuaternion operator*(float f, cQuaternion q)
{
    return q*f;
}
//////////////////////////////////////////////// QUATERNION

//////////////////////////////////////////////// FILTERS
inline void cDigitalFilter::update(float _u_k, float _T_s)
{
    u_kme = u_k;
    uI_kme = uI_k;
    upt1_kme = upt1_k;
    udt1_kme = udt1_k;

    u_k = _u_k;
    udot_kme = (1/_T_s) * (u_k - u_kme);


    upt1_k = upt1_kme + (_T_s/T_f)*(u_kme - upt1_kme);
    udt1_k = (1/T_f)*(u_k - u_kme) + udt1_kme*(1-(_T_s/T_f));
    uI_k = uI_kme + _T_s*u_kme;


}

inline void cFIRFilter::update(float u_k)
{
	float last = values.last();
    values.attach(u_k);
    u_FIR_k = u_FIR_kme + (values.first() - last)/(order);
    u_FIR_kme = u_FIR_k;
}

template <class T> inline void cRingBuffer<T>::attach(T value)
{
    currentIndex = (currentIndex+1)%length;
    buffer[currentIndex] = value;
}

template <class T> inline T cRingBuffer<T>::first()
{
    return buffer[currentIndex];
}

template <class T> inline T cRingBuffer<T>::last()
{
    return buffer[(currentIndex+1)%length];
}

template <class T> inline T& cRingBuffer<T>::operator [](int i)
{
    i = i%length;
    return buffer[(currentIndex+length-i)%length];
}

template <class T> inline int cRingBuffer<T>::size()
{
	return length;
}
//////////////////////////////////////////////// FILTERS



template <class T> inline T reverseBytes(T var)
{
        T reversed;
        uint8_t size = sizeof(T);
        uint8_t *ptT = (uint8_t*)&var;
        uint8_t *ptReversed = (uint8_t*)&reversed;
        for (int i=0; i<size; i++)
        {
            ptReversed[i] = ptT[size-1-i];
        }


        return reversed;
}


template <uint8_t n> inline cMatrix<n,1> solveLES(cMatrix<n,n> A, cMatrix<n,1> b)
{
    cMatrix<n,1> x;
    // Gauss Elimination
    for (int i=0; i<n-1; i++) // For every line, transform the lines below
    {
        // Pivoting: Swap current line with that line with greatest (norm) value at pivot position
            int max_line = i; // default = current line
            float max_value = A(i+1,i+1)*A(i+1,i+1); // default = current value
            for (int m=i+1; m<n; m++) // Check remaining lines for largest Pivot element
            {
                if ( A(m+1,i+1)*A(m+1,i+1) >= max_value) // line m has larger Pivot element
                {
                    max_value = A(m+1,i+1)*A(m+1,i+1);
                    max_line = m;
                }
            }
            // Swap i-th line with m-th line
            // Swapping lines in b
            float b_line_i = b(i+1,1);
            b(i+1,1) = b(max_line+1,1);
            b(max_line+1,1) = b_line_i;
            // Swapping lines in A
            float A_line_i_k;
            for (int k=0; k<n; k++)
            {
                A_line_i_k = A(i+1,k+1);
                A(i+1,k+1) = A(max_line+1,k+1);
                A(max_line+1,k+1) = A_line_i_k;
            }



        // Start transforming underlying lines...
        for (int j=i+1; j<n; j++) // for every line below the pivot line...
        {
            float factor = A(j+1,i+1)/A(i+1,i+1); // calculate pivot factor
            // Line Operation
            for (int k=0; k<n; k++) // substract pivotline*factor from that line
            {
                A(j+1,k+1) = A(j+1,k+1) - factor*A(i+1,k+1);
            }
            b(j+1,1) = b(j+1,1) - factor*b(i+1,1); // also from b vector

        }
    }
    // System should now be at upper triangular form...

    // Back Substitution
    for (int i=0; i<n; i++)
    {
        x(n-i,1) = b(n-i,1);
        for (int j=0; j<i; j++)
        {
            x(n-i,1) = x(n-i,1) - A(n-i,n-j)*x(n-j,1);
        }
        x(n-i,1) = x(n-i,1)/A(n-i,n-i);
    }

    return x;
}

////////////////////////////////// Optimization
inline void cOptimization::startOptimization(double *initialInput_, int maxSteps_, double threshold_)
{
    int i=0;

    ////////////////////////////////// Initial Argument
    double currentArg[argNr];
    if (initialInput_ == 0)
    {
        for (int j=0; j<argNr; j++) // init to 0
        {
            currentArg[j] = 0;
        }
    }
    else
    {
        for (int j=0; j<argNr; j++) // init to initial Input
        {
            currentArg[j] = initialInput_[j];
        }
    }
    ////////////////////////////////// Initial Argument

    ////////////////////////////////// Outer Loop
    while (i<maxSteps_)
    {
        double currentCost = costFctPt(currentArg); // current cost

     ////////////////////////////////// Gradient
        double gradient[argNr];
        double gainNorm = 0;
        // Generate gradient
        for (int input_i=0; input_i<argNr; input_i++) // For every Input
        {
            double perturbedInput[argNr];
            for (int perturbed_i=0; perturbed_i<argNr; perturbed_i++) // generate perturbed Input
            {
                perturbedInput[perturbed_i] = currentArg[perturbed_i];
                if (perturbed_i == input_i) perturbedInput[perturbed_i] += perturbationNr;
            }
            double perturbedCost = costFctPt(perturbedInput);
            gradient[input_i] = (1/perturbationNr)*(perturbedCost-currentCost);
            gainNorm += gradient[input_i]*gradient[input_i];
        }
     ////////////////////////////////// Gradient

     ////////////////////////////////// Backtracking Linesearch
        int backtracking_i = 0;
        double gain = initialGain;

        while (backtracking_i<innerLoopMax)
        {
            double inputCandidate[argNr];
            for (int input_i=0; input_i<argNr; input_i++) // Calculate Step
            {
                inputCandidate[input_i] = currentArg[input_i] + gain*gradient[input_i];
            }
            double costCandidate = costFctPt(inputCandidate);

            // Test Armijo Condition
            if (costCandidate <= currentCost + 0.5*gain*gainNorm)
            {
                for (int input_i=0; input_i<argNr; input_i++) // Update
                {
                    currentArg[input_i] = inputCandidate[input_i];
                }
                break;
            }
            else
            {
                gain *= 0.1;
            }

            backtracking_i++;
        }
     ////////////////////////////////// Backtracking Linesearch



        if (gainNorm <= threshold_) break;


        i++;
    }
    ////////////////////////////////// Outer Loop


    Result.optimalCost = costFctPt(currentArg);
    for (int input_i=0; input_i<argNr; input_i++)
    {
        Result.optimalArg[input_i] = currentArg[input_i];
    }

    Result.outerLoopCount = i;

}



///////////////////////////////////////////// cList


template <class T> inline void cList<T>::attach(T entry)
{
    T backupArray[listLength];
    for (int i=0; i<listLength; i++)
    {
        backupArray[i] = data[i];
    }
    if (listLength > 0) delete data;
    data = new T[listLength+1];
    for (int i=0; i<listLength; i++)
    {
        data[i] = backupArray[i];
    }
    data[listLength] = entry;
    listLength++;

}


template <class T> inline void cList<T>::attachList(cList<T> &otherList)
{
    T backupArray[listLength];
    for (int i=0; i<listLength; i++)
    {
        backupArray[i] = data[i];
    }
    if (listLength > 0) delete data;
    data = new T[listLength+otherList.length()];
    for (int i=0; i<listLength; i++)
    {
        data[i] = backupArray[i];
    }

    for (int i=0; i<otherList.length(); i++)
    {
        data[i+listLength] = otherList[i];
    }


    listLength += otherList.length();

}

template <class T> inline T& cList<T>::operator[](int i)
{
    return data[i];
}



template <class T> inline int cList<T>::length()
{
    return listLength;
}


template <class T> inline void cList<T>::clearList()
{
    if (listLength > 0) delete data;
    listLength = 0;
}



}

using namespace jorg;

#endif
