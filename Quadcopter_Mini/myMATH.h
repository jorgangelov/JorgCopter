#ifndef MYMATH_H
#define MYMATH_H
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

// Declarations
template <uint8_t dim> class cVector
{
public:
  cVector()
  {
    memset(entries,0,sizeof(entries));
  }

  float& operator()(uint8_t _i);
  cVector<dim> operator+(cVector<dim> _other);
  cVector<dim> operator*(float _f);
  float entries[dim];
  float norm();

};

template <uint8_t dim> cVector<dim> operator*(float _f,cVector<dim> _vector);
template <uint8_t dim> float operator*(cVector<dim> v1, cVector<dim> v2);
template <uint8_t dim> cVector<dim> operator-(cVector<dim> v1, cVector<dim> v2);


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

cQuaternion operator-(cQuaternion q1, cQuaternion q2);
cQuaternion operator*(float f, cQuaternion q);



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


  float upt1_k, udt1_k, uI_k;

  protected:
  float udot_kme, upt1_kme, udt1_kme, uI_kme, u_kme, u_k;
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
    cFIRFilter(int _order): order(_order),u_FIR_kme(0),values(_order)
    {

    }

    void update(float u_k);
    float u_FIR_k;
protected:
    int order;
    cRingBuffer<float> values;
    float u_FIR_kme;

};


//////////////////////////////////////////////////////////////////////////Definitions
///
template <uint8_t dim> inline float& cVector<dim>::operator()(uint8_t _i)
{
  return entries[_i-1];
}

template <uint8_t dim> inline cVector<dim> cVector<dim>::operator+(cVector<dim> _other)
{
  cVector<dim> temp;
  for(int i=0; i<dim; i++)
  {
    temp.entries[i] = entries[i] + _other.entries[i];
  }

  return temp;
}

template <uint8_t dim> inline cVector<dim> cVector<dim>::operator*(float _f)
{
  cVector<dim> temp;
  for (int i=0; i<dim; i++)
  {
    temp.entries[i] = entries[i] * _f;
  }
  return temp;
}

template <uint8_t dim> inline cVector<dim> operator*(float _f, cVector<dim> _vector)
{
  cVector<dim> temp;
  for (int i=0; i<dim; i++)
  {
    temp(i+1) = _vector.entries[i] * _f;
  }
  return temp;
}


template <uint8_t dim> inline float operator*(cVector<dim> v1, cVector<dim> v2)
{
    float scalar = 0;
    for (int i=0; i<dim; i++)
    {
        scalar += v1.entries[i]*v2.entries[i];
    }
    return scalar;
}

template <uint8_t dim> inline cVector<dim> operator-(cVector<dim> v1, cVector<dim> v2)
{
    return v1 + (-1)*v2;
}

template <uint8_t dim> inline float cVector<dim>::norm()
{
  float mynorm = 0;
  for (uint8_t i=0; i<dim; i++)
  {
    mynorm += entries[i]*entries[i];
  }
  return sqrt(mynorm);
}


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


namespace aux
{

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
}
#endif
