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



class cPT1Filter
{
  public:
  cPT1Filter(float _T_f)
  {
    T_f = _T_f;
    y_ = 0;
    ydot_ = 0;
    u_ = 0;
  }
  void update(float _u_k, float _T_s);
  float y()
  {
    return y_;
  }
  
  float ydot()
  {
    return ydot_;
  }

  private:
  float T_f, T_s, y_, ydot_,u_;
  
};

// Definitions
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


inline void cPT1Filter::update(float _u_k, float _T_s)
{
  ydot_ = (1/T_f) * (u_ - y_);
  y_ = y_  + _T_s * ydot_;
  u_ = _u_k; 
}


#endif
