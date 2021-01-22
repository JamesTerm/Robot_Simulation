
#pragma once
#include "Base_Includes.h"
#include <math.h>
#include <assert.h>
#include <algorithm>
#undef min
#undef max

#if 0
    inline bool isNaN(float v) { return _isnan(v)!=0; }
    inline bool isNaN(double v) { return _isnan(v)!=0; }
#else
    inline bool isNaN(float v) { return false; }
    inline bool isNaN(double v) { return false; }
#endif
namespace Framework 
{
namespace Base 
{

/** General purpose double pair, uses include representation of
  * texture coordinates.
  * No support yet added for double * Vec2d - is it necessary?
  * Need to define a non-member non-friend operator* etc.
  * BTW: Vec2d * double is okay
*/

class Vec2d
{
    public:

        /** Type of Vec class.*/
        typedef double value_type;

        /** Number of vector components. */
        enum { num_components = 2 };
        
        value_type _v[2];

        Vec2d() {_v[0]=0.0; _v[1]=0.0;}

        Vec2d(value_type x,value_type y) { _v[0]=x; _v[1]=y; }

#if 0
        inline Vec2d(const Vec2f& vec) { _v[0]=vec._v[0]; _v[1]=vec._v[1]; }
        inline operator Vec2f() const { return Vec2f(static_cast<float>(_v[0]),static_cast<float>(_v[1]));}
#endif

        inline bool operator == (const Vec2d& v) const { return _v[0]==v._v[0] && _v[1]==v._v[1]; }

        inline bool operator != (const Vec2d& v) const { return _v[0]!=v._v[0] || _v[1]!=v._v[1]; }

        inline bool operator <  (const Vec2d& v) const
        {
            if (_v[0]<v._v[0]) return true;
            else if (_v[0]>v._v[0]) return false;
            else return (_v[1]<v._v[1]);
        }

        inline value_type* ptr() { return _v; }
        inline const value_type* ptr() const { return _v; }

        inline void set( value_type x, value_type y ) { _v[0]=x; _v[1]=y; }

        inline value_type& operator [] (size_t i) { return _v[i]; }
        inline value_type operator [] (size_t i) const { return _v[i]; }

        inline value_type& x() { return _v[0]; }
        inline value_type& y() { return _v[1]; }

        inline value_type x() const { return _v[0]; }
        inline value_type y() const { return _v[1]; }

        inline bool valid() const { return !isNaN(); }
        inline bool isNaN() const { return ::isNaN(_v[0]) || ::isNaN(_v[1]); }

        /** Dot product. */
        inline value_type operator * (const Vec2d& rhs) const
        {
            return _v[0]*rhs._v[0]+_v[1]*rhs._v[1];
        }

        /** Multiply by scaler. */
        inline const Vec2d operator * (value_type rhs) const
        {
            return Vec2d(_v[0]*rhs, _v[1]*rhs);
        }

        /** Unary multiply by scaler. */
        inline Vec2d& operator *= (value_type rhs)
        {
            _v[0]*=rhs;
            _v[1]*=rhs;
            return *this;
        }

        /** Divide by scaler. */
        inline const Vec2d operator / (value_type rhs) const
        {
            return Vec2d(_v[0]/rhs, _v[1]/rhs);
        }

        /** Unary divide by scaler. */
        inline Vec2d& operator /= (value_type rhs)
        {
            _v[0]/=rhs;
            _v[1]/=rhs;
            return *this;
        }

        /** Binary vector add. */
        inline const Vec2d operator + (const Vec2d& rhs) const
        {
            return Vec2d(_v[0]+rhs._v[0], _v[1]+rhs._v[1]);
        }

        /** Unary vector add. Slightly more efficient because no temporary
          * intermediate object.
        */
        inline Vec2d& operator += (const Vec2d& rhs)
        {
            _v[0] += rhs._v[0];
            _v[1] += rhs._v[1];
            return *this;
        }

        /** Binary vector subtract. */
        inline const Vec2d operator - (const Vec2d& rhs) const
        {
            return Vec2d(_v[0]-rhs._v[0], _v[1]-rhs._v[1]);
        }

        /** Unary vector subtract. */
        inline Vec2d& operator -= (const Vec2d& rhs)
        {
            _v[0]-=rhs._v[0];
            _v[1]-=rhs._v[1];
            return *this;
        }

        /** Negation operator. Returns the negative of the Vec2d. */
        inline const Vec2d operator - () const
        {
            return Vec2d (-_v[0], -_v[1]);
        }

        /** Length of the vector = sqrt( vec . vec ) */
        inline value_type length() const
        {
            return sqrt( _v[0]*_v[0] + _v[1]*_v[1] );
        }

        /** Length squared of the vector = vec . vec */
        inline value_type length2( void ) const
        {
            return _v[0]*_v[0] + _v[1]*_v[1];
        }

        /** Normalize the vector so that it has length unity.
          * Returns the previous length of the vector.
        */
        inline value_type normalize()
        {
            value_type norm = Vec2d::length();
            if (norm>0.0)
            {
                value_type inv = 1.0/norm;
                _v[0] *= inv;
                _v[1] *= inv;
            }
            return( norm );
        }

};    // end of class Vec2d

inline Vec2d GlobalToLocal(double Heading, const Vec2d &GlobalVector)
{
	return Vec2d(sin(-Heading)*GlobalVector[1] + cos(Heading)*GlobalVector[0],
		cos(-Heading)*GlobalVector[1] + sin(Heading)*GlobalVector[0]);
}

inline Vec2d LocalToGlobal(double Heading, const Vec2d &LocalVector)
{
	return Vec2d(sin(Heading)*LocalVector[1] + cos(-Heading)*LocalVector[0],
		cos(Heading)*LocalVector[1] + sin(-Heading)*LocalVector[0]);
}

inline bool PosBNE(double val, double t)
{
	return !(fabs(val - t) < 1E-3);
}


inline const Vec2d Vec2Multiply(const Vec2d &A, const Vec2d &rhs)
{
	return Vec2d(A[0] * rhs._v[0], A[1] * rhs._v[1]);
}

inline const Vec2d Vec2_abs(const Vec2d &A)
{
	return Vec2d(fabs(A[0]), fabs(A[1]));
}
inline const Vec2d Vec2_min(const Vec2d &A, const Vec2d &B)
{
	return Vec2d(std::min(A[0], B[0]), std::min(A[1], B[1]));
}
}    // end of namespace Base
}	//end of namespace Framework

typedef Framework::Base::Vec2d Vec2D;
