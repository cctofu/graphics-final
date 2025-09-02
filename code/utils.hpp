#ifndef __UTILS_H__
#define __UTILS_H__

#include <cstdio>
#include <math.h>
#include <cstdlib>

const double PI = acos(-1);

double clamp(double x) {
    return std::max(0.0, std::min(1.0, x));
}

double rand_double(double minf=0.0, double maxf=1.0) {
    return minf + (maxf - minf) * drand48();
}

inline double random_double() {
    return rand() / (RAND_MAX + 1.0);
}

inline double random_double(double min, double max) {
    return min + (max-min)*random_double();
}

struct Vec3
{
    double x, y, z;
    Vec3(double x_ = 0, double y_ = 0, double z_ = 0) : x(x_), y(y_), z(z_) {}
    Vec3 operator+(const Vec3 &b) const { return Vec3(x + b.x, y + b.y, z + b.z); }
    Vec3 operator-(const Vec3 &b) const { return Vec3(x - b.x, y - b.y, z - b.z); }
    Vec3 operator*(double b) const { return Vec3(x * b, y * b, z * b); }
    Vec3 operator/(double b) const { return Vec3(x / b, y / b, z / b); }
    Vec3 operator+=(const Vec3 &b) { return *this = *this + b; }
    Vec3 operator-=(const Vec3 &b) { return *this = *this - b; }
    Vec3 operator*=(double b) { return *this = *this * b; }
    Vec3 operator/=(double b) { return *this = *this / b; }

    Vec3 mult(const Vec3 &b) const { return Vec3(x * b.x, y * b.y, z * b.z); }
    void normalize() { *this = *this * (1 / sqrt(x * x + y * y + z * z)); }
    Vec3 normalized() const { return *this * (1 / sqrt(x * x + y * y + z * z)); }
    bool operator==(const Vec3 &b) { return b.x == x && b.y == y && b.z == z; }
    double len() const { return sqrt(x * x + y * y + z * z); }
    double len2() const { return x * x + y * y + z * z; }
    double& operator [] ( int i ) {
        if (i == 0) return x;
        if (i == 1) return y;
        else return z;
    }
    double dot(const Vec3 &b) const { return x * b.x + y * b.y + z * b.z; } // cross:
    Vec3 operator%(const Vec3 &b) { return Vec3(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x); }
    Vec3 clip() const { return Vec3(clamp(x), clamp(y), clamp(z)); }
    Vec3 reflect(const Vec3 &n) { return (*this) - n * 2 * n.dot(*this); }
};

Vec3 random_in_unit_sphere()
{
    double theta = 2 * PI * drand48(); // random azimuthal angle
    double phi = acos(2 * drand48() - 1); // random polar angle
    double r = cbrt(drand48()); // cube root of random distance
    double sinPhi = sin(phi);
    return Vec3(r * sinPhi * cos(theta), r * sinPhi * sin(theta), r * cos(phi));
}

class Quat4f
{
public:

	static const Quat4f ZERO;
	static const Quat4f IDENTITY;

	Quat4f() = default;

	void setAxisAngle( float radians, const Vec3& axis ) {
		m_elements[ 0 ] = cos( radians / 2 );
		float sinHalfTheta = sin( radians / 2 );
		float vectorNorm = axis.len();
		float reciprocalVectorNorm = 1.f / vectorNorm;
		m_elements[ 1 ] = axis.x * sinHalfTheta * reciprocalVectorNorm;
		m_elements[ 2 ] = axis.y * sinHalfTheta * reciprocalVectorNorm;
		m_elements[ 3 ] = axis.z * sinHalfTheta * reciprocalVectorNorm;
	}

    Quat4f normalized() const {
		Quat4f q( *this );
		q.normalize();
		return q;
	}

    void normalize() {
		float abs =	sqrt(
			m_elements[ 0 ] * m_elements[ 0 ] +
			m_elements[ 1 ] * m_elements[ 1 ] +
			m_elements[ 2 ] * m_elements[ 2 ] +
			m_elements[ 3 ] * m_elements[ 3 ]
		);
		float reciprocalAbs = 1.f / abs;

		m_elements[ 0 ] *= reciprocalAbs;
		m_elements[ 1 ] *= reciprocalAbs;
		m_elements[ 2 ] *= reciprocalAbs;
		m_elements[ 3 ] *= reciprocalAbs;
	}

    float w() const {
		return m_elements[0];
	}

	float x() const {
		return m_elements[1];
	}

	float y() const {
		return m_elements[2];
	}
	
	float z() const{
		return m_elements[3];
	}

private:

	float m_elements[ 4 ];

};

class Matrix3f
{
public:

    // Fill a 3x3 matrix with "fill", default to 0.
	Matrix3f( float fill = 0.f );
	static Matrix3f rotation( const Quat4f& rq );
    Matrix3f( float m00, float m01, float m02,
		float m10, float m11, float m12,
		float m20, float m21, float m22 );
    const float& operator () ( int i, int j ) const {
		return m_elements[ j * 3 + i ];
	}

private:

	float m_elements[ 9 ];

};

Vec3 operator * ( const Matrix3f& m, Vec3 v )
{
	Vec3 output( 0, 0, 0 );

	for( int i = 0; i < 3; ++i )
	{
		for( int j = 0; j < 3; ++j )
		{
			output[ i ] += m( i, j ) * v[ j ];
		}
	}

	return output;
}

Matrix3f::Matrix3f( float fill )
{
	for( int i = 0; i < 9; ++i )
	{
		m_elements[ i ] = fill;
	}
}

Matrix3f::Matrix3f( float m00, float m01, float m02,
				   float m10, float m11, float m12,
				   float m20, float m21, float m22 )
{
	m_elements[ 0 ] = m00;
	m_elements[ 1 ] = m10;
	m_elements[ 2 ] = m20;

	m_elements[ 3 ] = m01;
	m_elements[ 4 ] = m11;
	m_elements[ 5 ] = m21;

	m_elements[ 6 ] = m02;
	m_elements[ 7 ] = m12;
	m_elements[ 8 ] = m22;
}



Matrix3f Matrix3f::rotation( const Quat4f& rq )
{
	Quat4f q = rq.normalized();

	float xx = q.x() * q.x();
	float yy = q.y() * q.y();
	float zz = q.z() * q.z();

	float xy = q.x() * q.y();
	float zw = q.z() * q.w();

	float xz = q.x() * q.z();
	float yw = q.y() * q.w();

	float yz = q.y() * q.z();
	float xw = q.x() * q.w();

	return Matrix3f
		(
			1.0f - 2.0f * ( yy + zz ),		2.0f * ( xy - zw ),				2.0f * ( xz + yw ),
			2.0f * ( xy + zw ),				1.0f - 2.0f * ( xx + zz ),		2.0f * ( yz - xw ),
			2.0f * ( xz - yw ),				2.0f * ( yz + xw ),				1.0f - 2.0f * ( xx + yy )
		);
}
#endif
