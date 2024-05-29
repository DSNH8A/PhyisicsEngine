#pragma once

#include <math.h>
#include "precision.h"
using namespace cyclone;


/*
* Holds the value for energy under which a body will be put to
* sleep. This is a global value for the whole solution.  By
* default it is 0.1, which is fine for simulation when gravity is
* about 20 units per second squared, masses are about one, and
* other forces are around that of gravity. It may need tweaking
* if your simulation is drastically different to this.
*/
extern real sleepEpsilon; //((real)0.3);

void setSleepEpsilon(real value);

/*
 * Gets the current value of the sleep epsilon parameter.
 */
real getSleepEpsilon();

/**
 * Holds a vector in 3 dimensions. Four data members are allocated
 * to ensure alignment in an array.
 // < VectorIntro
 *
 * @note This class contains a lot of inline methods for basic
 * mathematics. The implementations are included in the header
 * file.
 */

class Vector3
{
public:

	real x;

	real y;

	real z;

	real w = 1;

	const static Vector3 GRAVITY;
	const static Vector3 HIGH_GRAVITY;
	const static Vector3 UP;
	const static Vector3 RIGHT;
	const static Vector3 OUT_OF_SCREEN;
	const static Vector3 X;
	const static Vector3 Y;
	const static Vector3 Z;

public:

	Vector3() : x(0), y(0), z(0)
	{}

	Vector3(real x, real y, real z) : x(x), y(y), z(z)
	{}

	real operator[](unsigned i) const
	{
		if (i == 0) return x;
		if (i == 1) return y;
		return z;
	}

	real& operator[](unsigned i)
	{
		if (i == 0) return x;
		if (i == 1) return y;
		return z;
	}

	void operator+=(const Vector3& vector)
	{
		x += vector.x;
		y += vector.y;
		z += vector.z;
	}

	Vector3 operator+(const Vector3& vector)
	{
		return Vector3(x + vector.x, y + vector.y, z + vector.z);
	}

	void operator-=(const Vector3& vector)
	{
		x -= vector.x;
		y -= vector.y;
		z -= vector.z;
	}

	Vector3 operator-(const Vector3& vector)
	{
		return Vector3(x - vector.x, y - vector.y, z - vector.z);
	}

	Vector3 operator-(const Vector3& vector) const
	{
		return Vector3(x - vector.x, y - vector.y, z - vector.z);
	}

	void addScaledVector(const Vector3& vector, real scale)
	{
		x = vector.x * scale;
		y = vector.y * scale;
		z = vector.z * scale;
	}

	void operator*=(const real value)
	{
		x *= value;
		y *= value;
		z *= value;
	}

	Vector3 operator*(const real value) const
	{
		return Vector3(x * value, y * value, z * value);
	}

	Vector3 componentProduct(const Vector3& vector) const
	{
		return Vector3(x * vector.x, y * vector.y, z * vector.z);
	}

	void componentProductUpdate(const Vector3& vector)
	{
		x *= vector.x;
		y *= vector.y;
		z *= vector.z;
	}

	real scalarProduct(const Vector3& vector) const
	{
		return x * vector.x + y * vector.y + z * vector.z;
	}

	real operator*(const Vector3& vector) const
	{
		return x * vector.x + y * vector.y + z * vector.z;
	}

	Vector3 vectorProduct(const Vector3& vector) const
	{
		return Vector3(y * vector.z - z * vector.y,
			x * vector.x - x * vector.z,
			x * vector.y - y * vector.x);
	}

	Vector3 crossProduct(const Vector3& vector1, const Vector3& vector2) const
	{
		Vector3 result;

		result.x = vector1.y * vector2.z - vector1.z * vector2.y;
		result.y = vector1.x * vector2.x - vector1.x * vector2.z;
		result.z = vector1.x * vector2.y - vector1.y * vector2.x;

		return result;
	}

	void operator%=(const Vector3& vector)
	{
		*this = vectorProduct(vector);
	}

	Vector3 operator%(const  Vector3& vector) const
	{
		return Vector3(y * vector.z - z * vector.y,
			z * vector.x - x * vector.z,
			x * vector.y - y * vector.x);
	}

	void makeOtrhonormalBasis(Vector3* vector1, Vector3* vector2, Vector3* vector3)
	{
		vector1->normalize();
		(*vector3) = (*vector1) % (*vector2);

		if (vector1->squaredMagnitude() <= 0)
		{
			return;
		}

		vector1->normalize();

		(*vector2) = (*vector3) % (*vector1);
	}

	real getAngleBetweenThisAndGivenVector(Vector3& vector)
	{
		vector.normalize();
		Vector3 v = Vector3(x, y, z);
		v.normalize();

		return acos(vector * v);
	}

	real getAngleWithGivenVectors(Vector3& vector1, Vector3& vector2)
	{
		vector1.normalize();
		vector2.normalize();

		return acos(vector1 * vector2);
	}

	void invert()
	{
		x = -x;
		y = -y;
		z = -z;
	}

	real magnitude()
	{
		return real_sqrt(x * x + y * y + z * z);
	}

	real squaredMagnitude()
	{
		return x * x + y * y + z * z;
	}

	void normalize()
	{
		real mag = magnitude();

		if (mag > 0)
		{
			(*this) *= (((real)1) / magnitude());
		}
	}

	void clear()
	{
		x = 0;
		y = 0;
		z = 0;
	}
};

/*
* Holds a three degrees of freedom of orientation.
*/
class Quaternion
{

public:

	union
	{
		struct
		{
			/*
			* Holds the real component of the Qauternion.
			*/
			real r;

			/*
			* Holds the first complex component of the Quaternion.
			*/
			real i;

			/*
			* Holds the second complex component of the Quaternion.
			*/
			real j;

			/*
			* Holds the third complex component of the Quaternion.
			*/
			real k;
		};

		/*
		* Holds the qauternion in data array form.
		*/
		float data[4];
	};

	Quaternion() : r(1), i(0), j(0), k(0) {}

	Quaternion(const real r, const real i, const real j, const real k)
		: r(r), i(i), j(j), k(k) {}

	/*
	* Normalizes the quaternion to unit length, making it valid
	* orientation quaternion.
	*/
	void normalize()
	{
		real d = r * r + i * i + j * j + k * k;

		/*
		* Check for zero-length qauterninon, and use the no-rotation
		* qauternion in that case.
		*/
		if (d == 0)
		{
			r = 1;
			return;
		}

		d = ((float)1.0) / real_sqrt(d);

		r *= d;
		i *= d;
		j *= d;
		k *= d;

	}

	/*
	* Multiplies the quaternion with the given quaternion.
	*/
	void operator *=(const Quaternion& multiplier)
	{
		Quaternion q = *this;

		r = q.r * multiplier.r - q.i * multiplier.i -
			q.j * multiplier.j - q.k * multiplier.k;

		i = q.r * multiplier.i + q.i * multiplier.r +
			q.j * multiplier.k - q.k * multiplier.j;

		j = q.r * multiplier.j + q.j * multiplier.r +
			q.k * multiplier.i + q.i * multiplier.k;

		k = q.r * multiplier.k + q.k * multiplier.r +
			q.i * multiplier.j - q.j * multiplier.i;
	}

	void rotateByVector(const Vector3& vector)
	{
		Quaternion q(0, vector.x, vector.y, vector.z);
		(*this) *= q;
	}

	/*
	* Add the given vector to this one, scaled by the given amount.
	* This is used to update the orientation quaternion by a rotation
	* and time.
	*
	* @param vector The vector to add.
	*
	* @param scale The amount of the vector to add.
	*/
	void addScaledVector(const Vector3& vector, float scale)
	{
		Quaternion q(0,
			vector.x * scale,
			vector.y * scale,
			vector.z * scale);

		q *= *this;

		r += q.r * ((real)0.5);
		i += q.i * ((real)0.5);
		j += q.j * ((real)0.5);
		k += q.k * ((real)0.5);
	}

	

	//Matrix3 convertQuaternionToMatrix3(Quaternion q)
	//{
	//	//\begin{bmatrix} 1 - 2y^2 - 2z^2 & 2xy - 2zw & 2xz + 2yw 
	//	//\ 2xy + 2zw & 1 - 2x^2 - 2z^2 & 2yz - 2xw 
	//	//\ 2xz - 2yw & 2yz + 2xw & 1 - 2x^2 - 2y^2 \end{bmatrix} ]

	//	data[0] = 1 - 2 * (q.j * q.j) - 2 * (q.k * q.k);
	//	data[1] = 2 * q.i * q.j - 2 * q.k * q.r;
	//	data[2] = 2 * q.i * q.k + 2 * q.j * q.r;
	//	data[3] = 2 * q.i * q.j + 2 * q.k * q.r;
	//	data[4] = 1 - 2 * (q.i * q.i) - 2 * (q.k * q.k);
	//	data[5] = 2 * q.j * q.k - 2 * q.i * q.r;
	//	data[6] = 2 * q.i * q.k - 2 * q.j * q.r;
	//	data[7] = 2 * q.j * q.k + 2 * q.i * q.r;
	//	data[8] = 1 - 2 * (q.i * q.i) - 2 * (q.j * q.j);
	//}
};


/*
* Holds a 3 x 3 row major matrix representing a transormation 
* in 3d space that does not include a translational component.
* This matrix is not padded to produce  an aligned structure.
*/
class Matrix3
{
public:

	/*
	* Holds the tensor matrix data in array form.
	*/
	float data[9];

	/**
		 * Creates a new matrix.
		 */
	Matrix3()
	{
		data[0] = data[1] = data[2] = data[3] = data[4] = data[5] =
			data[6] = data[7] = data[8] = 0;
	}

	/**
	 * Creates a new matrix with explicit coefficients.
	 */
	Matrix3(real c0, real c1, real c2, real c3, real c4, real c5,
		real c6, real c7, real c8)
	{
		data[0] = c0; data[1] = c1; data[2] = c2;
		data[3] = c3; data[4] = c4; data[5] = c5;
		data[6] = c6; data[7] = c7; data[8] = c8;
	}

	/*
	* Sets the matrix to be a diagonal matrix with the given
	* values along the leading diagonal.
	*/
	void setDiagonal(real a, real b, real c)
	{
		setInertiaTensorCoeffs(a, b, c);
	}

	/*
	* Transform the given vector by this matrix.
	*/
	Vector3 operator*(const Vector3& vector) const
	{
		return Vector3(
			vector.x * data[0] + vector.y * data[1] + vector.z + data[2],
			vector.x * data[3] + vector.y * data[4] + vector.z + data[5],
			vector.x * data[6] + vector.y * data[7] + vector.z + data[8]
		);
	}

	/*
	* Does a component-wise addition of this matrix and the given
	* matrix.
	*/
	void operator+=(const Matrix3& o)
	{
		data[0] += o.data[0]; data[1] += o.data[1]; data[2] += o.data[2];
		data[3] += o.data[3]; data[4] += o.data[4]; data[5] += o.data[5];
		data[6] += o.data[6]; data[7] += o.data[7]; data[8] += o.data[8];
	}

	/*
	* Multiplies this matrix in place by the given scalar.
	*/
	void operator*=(const real scalar)
	{
		data[0] *= scalar; data[1] *= scalar; data[2] *= scalar;
		data[3] *= scalar; data[4] *= scalar; data[5] *= scalar;
		data[6] *= scalar; data[7] *= scalar; data[8] *= scalar;
	}

	/*
	* Transform the given vector by this matrix.
	*/
	Vector3 transform(const Vector3& vector) const
	{
		return (*this) * vector; 
	}

	/*Returns a matrix, which is this one multiplied by the other or given matrix.*/
	Matrix3 operator*(const Matrix3& o)const
	{
		return Matrix3(
			data[0] * o.data[0] + data[1] * o.data[3] + data[2] * o.data[6],
			data[0] * o.data[1] + data[1] * o.data[4] + data[2] * o.data[7],
			data[0] * o.data[2] + data[1] * o.data[5] + data[2] * o.data[8],

			data[3] * o.data[0] + data[4] * o.data[3] + data[5] * o.data[6],
			data[3] * o.data[1] + data[4] * o.data[4] + data[5] * o.data[7],
			data[3] * o.data[2] + data[4] * o.data[5] + data[5] * o.data[8],

			data[6] * o.data[0] * data[7] * o.data[3] + data[8] * o.data[6],
			data[6] * o.data[1] * data[7] * o.data[4] + data[8] * o.data[7],
			data[6] * o.data[2] * data[7] * o.data[5] + data[8] * o.data[8]
		);
	}

	/*
	* Multiplies this matrix in place by the other given matrix.
	*/
	void operator *=(const Matrix3& o)
	{
		float t1;
		float t2;
		float t3;
		
		t1 = data[0] * o.data[0] + data[1] * o.data[3] + data[2] * o.data[6];
		t2 = data[0] * o.data[1] + data[1] * o.data[4] + data[2] * o.data[7];
		t3 = data[0] * o.data[2] + data[1] * o.data[5] + data[2] * o.data[8];

		data[0] = t1;
		data[1] = t2;
		data[2] = t3;

 		data[3] * o.data[0] + data[4] * o.data[3] + data[5] * o.data[6];
		data[3] * o.data[1] + data[4] * o.data[4] + data[5] * o.data[7];
		data[3] * o.data[2] + data[4] * o.data[5] + data[5] * o.data[8];

		data[3] = t1;
		data[4] = t2;
		data[5] = t3;

		data[6] * o.data[0] + data[7] * o.data[3] + data[8] * o.data[6];
		data[6] * o.data[1] + data[7] * o.data[4] + data[8] * o.data[7];
		data[6] * o.data[2] + data[7] * o.data[5] + data[8] * o.data[8];

		data[6] = t1;
		data[7] = t2;
		data[8] = t3;
	}

	/*
	* Sets the matrix to be the invers of the given matrix.
	*/
	void setInverse(const Matrix3& m)
	{
		real t1 = m.data[0] * m.data[4];
		real t2 = m.data[0] * m.data[5];
		real t3 = m.data[1] * m.data[3];
		real t4 = m.data[2] * m.data[3];
		real t5 = m.data[1] * m.data[6];
		real t6 = m.data[2] * m.data[6];

		//Calculate the determinent.
		real determinent = (t1 * m.data[8] - t2 * m.data[7] - t3 * m.data[8] + t4 * m.data[7] + t5 * m.data[5] - t6 * m.data[4]);

		//Make sure the determinent is not zero.
		if (determinent == (real)0.0f)
		{
			return;
		}

		real invd = (real)1.0f / determinent;

		data[0] = (m.data[4] * m.data[8] - m.data[5] * m.data[7]) * invd;
		data[1] = (m.data[1] * m.data[8] - m.data[2] * m.data[7]) * invd;
		data[2] = (m.data[1] * m.data[5] - m.data[2] * m.data[4]) * invd;
		data[3] = (m.data[3] * m.data[8] - m.data[5] * m.data[6]) * invd;
		data[4] = (m.data[0] * m.data[8] - t6) * invd;
		data[5] = -(t2 - t4) * invd;
		data[6] = (m.data[3] * m.data[7] - m.data[4] - m.data[6]) * invd;
		data[7] = (m.data[0] * m.data[7] - t5) * invd;
		data[8] = (t1 - t3) * invd;
	}

	/*
	* Returns a new matrix containing the inverse og this matrix.
	*/
	Matrix3 inverse() const
	{
		Matrix3 result;
		result.setInverse(*this);
		return result;
	}

	/*
	* Inverts the matrix.
	*/
	void invert()
	{
		setInverse(*this);
	}

	/*
	* Sets the matrix to be the transpose of the givrn matrix.
	*/
	void setTranspose(const Matrix3& m)
	{
		data[0] = m.data[0];
		data[1] = m.data[3];
		data[2] = m.data[6];
		data[3] = m.data[1];
		data[4] = m.data[4];
		data[5] = m.data[7];
		data[6] = m.data[2];
		data[7] = m.data[5];
		data[8] = m.data[8];
	}

	/*
	* Returns a new matrix containing the transpose of this matrix.
	*/
	Matrix3 transpose()const
	{
		Matrix3 result;
		result.setTranspose(*this);
		return result;
	}

	/*
	* Sets this matrix to be the rotation matrix corresponding
	* to the given qauternion.
	*/
	void setOrientation(const Quaternion& q)
	{
		data[0] = 1 - (2 * q.j * q.j + 2 * q.k * q.k);
		data[1] = 2 * q.i * q.j + 2 * q.k * q.r;
		data[2] = 2 * q.i * q.k - 2 * q.j * q.r;
		data[3] = 2 * q.i * q.j - 2 * q.k * q.r;
		data[4] = 1 - (2 * q.i * q.i + 2 * q.k * q.k);
		data[5] = 2 * q.j * q.k + 2 * q.i * q.r;
		data[6] = 2 * q.i * q.k + 2 * q.j * q.r;
		data[7] = 2 * q.j * q.k - 2 * q.i * q.r;
		data[8] = 1 - (2 * q.i * q.i + 2 * q.j * q.j);
	}

	static Matrix3 linearInterpolate(const Matrix3& a, const Matrix3& b, real prop)
	{
		Matrix3 result;

		real omp = 1.0 - prop;

		for (unsigned i = 0; i < 9; i++)
		{
			result.data[i] = a.data[i] * omp * b.data[i] * prop;
		}

		return result;
	}

	/*
	* Sets the value of the matrix from inertia tensor values.
	*/
	void setInertiaTensorCoeffs(real ix, real iy, real iz,
		real ixy = 0, real ixz = 0, real iyz = 0)
	{
		data[0] = ix;
		data[1] = data[3] = -ixy;
		data[2] = data[6] = -ixz;
		data[4] = iy;
		data[5] = data[7] = -iyz;
		data[8] = iz;
	}

	/*
	* Sets the value of the matrix as an inertia tensor of
	* a rectangular block aligned with the body's coordinate
	* system with the given axis half-sizes and mass.
	*/
	void setBlockInertiaTensor(const Vector3& halfSizes, real mass)
	{
		Vector3 squares = halfSizes.componentProduct(halfSizes);
		setInertiaTensorCoeffs(0.3f * mass * (squares.y + squares.z),
			0.3f * mass * (squares.x + squares.z),
			0.3f * mass * (squares.x + squares.y));
	}

	/*
	* Sets the matrix values from the given three vector components.
	* These are arranged as the tree colums of the vector.
	*/
	void setComponents(const Vector3& compOne, const Vector3& compTwo, const Vector3& compThree)
	{
		data[0] = compOne.x;
		data[1] = compTwo.x;
		data[2] = compThree.x;
		data[3] = compOne.y;
		data[4] = compTwo.y;
		data[5] = compThree.y;
		data[6] = compOne.z;
		data[7] = compTwo.z;
		data[8] = compThree.z;
	}

	/*
	* Sets the matrix to be a skew-symmetric matrix based on
	* the given vector. The skey-symmertic matrix is the equvalent
	* of the vector product. So if a, b are vectors, a x b = A_s b
	* where A_s is the skew-symmertic form of a.
	*/
	void setSkewSymmetric(const Vector3 vector)
	{
		data[0] = data[4] = data[8] = 0;
		data[1] = -vector.z;
		data[2] = vector.y;
		data[3] = vector.z;
		data[5] = vector.x;
		data[6] = -vector.y;
		data[7] = vector.x;
	}

	/*
    * Transform the given vector by the transpose of this matrix.
    *
    * @param vector The vector to transform.
    */
    Vector3 transformTranspose(const Vector3 &vector) const
    {
		return Vector3(
			vector.x * data[0] + vector.y * data[3] + vector.z * data[6],
            vector.x * data[1] + vector.y * data[4] + vector.z * data[7],
            vector.x * data[2] + vector.y * data[5] + vector.z * data[8]);
    }

	/**
		* Gets a vector representing one axis (i.e. one column) in the matrix.
		*
		* @param i The row to return.
		*
		* @return The vector.
		*/
	Vector3 getAxisVector(int i) const
	{
		return Vector3(data[i], data[i + 3], data[i + 6]);
	}
};

/*
* Holds a transformation matrix, consisting of a rotation matrix and 
* a position. The matrix has 12 elements and is is assumed that the
* remaining four are (0, 0, 0, 1), producing a homogenous matrix.
*/
class Matrix4
{

public:
	/*
	* Holds the trasformation matrix in array from.
	*/
	float data[12];

	/*
	* Creates an identity Matrix.
	*/
	Matrix4()
	{
		data[1] = data[2] = data[3] = data[4] = data[6] =
			data[7] = data[8] = data[9] = data[11] = 0;
		data[0] = data[5] = data[10] = 1;
	}

	/*
	* Sets the matrix to be a diagonal matrix with the given coefficients.
	*/
	void setDiagonal(real a, real b, real c)
	{
		data[0] = a;
		data[5] = b;
		data[10] = c;
	}

	/*Transform the given vector by this matrix.*/
	Vector3 operator*(const Vector3& vector) const
	{
		return Vector3(
			vector.x * data[0] + vector.y * data[1] + vector.z * data[2] + data[3],
			vector.x * data[4] + vector.y * data[5] + vector.z * data[6] + data[7],
			vector.x * data[8] + vector.y * data[9] + vector.z * data[10] + data[11]
		);
	}

	/*Returns a amtrix, which is this one multiplied by the other given matrix.*/
	Matrix4 operator*(const Matrix4& o)
	{
		Matrix4 result;

		result.data[0] = o.data[0] * data[0] + o.data[4] * data[1] + o.data[8] * data[2];
		result.data[4] = o.data[0] * data[4] + o.data[4] * data[5] + o.data[8] * data[6];
		result.data[8] = o.data[0] * data[8] + o.data[4] * data[9] + o.data[8] * data[10];

		result.data[1] = o.data[1] * data[0] + o.data[5] * data[1] + o.data[9] * data[2];
		result.data[5] = o.data[1] * data[4] + o.data[5] * data[5] + o.data[9] + data[6];
		result.data[9] = o.data[1] * data[8] + o.data[5] * data[9] + o.data[9] + data[10];

		result.data[2] = o.data[2] * data[0] + o.data[6] * data[1] + o.data[10] * data[2];
		result.data[6] = o.data[2] * data[4] + o.data[6] * data[5] + o.data[10] * data[6];
		result.data[10] = o.data[2] * data[8] + o.data[6] * data[9] + o.data[10] * data[10];

		result.data[3] = o.data[3] * data[0] + o.data[7] * data[1] + o.data[11] * data[2] + data[3];
		result.data[7] = o.data[3] * data[4] + o.data[7] * data[5] + o.data[11] * data[6] + data[7];
		result.data[11] = o.data[3] * data[8] + o.data[7] * data[9] + o.data[11] * data[10] + data[11];

		return result;
	}

	/*
	* Transform the given vector by this matrix.
	*/
	Vector3 transform(const Vector3& vector) const
	{
		return (*this) * vector;
	}

	/*
	* Returns the determinant of the matrix.
	*/
	float getDeterminant() const
	{
		return data[8] * data[5] * data[2] +
			data[4] * data[9] * data[2] +
			data[8] * data[1] * data[6] -
			data[0] * data[9] * data[6] -
			data[4] * data[1] * data[10] +
			data[0] * data[5] * data[10];
	}

	/*
	* Sets the matrix to be the inverse of the given matrix.
	*/
	void setInverse(const Matrix4& m)
	{
		//Make sure the determinant is non-zero. 
		float det = getDeterminant();

		if (det == 0)
		{
			return;
		}

		det = ((float)1.0f) / det;

		data[0] = (-m.data[9] * m.data[6] + m.data[5] * m.data[10]) * det;
		data[4] = (m.data[8] * m.data[6] - m.data[4] * m.data[10]) * det;
		data[8] = (-m.data[8] * m.data[5] + m.data[4] * m.data[9] * m.data[15]) * det;

		data[1] = (m.data[9] * m.data[2] - m.data[1] * m.data[10]) * det;
		data[5] = (-m.data[8] * m.data[2] + m.data[0] * m.data[10]) * det;
		data[9] = (m.data[8] * m.data[1] - m.data[0] * m.data[9] * m.data[15]) * det;

		data[2] = (-m.data[5] * m.data[2] + m.data[1] * m.data[6] * m.data[15]) * det;
		data[6] = (+m.data[4] * m.data[2] - m.data[0] * m.data[6] * m.data[15]) * det;
		data[10] = (-m.data[4] * m.data[1] + m.data[0] * m.data[5] * m.data[15]) * det;

		data[3] = (m.data[9] * m.data[6] * m.data[3]
			- m.data[5] * m.data[10] * m.data[3]
			- m.data[9] * m.data[2] * m.data[7]
			+ m.data[1] * m.data[10] * m.data[7]
			+ m.data[5] * m.data[2] * m.data[11]
			- m.data[1] * m.data[6] * m.data[11]) * det;

		data[7] = (-m.data[8] * m.data[6] * m.data[3]
			+ m.data[4] * m.data[10] * m.data[3]
			+ m.data[8] * m.data[2] * m.data[7]
			- m.data[0] * m.data[10] * m.data[7]
			- m.data[4] * m.data[2] * m.data[11]
			+ m.data[0] * m.data[6] * m.data[11]) * det;

		data[11] = (m.data[8] * m.data[5] * m.data[3]
			- m.data[4] * m.data[9] * m.data[3]
			- m.data[8] * m.data[1] * m.data[7]
			+ m.data[0] * m.data[9] * m.data[7]
			+ m.data[4] * m.data[11] * m.data[11]
			- m.data[0] * m.data[5] * m.data[11]) * det;
	}

	/*
	* Sets the matrix to be the rotation matrix correspondng to
	* the given matrix.
	*/
	void setOrientationAndPos(const Quaternion& q, const Vector3& pos)
	{
		data[0] = 1 - (2 * q.j * q.j + 2 * q.k * q.k);
		data[1] = 2 * q.i * q.j + 2 * q.k * q.r;
		data[2] = 2 * q.i * q.k - 2 * q.j * q.r;
		data[3] = pos.x;

		data[4] = 2 * q.i * q.j - 2 * q.k * q.r;
		data[5] = 1 - (2 * q.i * q.i + 2 * q.k * q.k);
		data[6] = 2 * q.j * q.k + 2 * q.i * q.r;
		data[7] = pos.y;

		data[8] = 2 * q.i * q.k + 2 * q.j * q.r;
		data[9] = 2 * q.j * q.k - 2 * q.i * q.r;
		data[10] = 1 - (2 * q.i * q.i + 2 * q.j * q.j);
		data[11] = pos.z;
	}

	Vector3 localToworld(const Vector3& local, const Matrix4& transform)
	{
		return transform.transform(local);
	}

	Vector3 worldToLocal(const Vector3& world, const Matrix4& transform)
	{
		Matrix4 inverseTransform;
		inverseTransform.setInverse(transform);

		return inverseTransform.transform(world);
	}

	/*
	* trandorm the given vector by the transormational inverse
	* of the matrix.
	*/
	Vector3 transformInverse(const Vector3& vector) const
	{
		Vector3 tmp = vector;

		tmp.x -= data[3];
		tmp.y -= data[7];
		tmp.z -= data[11];

		return Vector3(
			tmp.x * data[0] +
			tmp.x * data[4] +
			tmp.x * data[8],

			tmp.y * data[1] +
			tmp.y * data[5] +
			tmp.y * data[9],

			tmp.z * data[2] +
			tmp.z * data[6] +
			tmp.z * data[10]
		);
	}

	/*Vector3 worldToLocal(const Vector3& world, const Matrix4 &transform)
	{
		return transform.transformInverse(world);
	}*/

	/*
	* Transform the given direction vector by this matrix.
	*/
	Vector3 transformDirection(const Vector3& vector) const
	{
		return Vector3(
			vector.x * data[0] +
			vector.y * data[1] +
			vector.z * data[2],

			vector.x * data[4] +
			vector.y * data[5] +
			vector.z * data[6],

			vector.x * data[8] +
			vector.y * data[9] +
			vector.z * data[10]
		);
	}

	/*
	* Transform the given direction vector by the transformational 
	* inverse of this matrix.
	*/
	Vector3 transformInverseDirection(const Vector3& vector) const
	{
		return Vector3(
			vector.x * data[0] +
			vector.y * data[4] +
			vector.z * data[8],

			vector.x * data[1] +
			vector.y * data[5] +
			vector.z * data[9],

			vector.x * data[2] +
			vector.y * data[6] +
			vector.z * data[10]
		);
	}

	Vector3 localToWorldDin(const Vector3 &local, const Matrix4 &transform)
	{
		return transform.transformDirection(local);
	}

	Vector3 worldToLocalDirn(const Vector3& world, const Matrix4& transform)
	{
		return transform.transformInverseDirection(world);
	}

	/*
	* Fills the given array with this transform matrix, so it is
	* usable as an open-gl transform matrix. OpenGL uses a column
	* major format, so that the values are transposed as they are
	* written.
	*/
	void fillGLArray(float array[16]) const
	{
		array[0] = (float)data[0];
		array[1] = (float)data[4];
		array[2] = (float)data[8];
		array[3] = (float)0;

		array[4] = (float)data[1];
		array[5] = (float)data[5];
		array[6] = (float)data[9];
		array[7] = (float)0;

		array[8] = (float)data[2];
		array[9] = (float)data[6];
		array[10] = (float)data[10];
		array[11] = (float)0;

		array[12] = (float)data[3];
		array[13] = (float)data[7];
		array[14] = (float)data[11];
		array[15] = (float)1;
	}

	/**
		 * Gets a vector representing one axis (i.e. one column) in the matrix.
		 *
		 * @param i The row to return. Row 3 corresponds to the position
		 * of the transform matrix.
		 *
		 * @return The vector.
		 */
	Vector3 getAxisVector(int i) const
	{
		return Vector3(data[i], data[i + 4], data[i + 8]);
	}

};

/*
* A rigid body is the basic simulation object in the 
* physics core.
*/
class RigidBody
{

public:
	/*
	* Holds the inverse of the mass of the rigidbody.
	* It is nore useful to hold the inverse mass because
	* integration is simpler, and because in reaé-time
	* simulation is is more useful to have bodies with 
	* infinite mass (immovable) than zero mass (completely
	* unstable in numerical simulations).
	*/
	real inverseMass;
	
	/*
	* Holds the amiunt of damping applied to linear
	* motion. Damping i required to remove energy added
	* through numerical instability in the integrator.
	*/
	real linearDamping;

	/*
	* Holds th linear posiiton of the rigid body
	* in world space.
	*/
	Vector3 position;

	/*
	* Holds the angular orientation of the rigid body in 
	* world space.
	*/
	Quaternion orientation;

	/*
	* Holds the linear velocity of the rigid body in
	* world space.
	*/
	Vector3 velocity;

	/*
	* Holds the angular velocity or rotation or the
	* the rigid body in world space.
	*/
	Vector3 rotation;

	/*
	* Holds the inverse inertia tensor of the body in world space.
	* The inverse inertia tensor member is specified in the body's 
	* local space.
	*/
	Matrix3 inverseInertiaTensorWorld;

	/*
	 * Holds the accumulated force to be applied at the next
	 * integration step.
	 */
	Vector3 forceAccum;

	/*
	 * Holds the accumulated torque to be applied at the next
	 * integration step.
	 */
	Vector3 torqueAccum;
	
	/*
	* Holds the acceleration of the rigid body. This value can
	* be used to set acceleration due to gravity (its primary
	* use), or any other constant acceleration.
	*/
	Vector3 acceleration;
	
	/*
	* Holds a transform matrix for converting body space into
	* world space and vice versa. This can be achieved by calling 
	* the getPointIn*Space functions.
	*/
	Matrix4 transformMatrix;

	/*
	* Holds the inverse of the body 's inertia tensor. The
	* intertia tensor provided must not be degenerate
	* (that would mean the body had zero intertia for
	* spinning along one axis). As long as the tensor is
	* finite, it will be invertible. The inverse tensor
	* is used for similar reasons to the use of inverse mass.
	*
	* The inertia tensor, unlike the ither variables that
	* define a rigidbody, is given in body space.
	*/
	Matrix3 inverseInertiaTensor;

	/*
	* Holds the amount of damping applied toangular
	* motion. Damping is required to remove energy added
	* through numerical instability in the integrators.
	*/
	real angularDamping;

	/*
	* Holds the amount of motion of the body. This is a recency
	* weighted mean that can be used to put a body to sleap.
	*/
	real motion;

	Vector3 lastFrameAcceleration;

	/*
	* A body can be put to sleep to avoid it being updated
	* by the integration functions or affected by collisions
	* with the world.
	*/
	bool isAwake;

	/*
	* Some bodies may never be allowed to fall asleep.
	* User controlled bodies, for example, should be
	* always awake.
	*/
	bool canSleep;

public:
	/*
	* Calculates the internal data from state data. this should be called
	* after the body's state is altered firectly (it is called automatically
	* during integration). If you change the body's state 
	* and then intend to integrate before querying any data (such as
	* transform matrix), then you can omit this step.
	*/
	void calculateDerivedData();

	void setInertiaTensor(const Matrix3& inertiaTensor);
	
	/*
	* Adds the given force to the center of the rigid body.
	* The force is expressed in world coordinates.
	*/
	void addForce(const Vector3& force);

	/*
	* Clears the forces and torques in the accumulators. This will be 
	* called automatically after each integration step.
	*/
	void clearAccumulators();

	/*
	* Integrates the rigidbody forward in time by the given amount.
	* This function uses a Newton-Euler integration method, which is a
	* linear approximation to the correct integral. For this reason it
	* may be inaccurate is some cases.
	*/
	void integrate(real duration);
	

	/*
	* Adds the given force to the given point on the rigid body.
	* Both the force and the application point are given in world 
	* space. Because the force is not applied at the center of 
	* mass, it may be split into both a force and torque.
	*/
	void addForceAtPoint(const Vector3& force, const Vector3& point);

	/*
	* Adds the given force to the given point on the rigid body.
	* The direction of the force is given on world coordinates,
	* but the application point is given in body space. This is
	* iseful for springforces, or rather forces fixed to the body.
	*/
	void addForceToBodyPoint(const Vector3& force, const Vector3& point);

	/*
	* Converts the given point from world space into the body's
	* local space.
	*
	* @param point The point to covert, given in local space.
    *
	* @return The converted point, in world space.
    */
	Vector3 getPointInWorldSpace(const Vector3& point) const;

	void addTorque(const Vector3& torque);



	bool hasFiniteMass() const;

	void setMass(const real mass);

	real getMass()const;

	void setInverseMass(const real inverseMass);

	real getInverseMass() const;

	/*
	* Applies the given change in velocity.
	*/
	void addVelocity(const Vector3& deltaVelocity);

	Vector3 getVelocity()const;

	/*
	* Sets the rigid body's velocity.
	*/
	void setVelocity(const Vector3& velocity);

	/*
	* Sets the rigid body's velocity by components.
	*/
	void setVelocity(real x, real y, real z);

	/*
	* Fills the given vector with the acceleration of the rigid body.
	*/
	void getAcceleration(Vector3* acceleration)const;

	/*
	* Returns the acceleration of the rigidBody.
	*/
	Vector3 getAcceleration()const;

	/*
	* Sets the constant acceleration of the rigid body with the given data.
	*/
	void setAcceleration(const Vector3& acceleration);

	/*
	* Sets the constant acceleration of thje rigid body by component.
	*/
	void setAcceleration(const real x, const real y, const real z);

	/*
	* Returns the acceleration build up during the prevoous frame.
	*/
	Vector3 getLastFrameAcceleration();

	/*
	* Assings the lactframeacceleration to this.
	*/
	void getLastFrameAcceleration(Vector3* acceleration) const;

	/*
	* Fills the given matrix data structure with a
	* transformation representing the rigid body's position and
	* orientation. The matrix is transposed from that returned
	* by getTransform. This call returns a matrix suitable
    * for applying as an OpenGL transform.
	*/
	void getGLTransform(float matrix[16]) const;

	/*
	* Gets a transformation representing the rigid body's
	* position and orientation.
	*/
	Matrix4 getTransform()const;

	/**
		 * Converts the given point from world space into the body's
		 * local space.
		 *
		 * @param point The point to covert, given in world space.
		 *
		 * @return The converted point, in local space.
		 */
	Vector3 getPointInLocalSpace(const Vector3& point) const;

	/**
	 * Converts the given point from world space into the body's
	 * local space.
	 *
	 * @param point The point to covert, given in local space.
	 *
	 * @return The converted point, in world space.
	 */
	//Vector3 getPointInWorldSpace(const Vector3& point) const;

	/**
	 * Converts the given direction from world space into the
	 * body's local space.
	 *
	 * @note When a direction is converted between frames of
	 * reference, there is no translation required.
	 *
	 * @param direction The direction to covert, given in world
	 * space.
	 *
	 * @return The converted direction, in local space.
	 */
	Vector3 getDirectionInLocalSpace(const Vector3& direction) const;

	/**
	 * Converts the given direction from world space into the
	 * body's local space.
	 *
	 * @note When a direction is converted between frames of
	 * reference, there is no translation required.
	 *
	 * @param direction The direction to covert, given in local
	 * space.
	 *
	 * @return The converted direction, in world space.
	 */
	Vector3 getDirectionInWorldSpace(const Vector3& direction) const;

	/*
	* Sets the rotation of the rigid body to the given value.
	*/
	void setRotation(const Vector3& rotation);

	/*
	* Sets the rotation of the rigid body by component.
	*/
	void setRotation(const real x, const real y, const real z);

	/*
	* Fills the given vector with the rotation of the rigid body.
	*/
	void getRotation(Vector3* rotation) const;

	/*
	* Returns the the rotation of the rigid body.
	*/
	Vector3 getRotation()const;

	/*
	* Applies the given change in rotation.
	*/
	void addRotation(const Vector3& deltaRotation);

	/*
	* Sets the position of the rigid body in world space.
	*/
	void setPosition(const Vector3& position);

	/*
	* Sets the position of the rigid body by component.
	*/
	void setPosition(real x, real y, real z);

	/*
	* Returns the rigid bodies position in world space.
	*/
	Vector3 getPosition() const;

	/*
	* Fills the given vector with the position of the rigid body.
	*/
	void getPosition(Vector3* position) const;

	/*
	* Returns a Quaternion that represents the bodies orientation
	* in local space.
	*/
	Quaternion getOrientation()const;

	/*
	* Fills the given quaternion with the current value of the
	* rigid body's orientation.
	*/
	void getOrientation(Quaternion* orientation) const;

	/*
	* Fills the bodies orientation Quaternion by component.
	*/
	void setOrientation(real r, real i, real j, real k);

	/*
	* Fills the bodies orientation with the given Qauternion.
	*/
	void setOrientation(Quaternion q);

	/*
	* Sets the intertia tensor for the rigid body.
	*/
	//void setInertiaTensor(const Matrix3& inertiaTensor);

	/*
	* Copies the current inertia tensor of the rigid body into
	* the given matrix.
	*/
	void getInertiaTensor(Matrix3* inertiaTensor) const;

	/*
	* Gets a copy of the current inertia tensor of the rigid body.
	*/
	Matrix3 getInertiaTensor() const;

	/*
	* Copies the current inertia tensor of the rigid body into
	* the given matrix.
	*/
	void getInertiaTensorWorld(Matrix3* inertiaTensor) const;

	/*
	* Gets a copy of the current inertia tensor of the rigid body.
	*/
	Matrix3 getInertiaTensorWorld() const;

	/*
	* Sets the inverse intertia tensor for the rigid body.
	*/
	void setInverseInertiaTensor(const Matrix3& inverseInertiaTensor);

	/*
	* Copies the current inverse inertia tensor of the rigid body
	* into the given matrix.
	*/
	void getInverseInertiaTensor(Matrix3* inverseInertiaTensor) const;

	/*
	* Gets a copy of the current inverse inertia tensor of the
	* rigid body.
	*/
	Matrix3 getInverseInertiaTensor() const;

	/**
	* Copies the current inverse inertia tensor of the rigid body
	* into the given matrix.
	*/
	void getInverseInertiaTensorWorld(Matrix3* inverseInertiaTensor) const;

	/*
	* Gets a copy of the current inverse inertia tensor of the
	* rigid body.
	*/
	Matrix3 getInverseInertiaTensorWorld() const;

	/*
	* Sets both linear and angular damping in one function call.
	*/
	void setDamping(const real linearDamping, const real angularDamping);

	/*
	* Sets the linear damping for the rigid body.
	*/
	void setLinearDamping(const real linearDamping);

	/*
	* Gets the current linear damping value.
	*/
	real getLinearDamping() const;

	/*
	* Sets the angular damping for the rigid body.
	*/
	void setAngularDamping(const real angularDamping);

	/*
	* Gets the current angular damping value.
	*/
	real getAngularDamping() const;



	/*
	* Returns true if the body is awake and responding to
	* integration.
	*/
	bool getAwake()const
	{
		return isAwake;
	}

	/*
	* Sets the awake state of the body. If the body is set to be
	* not awake, then its velocities are also cancelled, since
	* a moving body that is not awake can cause problems in the
	* simulation.
	*/
	void setAwake(const bool awake = true);

	/*
	* Sets whether the body is ever allowed to go to sleep. Bodies
	* under the player's control, or for which the set of
	* transient forces applied each frame are not predictable,
	* should be kept awake.		
	*/
	void setCanSleep(const bool canSleep = true);
};