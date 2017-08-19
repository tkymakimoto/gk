/*
 * gkquaternion.h
 *
 *  Created on: 2014/05/27
 *      Author: Takuya Makimoto
 */

#ifndef GKQUATERNION_H_
#define GKQUATERNION_H_

#include "gkdef.h"

#include <cmath>
#include <algorithm>

namespace gk {

/**
 * @brief Quaternion class.
 * @f[
 * \mathbf{\tilde{Q}} = w + x\mathbf{I} + y\mathbf{J} + z\mathbf{K}
 * @f]
 *
 * @author Takuya Makimoto
 * @date 2016/01/25
 */
class quaternion {
public:
	typedef float_type value_type;

	/**
	 * @brief The enum of component numbers.
	 */
	enum {
		X, ///< Imaginary @f$x@f$-component number.
		Y, ///< Imaginary @f$y@f$-component number.
		Z, ///< Imaginary @f$z@f$-component number.
		W, ///< Real component number.
		Size, ///< The number of quaternion component.
	};

public:
	/**
	 * @brief The default constructor.
	 */
	quaternion() :
			x_() {
		this->x_[X] = value_type(GK_FLOAT_ZERO);
		this->x_[Y] = value_type(GK_FLOAT_ZERO);
		this->x_[Z] = value_type(GK_FLOAT_ZERO);
		this->x_[W] = value_type(GK_FLOAT_ONE);
	}

	quaternion(const quaternion& other) :
			x_() {
		this->x_[X] = other.x_[X];
		this->x_[Y] = other.x_[Y];
		this->x_[Z] = other.x_[Z];
		this->x_[W] = other.x_[W];
	}

	template<typename Vector>
	quaternion(const Vector& v, value_type w) :
			x_() {
		this->x_[X] = v[X];
		this->x_[Y] = v[Y];
		this->x_[Z] = v[Z];
		this->x_[W] = w;
	}

	quaternion(value_type x, value_type y, value_type z, value_type w) :
			x_() {
		this->x_[X] = x;
		this->x_[Y] = y;
		this->x_[Z] = z;
		this->x_[W] = w;
	}

	~quaternion() {
	}

	const value_type& x() const {
		return this->x_[X];
	}
	value_type& x() {
		return this->x_[X];
	}

	const value_type& y() const {
		return this->x_[Y];
	}
	value_type& y() {
		return this->x_[Y];
	}

	const value_type& z() const {
		return this->x_[Z];
	}

	value_type& z() {
		return this->x_[Z];
	}

	const value_type& w() const {
		return this->x_[W];
	}

	value_type& w() {
		return this->x_[W];
	}

	value_type square_norm() const {
		return this->x_[X] * this->x_[X] + this->x_[Y] * this->x_[Y]
				+ this->x_[Z] * this->x_[Z] + this->x_[W] * this->x_[W];
	}

	value_type norm() const {
		return std::sqrt(this->square_norm());
	}

	void negative() {
		this->operator *=(value_type(GK_FLOAT_NEGATIVE_ONE));
	}

	void conjugate() {
		const value_type f(GK_FLOAT_NEGATIVE_ONE);
		this->x_[X] *= f;
		this->x_[Y] *= f;
		this->x_[Z] *= f;
	}

	void inverse() {
		const value_type f = value_type(GK_FLOAT_ONE) / this->square_norm();
		this->x_[X] *= -f;
		this->x_[Y] *= -f;
		this->x_[Z] *= -f;
		this->x_[W] *= f;
	}

	const value_type& operator[](size_t n) const {
		return this->x_[n];
	}

	value_type& operator[](size_t n) {
		return this->x_[n];
	}

	quaternion& operator=(const quaternion& rhs) {
		if (&rhs == this) {
			return *this;
		}

		this->x_[X] = rhs.x_[X];
		this->x_[Y] = rhs.x_[Y];
		this->x_[Z] = rhs.x_[Z];
		this->x_[W] = rhs.x_[W];

		return *this;
	}

	quaternion& operator+=(const quaternion& rhs) {
		this->x_[X] += rhs.x_[X];
		this->x_[Y] += rhs.x_[Y];
		this->x_[Z] += rhs.x_[Z];
		this->x_[W] += rhs.x_[W];

		return *this;
	}

	quaternion& operator-=(const quaternion& rhs) {
		this->x_[X] -= rhs.x_[X];
		this->x_[Y] -= rhs.x_[Y];
		this->x_[Z] -= rhs.x_[Z];
		this->x_[W] -= rhs.x_[W];

		return *this;
	}

	quaternion& operator*=(const quaternion& rhs) {
		value_type f[] = { this->x_[X], this->x_[Y], this->x_[Z], this->x_[W] };
		this->x_[W] = -f[X] * rhs.x_[X] - f[Y] * rhs.x_[Y] - f[Z] * rhs.x_[Z]
				+ f[W] * rhs.x_[W];

		std::swap_ranges(f, f + Z, f + Z);
//		value_type f[] = { this->x_[Z], this->x_[W], this->x_[X], this->x_[Y] };
		this->x_[Y] = f[X] * rhs.x_[X] + f[Y] * rhs.x_[Y] - f[Z] * rhs.x_[Z]
				+ f[W] * rhs.x_[W];

		std::swap(f[X], f[Y]);
		std::swap(f[Z], f[W]);
//		value_type f[] = { this->x_[W], this->x_[Z], this->x_[Y], this->x_[X] };
		this->x_[X] = f[X] * rhs.x_[X] - f[Y] * rhs.x_[Y] + f[Z] * rhs.x_[Z]
				+ f[W] * rhs.x_[W];

		std::swap_ranges(f, f + Z, f + Z);
//		value_type f[] = { this->x_[Y], this->x_[X], this->x_[W], this->x_[Z] };
		this->x_[Z] = -f[X] * rhs.x_[X] + f[Y] * rhs.x_[Y] + f[Z] * rhs.x_[Z]
				+ f[W] * rhs.x_[W];

		return *this;
	}

	quaternion& operator*=(value_type rhs) {
		this->x_[X] *= rhs;
		this->x_[Y] *= rhs;
		this->x_[Z] *= rhs;
		this->x_[W] *= rhs;
		return *this;
	}

	quaternion& operator/=(value_type rhs) {
		const value_type inv_rhs = value_type(GK_FLOAT_ONE) / rhs;
		return this->operator *=(inv_rhs);
	}

private:
	value_type x_[Size];
}
;

quaternion conj(const quaternion& q) {
	quaternion dst = q;
	dst.conjugate();
	return dst;
}

quaternion operator-(const quaternion& q) {
	quaternion dst = q;
	dst.negative();
	return dst;
}

quaternion operator+(const quaternion& lhs, const quaternion& rhs) {
	quaternion dst(lhs);
	dst += rhs;
	return dst;
}

quaternion operator-(const quaternion& lhs, const quaternion& rhs) {
	quaternion dst = lhs;
	dst -= rhs;
	return dst;
}

quaternion operator*(const quaternion& lhs, float_type rhs) {
	quaternion dst = lhs;
	dst *= rhs;
	return dst;
}

quaternion operator*(float_type lhs, const quaternion& rhs) {
	return rhs * lhs;
}

quaternion operator*(const quaternion& lhs, const quaternion& rhs) {
	quaternion dst = lhs;
	dst *= rhs;
	return dst;
}

quaternion operator/(const quaternion& lhs, float_type rhs) {
	quaternion dst = lhs;
	dst /= rhs;
	return dst;
}

} // namespace gk

#endif /* GKQUATERNION_H_ */
