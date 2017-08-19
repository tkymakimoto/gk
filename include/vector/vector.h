/*
 * algebra.h
 *
 *  Created on: 2016/07/29
 *      Author: J0115775
 */

#ifndef INCLUDE_VECTOR_ALGEBRA_H_
#define INCLUDE_VECTOR_ALGEBRA_H_

#include "../gkdef.h"
#include "../gkfunctional.h"
#include <Eigen/Core>

namespace gk {

//template<typename T, std::size_t DimensionSize>
//struct vector_type {
//	typedef Eigen::Matrix<T, 1, DimensionSize, Eigen::RowMajor> type;
//
//	typedef T value_type;
//	static const std::size_t Dimension = DimensionSize;
//};

template<typename Vector>
struct vector_traits {
	typedef typename Vector::value_type value_type;
	static const std::size_t Dimension = Vector::Dimension;
};

template<typename T, std::size_t DimensionSize>
struct vector_traits<Eigen::Matrix<T, DimensionSize, Eigen::RowMajor> > {
	typedef T value_type;
	static const std::size_t Dimension = DimensionSize;
};

template<typename Vector, typename T, std::size_t Dimension>
void assign(const Vector& src,
		const Eigen::Matrix<T, Dimension, Eigen::RowMajor>& dst) {
	assign_dimension(src, dst, dimension_tag<Dimension>());
}

template<typename Vector, typename T>
void assign_dimension(const Vector& src,
		const Eigen::Matrix<T, GK::GK_2D, Eigen::RowMajor>& dst,
		dimension_tag<GK::GK_2D>) {
	dst[GK::X] = src[GK::X];
	dst[GK::Y] = src[GK::Y];
}

template<typename Vector, typename T>
void assign_dimension(const Vector& src,
		const Eigen::Matrix<T, GK::GK_3D, Eigen::RowMajor>& dst,
		dimension_tag<GK::GK_3D>) {
	dst[GK::X] = src[GK::X];
	dst[GK::Y] = src[GK::Y];
	dst[GK::Z] = src[GK::Z];
}

template<typename T, std::size_t Dimension>
T norm(const Eigen::RowMajor<T, Dimension, Eigen::RowMajor>& v);

template<typename S, typename T, std::size_t Dimension>
typename multiplies_result<S, T>::value_type dot(
		const Eigen::Matrix<S, Dimension, Eigen::RowMajor>& u,
		const Eigen::Matrix<T, Dimension, Eigen::RowMajor>& v);

template<std::size_t Dimension>
class direction {
public:
	typedef float_type value_type;
	typedef Eigen::Matrix<value_type, Dimension, Eigen::RowMajor> vector_type;

private:
	template<typename T>
	static vector_type Normalized_(const vector_type& x) {

	}

	template<typename Vector>
	static vector_type Normalized_(const Vector& v) {
		typedef typename vector_traits<Vector>::value_type T;
		const typename multiplies_result<T, T>::value_type L2 = dot(v, v);

		const T L = std::sqrt(L2);

		return v / L;
	}

public:
	direction() :
			x_() {
	}

	direction(const direction& other) :
			x_(other.x_) {
	}

	template<typename Vector>
	direction(const Vector& v) :
			x_(Normalized_(v)) {
	}

	~direction() {
	}

	value_type operator[](std::size_t n) const {
		return this->x_[n];
	}

	value_type& operator[](std::size_t n) {
		return this->x_[n];
	}

	direction& operator=(const direction& rhs) {
		if (&rhs == this) {
			return *this;
		}

		this->x_ = rhs.x_;
		return *this;
	}

private:
	vector_type x_;
};

template<typename T, std::size_t Dimension>
Eigen::Matrix<T, Dimension, Eigen::RowMajor> operator*(const T& alpha,
		const direction<Dimension>& d) {
	Eigen::Matrix<T, Dimension, Eigen::RowMajor> vector_t;
	vector_t r;
	for (std::size_t i = 0; i < Dimension; ++i) {
		r[i] = alpha * d[i];
	}

	return r;
}

template<typename T, std::size_t Dimension>
Eigen::Matrix<T, Dimension, Eigen::RowMajor> operator*(
		const direction<Dimension>& d, const T& alpha) {
	return alpha * d;
}

} // namespace gk

#endif /* INCLUDE_VECTOR_ALGEBRA_H_ */
