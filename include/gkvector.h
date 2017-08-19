/*
 * gkvector.h
 *
 *  Created on: 2015/07/08
 *      Author: Takuya Makimoto
 */

/**
 * @file gkvector.h
 * @date 2017/08/15
 * @author Takuya Makimoto
 *
 */
#ifndef GKVECTOR_H_
#define GKVECTOR_H_

#include "gkdef.h"
#include "gkfunctional.h"
#include "gkgeometry.h"
#include "gkquaternion.h"

#include <numeric>
#include <cmath>
#include <utility>
#include <ostream>
#include <iterator>
#include <algorithm>
#include <functional>
#include <Eigen/Core>

namespace gk {

/**
 * @brief
 *
 * @tparam _T A type of a value.
 * @tparam _Dimension
 *
 * @author Takuya Makimoto
 * @date 2017/06/25
 * @version 0.X
 */
#if __cplusplus >= 201103L
template<typename _T, std::size_t _Dimension>
using vector = Eigen::Matrix<_T,1,_Dimension,Eigen::RowMajor>;
#else
template<typename _T, std::size_t _Dimension>
struct vector {
	typedef Eigen::Matrix<_T, 1, _Dimension, Eigen::RowMajor> type;
};
#endif

#if __cplusplus >= 201103L
#define GK_VECTOR_TYPEDEF(TYPE, DIMENSION) \
		   typedef vector<TYPE, DIMENSION> vector_type; \
		   typedef TYPE value_type;

#else
#define GK_VECTOR_TYPEDEF(TYPE, DIMENSION) \
	typedef typename vector<TYPE, DIMENSION>::type vector_type; \
	typedef TYPE value_type;

#endif

/**
 * @brief Traits of a vector.
 * @tparam _Vector The type of the vector in a vector space.
 *
 * @date 2016/02/24
 * @author Takuya Makimoto
 */
template<typename _Vector>
struct vector_traits {
	typedef typename _Vector::value_type value_type;
	static const std::size_t Dimension = _Vector::Dimension;
};

template<typename _T, std::size_t _Dimension>
struct vector_traits<vector<_T, _Dimension> > {
	typedef _T value_type;
	static const std::size_t Dimension = _Dimension;
};

#if __cplusplus >= 201103L
template<typename _T1, typename _T2, std::size_t _Dimension>
product<_T1, _T2> dot(const vector<_T1, _Dimension>& u,
		const vector<_T2, _Dimension>& v) {

	product<_T1, _T2> r = product<_T1, _T2>(GK_FLOAT_ZERO);
	for (std::size_t i = 0; i < _Dimension; ++i) {
		r += u[i] * v[i];
	}

	return r;
}

#else
template<typename Vector1, typename Vector2>
typename multiplies_result<typename vector_traits<Vector1>::value_type,
typename vector_traits<Vector2>::value_type>::value_type dot(
		const Vector1& u, const Vector2& v) {

#ifdef GK_DEBUG
#endif

	typedef typename multiplies_result<
	typename vector_traits<Vector1>::value_type,
	typename vector_traits<Vector2>::value_type>::value_type result_type;

	const std::size_t _Dimension = vector_traits<Vector1>::_Dimension;

	result_type r = result_type(GK_FLOAT_ZERO);
	for (size_t i = 0; i < _Dimension; ++i) {
		r += u[i] * v[i];
	}

	return r;
}
#endif

struct dot_function {
#if __cplusplus >= 201103L

#else
	template<typename Vector1, typename Vector2>
	typename multiplies_result<typename vector_traits<Vector1>::value_type,
	typename vector_traits<Vector2>::value_type>::value_type operator()(
			const Vector1& u, const Vector2& v) {
		typedef typename multiplies_result<
		typename vector_traits<Vector1>::value_type,
		typename vector_traits<Vector2>::value_type>::value_type result_type;

		const size_t _Dimension = vector_traits<Vector1>::_Dimension;

		result_type r = result_type(GK_FLOAT_ZERO);
		for (size_t i = 0; i < _Dimension; ++i) {
			r += u[i] * v[i];
		}

		return r;
	}
#endif

};

template<typename _Vector>
typename vector_traits<_Vector>::value_type norm(const _Vector& v) {
	return std::sqrt(dot(v, v));
}

//namespace impl {
//
//template<typename Vector1, typename Vector2, typename Result>
//void cross_kernel(const Vector1&, const Vector2&, Result& r,
//		dimension_tag<GK::GK_2D>) {
//	typedef typename vector_traits<Result>::value_type value_type;
//	r[GK::X] = value_type(GK_FLOAT_ZERO);
//	r[GK::Y] = value_type(GK_FLOAT_ZERO);
//}
//
//template<typename Vector1, typename Vector2, typename Result>
//void cross_kernel(const Vector1& u, const Vector2& v, Result& r,
//		dimension_tag<GK::GK_3D>) {
//	r[GK::X] = u[GK::Y] * v[GK::Z] - u[GK::Z] * v[GK::Y];
//	r[GK::Y] = u[GK::Z] * v[GK::X] - u[GK::X] * v[GK::Z];
//	r[GK::Z] = u[GK::X] * v[GK::Y] - u[GK::Y] * v[GK::X];
//}
//
//}

template<typename Vector1, typename Vector2 = Vector1, typename Result = Vector1>
struct cross {
	typedef Vector1 first_argument_type;
	typedef Vector2 second_argument_type;
	typedef Result result_type;

	Result operator()(const Vector1& u, const Vector2& v) const {
		Result r;
		r[GK::X] = u[GK::Y] * v[GK::Z] - u[GK::Z] * v[GK::Y];
		r[GK::Y] = u[GK::Z] * v[GK::X] - u[GK::X] * v[GK::Z];
		r[GK::Z] = u[GK::X] * v[GK::Y] - u[GK::Y] * v[GK::X];
		return r;
	}
};

/**
 * @brief Direction.
 *
 * @author Takuya Makimoto
 * @date 2016/01/25
 */
template<std::size_t _Dimension>
class direction {
public:
	typedef float_type value_type;
	typedef const value_type* const_iterator;
	typedef std::reverse_iterator<const_iterator> const_reverse_iterator;

	typedef vector<value_type, _Dimension> vector_type;

//	static const std::size_t Dimension = DimensionSize;
//	static const std::size_t ElementSize = DimensionSize;

public:
	/**
	 * @brief Default constructor.
	 *
	 * An instance made by this constructor is implemented a zero vector.
	 */
	direction() :
			x_() {
		std::fill(this->x_, this->x_ + _Dimension, value_type(GK_FLOAT_ZERO));
	}

	/**
	 * @brief Copy constructor.
	 * @param other
	 */
	direction(const direction& other) :
			x_() {
		std::copy(other.x_, other.x_ + _Dimension, this->x_);
	}

	template<typename _Vector>
	direction(const _Vector& v) :
			x_() {

	}

	template<typename _Vector>
	direction(const _Vector& start, const _Vector& end) :
			x_() {

		typedef vector_traits<_Vector> vtraits;
		typedef typename vtraits::value_type L_t;
		typedef product<L_t, L_t> L2_t;
		typedef quotient<value_type, L_t> InvL_t;

		_Vector v = end - start;
		typename vtraits::iterator first = vtraits::begin(v);
		typename vtraits::iterator last = vtraits::end(v);
		const L2_t L2 = std::inner_product(first, last, first,
				L2_t(GK_FLOAT_ZERO));
		const InvL_t F = value_type(GK_FLOAT_ONE) / std::sqrt(L2);

		std::transform(first, last, this->x_,
				std::bind2nd(multiplier<L_t, InvL_t>(), F));
	}

	~direction() {
	}

	const_iterator begin() const {
		return this->x_;
	}

	const_iterator end() const {
		return this->x_ + _Dimension;
	}

	const_reverse_iterator rbegin() const {
		return std::reverse_iterator<const_iterator>(this->end());
	}

	const_reverse_iterator rend() const {
		return std::reverse_iterator<const_iterator>(this->begin());
	}

	const value_type* data() const {
		return this->x_;
	}

	value_type operator[](const size_t n) const {
		return this->x_[n];
	}

	direction& operator=(const direction& u) {
		if (&u == this) {
			return *this;
		}

		std::copy(u.x_, u.x_ + _Dimension, this->x_);
		return *this;
	}

private:
	value_type x_[_Dimension];
};

/**
 * @brief The traits of a direction specified @code vector_traits.
 * @tparam _Dimension Dimension of this object.
 * @author Takuya Makimoto
 * @date 2016/03/07
 */
template<size_t _Dimension>
struct vector_traits<direction<_Dimension> > {
	typedef typename direction<_Dimension>::value_type value_type; ///< Type of elements in a vector.

	static const std::size_t Dimension = _Dimension; ///< A dimension size of a vector space.
	static const bool IsHomogeneous = false;

	typedef typename direction<Dimension>::const_iterator iterator;
	typedef typename direction<Dimension>::const_iterator const_iterator;
	typedef typename direction<Dimension>::const_reverse_iterator reverse_iterator;
	typedef typename direction<Dimension>::const_reverse_iterator const_reverse_iterator;

	static const_iterator begin(const direction<Dimension>& d) {
		return d.begin();
	}

	static iterator begin(direction<Dimension>& d) {
		return d.begin();
	}

	static const_iterator end(const direction<Dimension>& d) {
		return d.end();
	}

	static iterator end(direction<Dimension>& d) {
		return d.end();
	}

	static const_reverse_iterator rbegin(const direction<Dimension>& d) {
		return d.rbegin();
	}

	static reverse_iterator rbegin(direction<Dimension>& d) {
		return d.rbegin();
	}

	static const_reverse_iterator rend(const direction<Dimension>& d) {
		return d.rend();
	}

	static reverse_iterator rend(direction<Dimension>& d) {
		return d.rend();
	}
};

template<size_t _Dimension>
bool operator==(const direction<_Dimension>& u,
		const direction<_Dimension>& v) {
	return std::equal(u.begin(), u.end(), v.begin());
}

template<size_t _Dimension>
bool operator!=(const direction<_Dimension>& u,
		const direction<_Dimension>& v) {
	return !(u == v);
}

template<typename CharT, typename Traits, size_t _Dimension>
std::basic_ostream<CharT, Traits>& operator<<(
		std::basic_ostream<CharT, Traits>& os, const direction<_Dimension>& v) {
	const std::streamsize n = os.width();
	for (typename direction<_Dimension>::const_iterator p = v.begin();
			p != v.end(); ++p) {
		os.width(n);
		os << *p;
	}

	return os;
}

/**
 * @brief Outputs a direction of a geometry, @a a.
 *
 * @tparam _T The type of the geometry.
 * @param a
 * @return
 */
template<typename _T>
direction<geometry_traits<_T>::Dimension> direction_of(const _T& a);

/**
 * @brief Computes a direction of a vector.
 *
 * @f[
 * \mathbf{d} = \frac{\mathbf{v}}{|\mathbf{v}|}
 * @f]
 *
 * @param v The vector to be computed.
 * @return
 */
template<typename _Vector>
direction<vector_traits<_Vector>::Dimension> normalize(const _Vector& v) {
	return direction<vector_traits<_Vector>::Dimension>(
			vector_traits<_Vector>::begin(v));
}

/**
 * @brief Computes a norm of a direction. This function always returns @b1
 * because a direction is a unit vector.
 * @tparam Dimension Dimension of a vector space.
 * @param
 * @return Returns the magnitude of the direction, 1.
 */
template<size_t _Dimension>
float_type norm(const direction<_Dimension>&) {
	return float_type(GK_FLOAT_ONE);
}

namespace inner {

template<size_t Dimension, typename _Vector>
direction<Dimension> gk_normal_direction(const _Vector&, const _Vector&,
		dimension_tag<Dimension>) {
	return direction<Dimension>();
}

template<typename _Vector>
direction<GK::GK_3D> gk_normal_direction(const _Vector& u, const _Vector& v,
		dimension_tag<GK::GK_3D>) {
	const typename vector_traits<_Vector>::value_type Unit(GK_FLOAT_ONE);

	_Vector r;
	r[GK::X] = (u[GK::Y] * v[GK::Z] - u[GK::Z] * v[GK::Y]) / Unit;
	r[GK::Y] = (u[GK::Z] * v[GK::X] - u[GK::X] * v[GK::Z]) / Unit;
	r[GK::Z] = (u[GK::X] * v[GK::Y] - u[GK::Y] * v[GK::X]) / Unit;

	return direction<GK::GK_3D>(r);
}

} // namespace inner

/**
 * @brief Computes a normal vector made by 2 vectors @a u and @a v
 * in a 3D space.
 * @param u A vector for computing the normal vector.
 * @param v The other vector.
 * @return The normal vector to have been computed.
 */
template<typename _Vector>
direction<vector_traits<_Vector>::Dimension> normal_direction(const _Vector& u,
		const _Vector& v) {
	return inner::gk_normal_direction(u, v,
			dimension_tag<vector_traits<_Vector>::Dimension>());
}

/**
 * @brief Basis in a vector space.
 * @author Takuya Makimoto
 * @date 2015/12/09
 */
template<std::size_t _Dimension>
class basis {
public:
	static const std::size_t Dimension = _Dimension;
	typedef direction<_Dimension> direction_type;

private:
	static direction_type Value_(std::size_t n) {
		float_type x[_Dimension] = { float_type(GK_FLOAT_ZERO) };
		x[n] = GK_FLOAT_ONE;
		return direction_type(x);
	}

public:
	basis() {
	}

	~basis() {
	}

	direction_type operator[](std::size_t n) const {
		return Value_(n);
	}

private:
	basis(const basis&);
	basis& operator=(const basis&);
};

template<typename _Vector>
_Vector rotate(const _Vector& v, float_type angle) {
	const float_type sin = std::sin(angle);
	const float_type cos = std::cos(angle);

	_Vector r;
	r[GK::X] = v[GK::X] * cos - v[GK::Y] * sin;
	r[GK::Y] = v[GK::X] * sin + v[GK::Y] * cos;
	return r;
}

template<typename _Vector, typename _Angle>
_Vector rotate(const _Vector& v, const _Angle& angle) {

	const float_type theta = norm(angle);
	const direction<GK::GK_3D> axis(angle);

	const float_type sin = std::sin(0.5 * theta);
	const float_type cos = std::cos(0.5 * theta);

	const quaternion Q(sin * axis[GK::X], sin * axis[GK::Y], sin * axis[GK::Z],
			cos);

	typedef typename vector_traits<_Vector>::value_type value_type;
	const value_type unit = value_type(GK_FLOAT_ONE);

#if __cplusplus >= 201103L
	const vector<float_type, GK::GK_3D> u = v / unit;
#else
	const typename vector_type<float_type,GK::GK_3D>::type u=v/unit;
#endif
	const quaternion P(u, quaternion::value_type(GK_FLOAT_ZERO));

//		const quaternion R = conj(Q) * P * Q;
	const quaternion R = Q * P * conj(Q);

	_Vector r;
	r[GK::X] = R[quaternion::X] * unit;
	r[GK::Y] = R[quaternion::Y] * unit;
	r[GK::Z] = R[quaternion::Z] * unit;

	return r;
}

///**
// * @brief
// * @tparam _Vector
// * @tparam _Angle
// *
// * @author Takuya Makimoto
// * @date 2016/01/20
// */
//template<typename _Angle>
//struct rotate {
//	const _Angle angle;
//
//	rotate() :
//			angle() {
//	}
//
//	rotate(const _Angle& theta) :
//			angle(theta) {
//	}
//
//	template<typename _Vector, typename Axis>
//	_Vector operator()(const _Vector& v) const {
//		return rotate_(v, dimension_tag<vector_traits<_Vector>::Dimension>());
//	}
//
//private:
//	template<typename _Vector>
//	_Vector rotate_(const _Vector& v, dimension_tag<GK::GK_2D>) const {
//		const float_type sin = std::sin(this->angle);
//		const float_type cos = std::cos(this->angle);
//
//		_Vector r;
//		r[GK::X] = v[GK::X] * cos - v[GK::Y] * sin;
//		r[GK::Y] = v[GK::X] * sin + v[GK::Y] * cos;
//		return r;
//	}
//
//	template<typename _Vector>
//	_Vector rotate_(const _Vector& v, dimension_tag<GK::GK_3D>) const {
//		const float_type theta = norm(this->angle);
//		const direction<GK::GK_3D> axis(this->angle);
//
//		const float_type sin = std::sin(0.5 * theta);
//		const float_type cos = std::cos(0.5 * theta);
//
//		const quaternion Q(sin * axis[GK::X], sin * axis[GK::Y],
//				sin * axis[GK::Z], cos);
//
//		const typename vector_traits<_Vector>::value_type unit =
//				typename vector_traits<_Vector>::value_type(GK_FLOAT_ONE);
//
//		const typename vector_type<float_type, GK::GK_3D>::type u = v / unit;
//		const quaternion P(u, quaternion::value_type(GK_FLOAT_ZERO));
//
////		const quaternion R = conj(Q) * P * Q;
//		const quaternion R = Q * P * conj(Q);
//
//		_Vector r;
//		r[GK::X] = R[quaternion::X] * unit;
//		r[GK::Y] = R[quaternion::Y] * unit;
//		r[GK::Z] = R[quaternion::Z] * unit;
//
//		return r;
//	}
//};

}// namespace gk

#if defined(GK_EIGEN_ROWVECTOR) || defined(GK_EIGEN_COLUMNVECTOR)
#include "eigen/eigen_vector.h"
#endif

#endif /* GKVECTOR_H_ */
