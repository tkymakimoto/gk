/*
 * algorithm.h
 *
 *  Created on: 2015/11/30
 *      Author: makitaku
 */

#ifndef PRIMITIVE_ALGORITHM_H_
#define PRIMITIVE_ALGORITHM_H_

#include "line.h"
#include "plane.h"

namespace gk {

template<typename _T>
bool orientation(const plane<_T>& p, const direction<GK::GK_3D>& d) {
	const direction<GK::GK_3D> u = normalize(p.reference() - d);
	return !std::signbit(dot(u, p.normal()));
}

namespace inner {

template<typename Plane, typename Vector>
bool orientation(const Plane& plane, const Vector& v, plane_tag) {
	return !std::signbit(dot(v, direction_of(plane)));
}

template<typename Line, typename Vector>
typename geometry_traits<Line>::parameter nearest(const Line& l,
		const Vector& v, line_tag) {
	const direction<Vector> u = direction_of(l);

	typedef typename geometry_traits<Line>::parameter parameter;
	const Vector r = l(parameter(GK_FLOAT_ZERO)) - v;
	return dot(r, u);
}

}  // namespace inner

template<typename Plane, typename Vector>
bool orientation(const Plane& plane, const Vector& v) {
	return inner::orientation(plane, v,
			geometry_traits<Plane>::geometry_category());
}

template<typename Geometry, typename Vector>
typename geometry_traits<Geometry>::parameter nearest(const Geometry& a,
		const Vector& v) {
	return inner::nearest(a, v, geometry_traits<Geometry>::geometry_category());
}

/**
 * @brief This function calculates the nearest position vector on a line between
 * the line and a position vector.
 *
 * @param line
 * @param v
 * @return
 */
template<typename _T, std::size_t _Dimension>
vector<_T, _Dimension> nearest(const line<_T, _Dimension>& line,
		const vector<_T, _Dimension>& v) {
	const direction<_Dimension> u = line.direction();
	const vector<_T, _Dimension> r = v - line.reference();
	return dot(r, u) * u - r;
}

template<typename _T, std::size_t _Dimension>
vector<_T, _Dimension> nearest(const segment<_T, _Dimension>& segment,
		const vector<_T, _Dimension>& r) {
	return nearest(
			line<_T, _Dimension>(segment[GK::StartEdge], segment[GK::EndEdge]),
			r);
}

template<typename _T>
vector<_T, GK::GK_3D> nearest(const plane<_T>& plane,
		const vector<_T, GK::GK_3D>& r) {

	typedef vector<_T, GK::GK_3D> vector_type;
	typedef direction<GK::GK_3D> direction;
	const direction n = plane.normal();
	const vector_type p = plane.reference() - r;

	return dot(p, n) * n;
}

/**
 * @brief Computes nearest distance parameters of 2 lines.
 *
 * This function works correctly whatever the dimension number is.
 * Because the following formulas for the solution are implemented.
 *
 * @f{eqnarray*}{
 * \mathbf{l}(s) &=& \mathbf{p}_{start} + s \mathbf{u} \\
 * \mathbf{m}(t) &=& \mathbf{q}_{start} + t \mathbf{v}
 * @f}
 *
 * @param l	A line.
 * @param m The other line.
 *
 * @return Pair of the distance parameters between 2 lines.
 */
//template<typename Vector>
//std::pair<typename vector_traits<Vector>::value_type,
//		typename vector_traits<Vector>::value_type> nearest_between(
//		const line<Vector>& l, const line<Vector>& m) {
//
//	typedef typename vector_traits<Vector>::value_type length;
//	typedef direction<vector_traits<Vector>::Dimension> direction;
//
//	const direction u = l.direction();
//	const direction v = m.direction();
//
//	const gkfloat alpha = dot<direction, direction, gkfloat>(u, v);
//	const gkfloat beta = GK_FLOAT_ONE - alpha * alpha;
//
//	const Vector r = l.reference() - m.reference();
//
//	if (beta == GK_FLOAT_ZERO) {
//		/* parallel */
//		return std::make_pair(length(GK_FLOAT_ZERO), dot(r, v));
//
//	} else {
//		/* no parallel */
//		const dot<Vector, direction, length> dot;
//		const length s = (alpha * dot(r, v) - dot(r, u)) / beta;
//		const length t = dot(r, v) + alpha * s;
//
//		return std::make_pair(s, t);
//	}
//}
template<typename _T>
std::pair<_T, _T> nearest_between(const line<_T, GK::GK_3D>& a,
		const plane<_T>& b) {
	typedef _T length_type;
	typedef direction<GK::GK_3D> direction_type;

	const direction_type u = a.direction();
	const direction_type n = b.normal();

	if (dot(u, n) == GK_FLOAT_ZERO) {
		// The line and the plane are parallel.

		const length_type t = length_type(GK_FLOAT_ZERO);
		return std::make_pair(t, nearest(b, a(t)));

	} else {
		// Not parallel, otherwise intersection.

		const length_type t = dot(b.reference() - a.reference(), n) / dot(u, n);

		const vector<_T, GK::GK_3D> r = a(t) - b.reference();
		return std::make_pair(t,
				make_plane_parameter(dot(r, b.u_direction()),
						dot(r, b.v_direction())));
	}
}

template<typename _T>
std::pair<_T, _T> nearest_between(const plane<_T>& a,
		const line<_T, GK::GK_3D>& b) {
	typedef _T length_type;

	const std::pair<length_type, length_type> result = nearest_between(b, a);
	return std::make_pair(result.second, result.first);
}

template<typename Projected, typename Support>
struct project_result {
	typedef Projected type;
};

template<typename Projected, typename Support, typename OutputIterator>
OutputIterator project(const Projected& a, const Support& b,
		OutputIterator result);

template<typename _T, std::size_t _Dimension, typename OutputIterator>
OutputIterator project(const vector<_T, _Dimension>& a, const plane<_T>& b,
		OutputIterator result) {

}

template<typename Geometry1, typename Geomerty2>
struct intersect_result {
	typedef void type; // dummy definition.
};

#if __cplusplus >= 201103L
template<typename _Geometry1, typename _Geometry2>
using intersect = typename intersect_result<_Geometry1,_Geometry2>::type;
#	define INTERSECT_TYPE(GEOM1, GEOM2) \
	intersect<GEOM1, GEOM2>
#else
#	define INTERSECT_TYPE(GEOM1, GEOM2) \
	typename intersect_result<GEOM1, GEOM2>::type
#endif

template<typename Geometry1, typename Geometry2, typename Tolerance,
		typename OutputIterator>
OutputIterator intersect(const Geometry1& a, const Geometry2& b,
		const Tolerance& epsilon, OutputIterator result);

//template<typename Vector, typename Tolerance, typename OutputIterator>
//OutputIterator intersect(const line<Vector>& a, const line<Vector>& b,
//		const Tolerance& epsilon, OutputIterator result) {
//	typedef typename vector_traits<Vector>::value_type value_type;
//
//	const std::pair<value_type, value_type> _T = nearest_between(a, b);
//
//	if (norm(a(_T.first) - b(_T.second)) < epsilon) {
//		*result = 0.5 * (a(_T.first) + b(_T.second));
//		++result;
//	}
//
//	return result;
//}

template<typename _T, std::size_t _Dimension, typename Tolerance,
		typename OutputIterator>
OutputIterator intersect(const segment<_T, _Dimension>& a,
		const segment<_T, _Dimension>& b, const Tolerance& epsilon,
		OutputIterator result) {

	typedef typename geometry_traits<segment<_T, _Dimension> >::parameter parameter;

	std::pair<_T, _T> T = nearest_between(
			line<_T, _Dimension>(a[GK::StartEdge], a[GK::EndEdge]),
			line<_T, _Dimension>(b[GK::StartEdge], b[GK::EndEdge]));

	const _T La = _T(a);
	const _T Lb = _T(b);

	const _T s = T.first / La;
	const _T t = T.second / Lb;

	const std::pair<parameter, parameter> da = domain(a);
	const std::pair<parameter, parameter> db = domain(b);
	if ((s < da.first || da.second < s) || (t < db.first || db.second < t)) {
		return result;
	}

	*result = 0.5 * (a(s) + b(t));
	++result;
	return result;
}

template<typename Vector, typename Tolerance, typename OutputIterator>
OutputIterator intersect(const plane<Vector>& a, const plane<Vector>& b,
		const Tolerance& epsilon, OutputIterator result) {

	const direction<Vector> n = a.normal();
	const direction<Vector> m = b.normal();

	if (std::fabs(dot(n, m)) == direction<Vector>::value_type(GK_FLOAT_ONE)) {
		return result;
	}

	const direction<Vector> u = cross(n, m);
	const Vector r = 0.5 * a(nearest(a, b.reference()))
			+ b(nearest(b, a.reference()));
	*result = line<Vector>(r, u);
	++result;
	return result;
}

template<typename _T, std::size_t _Dimension, typename Tolerance,
		typename OutputIterator>
OutputIterator intersect(const line<_T, _Dimension>& a,
		const segment<_T, _Dimension>& b, const Tolerance& epsilon,
		OutputIterator result) {

	typedef typename geometry_traits<line<_T, _Dimension> >::parameter line_parameter;
	std::pair<line_parameter, line_parameter> _T = nearest_between(a,
			line<_T, _Dimension>(b[GK::StartEdge], b[GK::EndEdge]));

	typedef typename geometry_traits<segment<_T, _Dimension> >::parameter parameter;
	const parameter t = _T.second / curve_traits < segment<_T, _Dimension>
			> ::length_type(b);
	const std::pair<parameter, parameter> D = curve_traits
			< segment<_T, _Dimension> > ::domain(b);

	if (t < D.first || D.second < t) {
		return result;
	}

	*result = 0.5 * (a(_T.first) + b(t));
	++result;
	return result;
}

template<typename Vector, typename Tolerance, typename OutputIterator>
OutputIterator intersect(const segment<Vector>& a, const line<Vector>& b,
		const Tolerance& epsilon, OutputIterator result) {
	return intersect(b, a, epsilon, result);
}

template<typename Vector, typename Tolerance, typename OutputIterator>
OutputIterator itersect(const line<Vector>& a, const plane<Vector>& b,
		const Tolerance& epsilon, OutputIterator result) {

	if (dot(a.direction(), b.normal()) == GK_FLOAT_ZERO) {
		return result;
	}

	typedef typename geometry_traits<line<Vector> >::parameter line_parameter;
	typedef typename geometry_traits<plane<Vector> >::parameter plane_parameter;
	const std::pair<line_parameter, plane_parameter> _T = nearest_between(a, b);

	*result = 0.5 * (a(_T.first) + b(_T.second));
	++result;

	return result;
}

template<typename Vector, typename Tolerance, typename OutputIterator>
OutputIterator intersect(const plane<Vector>& a, const line<Vector>& b,
		const Tolerance& epsilon, OutputIterator result) {
	return intersect(b, a, epsilon, result);
}

template<typename Vector, typename Tolerance, typename OutputIterator>
OutputIterator intersect(const segment<Vector>& a, const plane<Vector>& b,
		const Tolerance& epsilon, OutputIterator result) {
	if (!(orientation(b, a[GK::StartEdge]) ^ orientation(b, a[GK::EndEdge]))) {
		return result;
	}

	return intersect(line<Vector>(a[GK::StartEdge], a[GK::EndEdge]), b, epsilon,
			result);
}

template<typename Vector, typename Tolerance, typename OutputIterator>
OutputIterator intersect(const plane<Vector>& a, const segment<Vector>& b,
		const Tolerance& epsilon, OutputIterator result) {
	return intersect(b, a, epsilon, result);
}

namespace inner {

template<typename Vector>
bool test_itersect(const line<Vector>& l, const aabb<Vector>& boundary,
		dimension_tag<GK::GK_2D>) {

	std::pair<line<Vector>, line<Vector> > x_lines = std::make_pair(
			line<Vector>(boundary.min()), basis()[GK::Y]);

	std::pair<line<Vector>, line<Vector> > y_lines;
}

template<typename Vector>
bool test_itersect(const line<Vector>& l, const aabb<Vector>& boundary,
		dimension_tag<GK::GK_3D>) {

}

}  // namespace inner

template<typename Vector, typename Tolerance>
bool test_intersect(const line<Vector>& l, const aabb<Vector>& boundary) {
	const basis<Vector> basis;
	const Vector min = boundary.min();

	typedef typename vector_traits<Vector>::value_type value_type;

//	typename line<vector>::parameter t_min;
	value_type t_min;
	for (size_t i = 0; i < vector_traits<Vector>::Dimension; ++i) {
		const line<Vector> min_line(boundary.min(), basis[i]);
		const std::pair<value_type, value_type> t = nearest_between(l,
				min_line);
		t_min = std::max(t_min, t);
	}

}

} // namespace gk

#endif /* PRIMITIVE_ALGORITHM_H_ */
