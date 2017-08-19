/*
 * gknearest.h
 *
 *  Created on: 2016/01/22
 *      Author: makitaku
 */

#ifndef GKNEAREST_H_
#define GKNEAREST_H_

#include "gkvector.h"

namespace gk {

namespace inner {

template<typename _Geometry, typename _T, std::size_t _Dimension,
		typename _GeometryCategory>
typename geometry_traits<_Geometry>::parameter nearest(
		const _Geometry& geometry, const vector<_T, _Dimension>& v,
		_GeometryCategory);

template<typename _Line, typename _T, std::size_t _Dimension>
typename geometry_traits<_Line>::parameter nearest(const _Line& line,
		const vector<_T, _Dimension>& v, line_tag) {
	typedef typename vector_traits<vector<_T, _Dimension>>::value_type value_type;
	return nearest_to_line(line(value_type(GK_FLOAT_ZERO)), direction_of(line),
			v);
}

template<typename Plane, typename _T, std::size_t _Dimension>
typename geometry_traits<Plane>::parameter nearest(const Plane& plane,
		const vector<_T, _Dimension>& v, plane_tag) {
	typedef direction<_Dimension> direction;

	const direction n = direction_of(plane);
	const vector<_T, _Dimension> r = plane(_T(GK_FLOAT_ZERO), _T(GK_FLOAT_ZERO))
			- v;

	return v + dot(r, n) * n;
}

}  // namespace inner

/**
 * @brief Computes a vector being from a position vector @a v to a nearest position
 * on a geometry @a g.
 * @param g
 * @param v
 * @return
 */
template<typename _Geometry, typename _T, std::size_t _Dimension>
typename geometry_traits<_Geometry>::parameter nearest(
		const _Geometry& geometry, const vector<_T, _Dimension>& v) {
	return inner::nearest(geometry, v,
			typename geometry_traits<_Geometry>::geometry_category());
}

namespace inner {

template<typename _Geometry1, typename _Geometry2, typename _Geometry1Category,
		typename _Geometry2Category>
std::pair<typename geometry_traits<_Geometry1>::parameter,
		typename geometry_traits<_Geometry2>::parameter> nearest_between(
		const _Geometry1& a, const _Geometry2& b, _Geometry1Category,
		_Geometry2Category);

template<typename _Line1, typename _Line2>
std::pair<typename geometry_traits<_Line1>::parameter,
		typename geometry_traits<_Line2>::parameter> nearest_between(
		const _Line1& l, const _Line2& m, line_tag, line_tag) {
	typedef geometry_traits<_Line1> traits1;
	typedef geometry_traits<_Line2> traits2;

	typedef typename traits1::parameter length1;
	typedef typename traits2::parameter length2;

	return std::pair<length1, length2>();
}

}  // namespace inner

/**
 * @brief Computes nearest positions of each geometry.
 * @param a
 * @param b
 * @return
 */
template<typename Geometry1, typename Geometry2>
std::pair<typename geometry_traits<Geometry1>::vector_type,
		typename geometry_traits<Geometry2>::vector_type> nearest_between(
		const Geometry1& a, const Geometry2& b) {
	return inner::nearest_between(a, b,
			typename geometry_traits<Geometry1>::geometry_category(),
			typename geometry_traits<Geometry2>::geometry_category());
}

}  // namespace gk

#endif /* GKNEAREST_H_ */
