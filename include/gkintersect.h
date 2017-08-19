/*
 * gkintersect.h
 *
 *  Created on: 2015/10/06
 *      Author: makitaku
 */

#ifndef GKINTERSECT_H_
#define GKINTERSECT_H_

#include "gkgeometry.h"
#include "algorithm/kernel.h"

namespace gk {

/**
 * @brief Result type of intersection.
 *
 * @tparam Geometry1
 * @tparam Geometry2
 */
template<typename Geometry1, typename Geometry2>
struct intersect_result {
	typedef void value_type;
};

namespace inner {

/**
 * @brief This declaration is a kernel function to compute intersections
 * @a Geometry1 and @a Geometry2.
 *
 * @tparam Geometry1 Type of first geometry.
 * @tparam Geometry2 Type of second geometry.
 * @tparam Tolerance Type of a tolerance value.
 * @tparam OutputIterator Type of an output iterator of results.
 * @tparam Geometry1Category Type of the category of first geometry.
 * @tparam Geometry2Category Type of the category of second geometry.
 *
 * @param a First geometry to be intersected.
 * @param b Second geometry to be intersected.
 * @param epsilon A tolerance value.
 * @param result
 * @param
 * @param
 * @return
 */
template<typename Geometry1, typename Geometry2, typename Tolerance,
		typename OutputIterator, typename Geometry1Category,
		typename Geometry2Category>
OutputIterator intersect_kernel(const Geometry1& a, const Geometry2& b,
		const Tolerance& epsilon, OutputIterator result, Geometry1Category,
		Geometry2Category);

/**
 * @brief Computes an intersection of 2 lines.
 * @param l
 * @param m
 * @param epsilon
 * @param result
 * @param
 * @param
 * @return
 */
template<typename Line, typename Tolerance, typename OutputIterator>
OutputIterator intersect_kernel(const Line& l, const Line& m,
		const Tolerance& epsilon, OutputIterator result, line_tag, line_tag) {

	typedef typename geometry_traits<Line>::vector_type vector_type;
	typedef typename vector_traits<vector_type>::value_type value_type;

	const size_t Dimension = vector_traits<vector_type>::Dimension;

	const value_type Zero = value_type(GK_FLOAT_ZERO);
	const vector_type ref1 = l(Zero);
	const direction<Dimension> u = direction_of(l);
	const vector_type ref2 = m(Zero);
	const direction<Dimension> v = direction_of(m);

	return alg::intersect_2lines(ref1, u, ref2, v, epsilon, result);
}

/**
 * @brief Computes an intersection of 2 planes.
 *
 * @param a
 * @param b
 * @param epsilon
 * @param result
 * @param
 * @param
 * @return
 */
template<typename Plane, typename Tolerance, typename OutputIterator>
OutputIterator intersect_kernel(const Plane& a, const Plane& b,
		const Tolerance& epsilon, OutputIterator result, plane_tag, plane_tag) {

	typedef typename geometry_traits<Plane>::vector_type vector_type;
	typedef typename vector_traits<vector_type>::value_type value_type;

	const size_t Dimension = geometry_traits<Plane>::Dimension;
	const value_type Zero(GK_FLOAT_ZERO);

	const vector_type ref1 = a(Zero, Zero);
	const direction<Dimension> n = direction_of(a);
	const vector_type ref2 = b(Zero, Zero);
	const direction<Dimension> m = direction_of(b);
	return alg::intersect_2planes(ref1, n, ref2, m, epsilon, result);
}

} // namespace inner

/**
 * @brief This is a template function to intersect 2 geometries.
 *
 * @param a First geometry.
 * @param b Second geometry.
 * @param epsilon A tolerance value.
 * @param result
 * @return
 */
template<typename Geometry1, typename Geometry2, typename Tolerance,
		typename OutputIterator>
OutputIterator intersect(const Geometry1& a, const Geometry2& b,
		const Tolerance& epsilon, OutputIterator result) {
	return inner::intersect_kernel(a, b, epsilon, result,
			typename geometry_traits<Geometry1>::geometry_category(),
			typename geometry_traits<Geometry2>::geometry_category());
}

}  // namespace gk

#endif /* GKINTERSECT_H_ */
