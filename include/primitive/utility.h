/*
 * utility.h
 *
 *  Created on: 2015/10/30
 *      Author: Takuya Makimoto
 */

#ifndef PRIMITIVE_UTILITY_H_
#define PRIMITIVE_UTILITY_H_

namespace gk {

namespace inner {
/**
 * @brief
 * @param u
 * @param v
 * @param epsilon
 * @param
 * @return
 *
 * @date 2017/06/29
 * @version
 */
template<typename Vector, typename Tolerance>
bool is_parallel(const direction<Vector>& u, const direction<Vector>& v,
		const Tolerance& epsilon, dimension_tag<GK::GK_2D>) {
	return (std::abs(u[GK::X] - v[GK::X]) < epsilon)
			& (std::abs(u[GK::Y] - v[GK::Y]) < epsilon);
}

/**
 * @brief Function to have been implemented the code to check whether
 * two directions are prallel in 3D.
 *
 * @param u
 * @param v
 * @param epsilon
 * @param
 * @return
 */
template<typename Vector, typename Tolerance>
bool is_parallel(const direction<Vector>& u, const direction<Vector>& v,
		const Tolerance& epsilon, dimension_tag<GK::GK_3D>) {
	return (std::abs(u[GK::X] - v[GK::X]) < epsilon)
			& (std::abs(u[GK::Y] - v[GK::Y]) < epsilon)
			& (std::abs(u[GK::Z] - v[GK::Z]) < epsilon);
}

}  // namespace inner

template<typename Vector,
		typename Tolerance = typename vector_traits<Vector>::value_type>
bool is_parallel(const direction<Vector>& u, const direction<Vector>& v,
		const Tolerance& epsilon = std::numeric_limits<
				typename vector_traits<Vector>::value_type>::epsilon()) {

	return inner::is_parallel(u, v, epsilon,
			dimension_tag<vector_traits<Vector>::Dimension>());
}

template<typename Vector,
		typename Tolerance = typename vector_traits<Vector>::value_type>
bool is_parallel(const Vector& u, const Vector& v, const Tolerance& epsilon) {
	return is_parallel(direction<Vector>(u), direction<Vector>(v), epsilon);
}

}  // namespace gk

#endif /* PRIMITIVE_UTILITY_H_ */
