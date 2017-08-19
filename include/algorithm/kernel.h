/*
 * kernel.h
 *
 *  Created on: 2016/02/16
 *      Author: makitaku
 */

#ifndef INCLUDE_ALGORITHM_KERNEL_H_
#define INCLUDE_ALGORITHM_KERNEL_H_

#include <utility>
#include <map>

#include "../gkvector.h"
#include "../gkaabb.h"

namespace gk {

template<typename Vector>
struct closer {
	const Vector first;
	const Vector second;

	closer(const Vector& a, const Vector& b) :
			first(a), second(b) {
	}

	Vector operator()(const Vector& v) {
		const Vector p = this->first - v;
		const Vector q = this->second - v;

		return (dot(p, p) < dot(q, q)) ? p : q;
	}

private:
	closer();
};

template<typename Vector>
closer<Vector> make_closer(const Vector& a, const Vector& b) {
	return closer<Vector>(a, b);
}

template<typename Vector>
struct futher {
	const Vector first;
	const Vector second;

	futher(const Vector& a, const Vector& b) :
			first(a), second(b) {
	}

	Vector operator()(const Vector& v) {
		const Vector p = this->first - v;
		const Vector q = this->second - v;

		return (dot(p, p) > dot(q, q)) ? p : q;
	}

private:
	futher();
};

template<typename Vector>
futher<Vector> make_futher(const Vector& a, const Vector& b) {
	return futher<Vector>(a, b);
}

/**
 * @brief Algorithm namespace.
 */
namespace alg {

/**
 * @brief Computes a position vector between 2 position vectors at
 * a specified ratio value.
 * @param first
 * @param second
 * @param ratio
 * @return
 */
template<typename Vector>
Vector between_positions(const Vector& first, const Vector& second,
		gkfloat ratio) {
	return first + ratio * (second - first);
}

/*
 * Nearest Computation Algorithms
 */

/**
 * @brief Computes a displacement vector from a point to a line.
 * @param reference Reference of a line.
 * @param u Direction of a line.
 * @param v Position vector.
 * @return
 */
template<typename Vector>
Vector nearest_to_line(const Vector& reference,
		const direction<vector_traits<Vector>::Dimension>& u, const Vector& v) {
	const Vector r = v - reference;
	return dot(r, u) * u - r;
}

/**
 * @brief Computes a displacement vector from a point to a plane.
 * @param reference
 * @param n
 * @param v
 * @return
 */
template<typename Vector>
Vector nearest_to_plane(const Vector& reference,
		const direction<vector_traits<Vector>::Dimension>& n, const Vector& v) {

	const Vector r = reference - v;

	return dot(r, n) * n;
}

/**
 * @brief Computes each position vector to be nearest between 2 lines
 * from each reference. This function returns not as a pair of displacement
 * vectors but as a pair of the position vectors.
 *
 * @param reference1
 * @param direction1
 * @param reference2
 * @param direction2
 * @return
 */
template<typename Vector>
std::pair<Vector, Vector> nearest_between_lines(const Vector& reference1,
		const direction<vector_traits<Vector>::Dimension>& direction1,
		const Vector& reference2,
		const direction<vector_traits<Vector>::Dimension>& direction2) {

	typedef typename vector_traits<Vector>::value_type length;

	const gkfloat alpha = dot(direction1, direction2);
	const gkfloat beta = GK_FLOAT_ONE - alpha * alpha;

	const Vector r = reference1 - reference2;

	if (beta == GK_FLOAT_ZERO) {
		/* parallel */
		return std::make_pair(reference1 + length(GK_FLOAT_ZERO) * direction1,
				reference2 + dot(r, direction2) * direction2);

	} else {
		/* no parallel */
		const length s = (alpha * dot(r, direction2) - dot(r, direction1))
				/ beta;
		const length t = dot(r, direction2) + alpha * s;

		return std::make_pair(reference1 + s * direction1,
				reference2 + t * direction2);
	}
}

/*
 * Intersection Algorithms
 */

/**
 * @brief Computes an intersection point of 2 lines.
 *
 * If the distance between @a result and the returned output iterator
 * is zero, 2 lines aren't intersected. Otherwise 2 lines are parallel.
 *
 * @param reference1 A reference point of first line.
 * @param direction1 A direction of first line.
 * @param reference2 A reference point of second line.
 * @param direction2 A direction of second line.
 * @param epsilon A tolerance value.
 * @param result A begin iterator of a result.
 * @return A last iterator of a result.
 */
template<typename Vector, typename Tolerance, typename OutputIterator>
OutputIterator intersect_2lines(const Vector& reference1,
		const direction<vector_traits<Vector>::Dimension>& direction1,
		const Vector& reference2,
		const direction<vector_traits<Vector>::Dimension>& direction2,
		const Tolerance& epsilon, OutputIterator result) {
	const std::pair<Vector, Vector> X = nearest_between_lines(reference1,
			direction1, reference2, direction2);

	const Vector d = X.second - X.first;
	if (dot(d, d) < epsilon * epsilon) {
		*result = X.first + 0.5 * d;
		++result;
		return result;
	} else {
		return result;
	}
}

/**
 * @brief Computes an intersection of 2 segments.
 *
 * @param a_start
 * @param a_end
 * @param b_start
 * @param b_end
 * @param epsilon
 * @param result
 * @return
 */
template<typename Vector, typename Tolerance, typename OutputIterator>
OutputIterator intersect_2segments(const Vector& a_start, const Vector& a_end,
		const Vector& b_start, const Vector& b_end, const Tolerance& epsilon,
		OutputIterator result) {

	std::vector<Vector> X;
	intersect_2lines(a_start, normalize(a_end - a_start), b_start,
			normalize(b_end - b_start), epsilon, std::inserter(X, X.begin()));

	if (X.empty()) {
		return result;
	}

	if (std::signbit(
			dot(X.front() - a_start, X.front() - a_end)
					* dot(X.front() - b_start, X.front() - b_end))) {
		*result = X.front();
		++result;
		return result;
	} else {
		return result;
	}
}

/**
 * @brief Computes an intersection of 2 planes.
 *
 * @param reference1 A reference of first plane.
 * @param normal1 A normal vector of first plane.
 * @param reference2 A reference of second plane.
 * @param normal2 A normal vector of second plane.
 * @param epsilon A tolerance value.
 * @param result
 * @return
 */
template<typename Vector, typename Tolerance, typename OutputIterator>
OutputIterator intersect_2planes(const Vector& reference1,
		const direction<vector_traits<Vector>::Dimension>& normal1,
		const Vector& reference2,
		const direction<vector_traits<Vector>::Dimension>& normal2,
		const Tolerance& epsilon, OutputIterator result) {

	const size_t Dimension = vector_traits<Vector>::Dimension;

	if (std::fabs(dot(normal1, normal2)) == gkfloat(GK_FLOAT_ONE)) {
		return result;
	}

	const Vector a = nearest_to_plane(reference1, normal1, reference2);
	const Vector b = nearest_to_plane(reference2, normal2, reference1);

	const direction<Dimension> u = normal_direction(normal1, normal2);

	const Vector r = 0.5 * (a + b);
	typedef typename vector_traits<Vector>::value_type value_type;
	const value_type unit = value_type(GK_FLOAT_ONE);
	*result = std::iterator_traits<OutputIterator>::value_type(r, r + unit * u);
	++result;
	return result;
}

/**
 * @brief Computes an intersection of a line and a segment.
 * @param reference
 * @param u
 * @param segment_start
 * @param segment_end
 * @param epsilon
 * @param result
 * @return
 */
template<typename Vector, size_t Dimension, typename Tolerance,
		typename OutputIterator>
OutputIterator intersect_line_segment(const Vector& reference,
		const direction<Dimension>& u, const Vector& segment_start,
		const Vector& segment_end, const Tolerance& epsilon,
		OutputIterator result) {

	const Vector r = segment_end - segment_start;
	const direction<Dimension> v = normalize(r);
	OutputIterator last = intersect_2lines(reference, u, segment_start, v,
			epsilon, result);

	const size_t NoIntersection = 0;
	const size_t OneIntersection = 1;
	switch (std::distance(result, last)) {
	case NoIntersection:
		return result;

	case OneIntersection:
		typedef typename vector_traits<Vector>::value_type value_type;
		const value_type t = dot(*result - segment_start, v);
		const value_type L = dot(r, v);

		if (-epsilon < t && t < L + epsilon) {
			return last;
		} else {
			return result;
		}

	default:
		return result;
	}
}

/**
 * @todo Implementation.
 * @brief Computes an intersection of a line and a plane.
 *
 * @tparam Vector A type of a vector.
 * @tparam Dimension A dimension number.
 * @tparam Tolerance A type of a tolerance value.
 * @tparam OutputIterator A type of an output iterator.
 *
 * @param line_reference A reference of the line.
 * @param direction A direction of the line.
 * @param plane_referece A reference of the plane.
 * @param normal A normal vector of the plane.
 * @param epsilon A tolerance value.
 * @param result
 * @return
 */
template<typename Vector, size_t Dimension, typename Tolerance,
		typename OutputIterator>
OutputIterator intersect_line_plane(const Vector& line_reference,
		const direction<Dimension>& line_direction,
		const Vector& plane_referece, const direction<Dimension>& normal,
		const Tolerance& epsilon, OutputIterator result) {

	return result;
}

/**
 * @brief Computes an intersection of a line and a box aligned axis in
 * 2D.
 *
 * @param reference A reference of the line.
 * @param u A direction of the line.
 * @param box_a A position of the box.
 * @param box_b The other position of the box.
 * @param epsilon A tolerance value.
 * @param result An iterator of an initial position in results of this
 * function.
 *
 * @return An iterator of an end position in the results.
 */
template<typename Vector, typename Tolerance, typename OutputIterator>
OutputIterator gk_intersect_line_box(const Vector& reference,
		const direction<GK::GK_2D>& u, const Vector& box_a, const Vector& box_b,
		const Tolerance& epsilon, OutputIterator result,
		dimension_tag<GK::GK_2D>) {

	typedef typename vector_traits<Vector>::value_type value_type;
	const value_type unit = value_type(GK_FLOAT_ONE);

	const aabb<Vector> box = make_boundary(box_a, box_b);

	std::vector<Vector> min_intersections;
	std::vector<Vector> max_intersections;

	std::map<value_type, Vector> in;
	std::map<value_type, Vector> out;

	for (size_t d = 0; d < GK::GK_2D; ++d) {

		std::vector<Vector> X;
		std::vector<Vector> Y;

		intersect_2lines(reference, u, box.min(), basis<GK::GK_2D>()[d],
				epsilon, std::inserter(X, X.begin()));
		if (X.empty()) {
			continue;
		}

		intersect_2lines(reference, u, box.max(), basis<GK::GK_2D>()[d],
				epsilon, std::inserter(Y, Y.begin()));

		const Vector x = X.front() - reference;
		const std::pair<value_type, Vector> S = std::make_pair(dot(x, u),
				X.front());

		const Vector y = Y.front() - reference;
		const std::pair<value_type, Vector> T = std::make_pair(dot(y, u),
				Y.front());

		(S.first < T.first) ?
				(in.insert(S), out.insert(T)) : (out.insert(S), in.insert(T));
	}

	if (std::abs(in.rbegin()->first - out.begin()->first) < epsilon) {
		*result = 0.5 * (in.rbegin()->second + out.begin()->second);
		++result;
		return result;

	} else {
		if (in.rbegin()->first < out.begin()->first) {
			*result = in.rbegin()->second;
			++result;
			*result = out.begin()->second;
			return result;
		} else {
			return result;
		}
	}
}

template<typename Vector, typename Tolerance, typename OutputIterator>
OutputIterator gk_intersect_line_box(const Vector& reference,
		const direction<GK::GK_3D>& u, const Vector& box_a, const Vector& box_b,
		const Tolerance& epsilon, OutputIterator result,
		dimension_tag<GK::GK_3D>) {
	return result;
}

/**
 * @brief Computes a intersection of a line and a box.
 *
 * @tparam Vector A type of a vector.
 * @tparam Dimension A dimension number.
 * @tparam Tolerance A type of a tolerance value.
 * @tparam OutputIterator A type of an output iterator.
 *
 * @param reference A reference of the line.
 * @param u A direction of the line.
 * @param box_pt1 A edge position of the box.
 * @param box_pt2 The other edge position of the box.
 * @param epsilon A tolerance value.
 * @param result An initial iterator of results of this function.
 * @return A last iterator of results of this function.
 */
template<typename Vector, size_t Dimension, typename Tolerance,
		typename OutputIterator>
OutputIterator intersect_line_box(const Vector& reference,
		const direction<Dimension>& u, const Vector& box_pt1,
		const Vector& box_pt2, const Tolerance& epsilon,
		OutputIterator result) {
	return gk_intersect_line_box(reference, u, box_pt1, box_pt2, epsilon,
			result, dimension_tag<vector_traits<Vector>::Dimension>());
}

}  // namespace alg

}  // namespace gk

#endif /* INCLUDE_ALGORITHM_KERNEL_H_ */
