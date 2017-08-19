/*
 * gkaabb.h
 *
 *  Created on: 2014/06/24
 *      Author: Takuya Makimoto
 */

#ifndef GKAABB_H_
#define GKAABB_H_

#include "gkvector.h"

#include <typeinfo>
#include <algorithm>

namespace gk {

/**
 * @brief Axis-aligned bounding box (AABB).
 *
 * @author Takuya Makimoto
 * @date 2014
 */
template<typename Vector>
class aabb {
public:
	typedef Vector vector_type;
	static const std::size_t Dimension = vector_traits<Vector>::Dimension;

public:
	/**
	 * @brief Default constructor.
	 */
	aabb() {
	}

	/**
	 * @brief Copy constructor.
	 * @param other
	 */
	aabb(const aabb& other) :
			min_(other.min_), max_(other.max_) {
	}

	aabb(const vector_type& u, const vector_type& v) :
			min_(), max_() {
		this->set_(u, v, dimension_tag<Dimension>());
	}

	template<class InputIterator>
	aabb(InputIterator first, InputIterator last) :
			min_(), max_() {
		typedef InputIterator iterator;
//		typedef std::iterator_traits<InputIterator> traits;

		{
			iterator p = first;
			this->min_ = *p;
			this->max_ = *p;

			++p;

			while (p != last) {
				for (size_t i = 0; i < Dimension; ++i) {
					this->min_[i] = std::min(this->min_[i], (*p)[i]);
					this->max_[i] = std::max(this->max_[i], (*p)[i]);
				}
				++p;
			}
		}
	}

	/**
	 * @brief Destructor.
	 */
	~aabb() {
	}

	const vector_type& min() const {
		return this->min_;
	}

	const vector_type& max() const {
		return this->max_;
	}

	aabb& operator=(const aabb& rhs) {
		if (&rhs == this) {
			return *this;
		}

		this->min_ = rhs.min_;
		this->max_ = rhs.max_;

		return *this;
	}

private:
	vector_type min_;
	vector_type max_;

private:

	template<std::size_t Dimension>
	void set_(const vector_type& u, const vector_type& v,
			dimension_tag<Dimension>) {
		for (std::size_t i = 0; i < Dimension; ++i) {
			(u[i] < v[i]) ?
					this->min_[i] = u[i], this->max_[i] = v[i] : this->min_[i] =
							v[i], this->max_[i] = u[i];
		}
	}

	void set_(const vector_type& u, const vector_type& v,
			dimension_tag<GK::GK_2D>) {
		(u[GK::X] < v[GK::X]) ?
				(this->min_[GK::X] = u[GK::X], this->max_[GK::X] = v[GK::X]) :
				(this->min_[GK::X] = v[GK::X], this->max_[GK::X] = u[GK::X]);

		(u[GK::Y] < v[GK::Y]) ?
				(this->min_[GK::Y] = u[GK::Y], this->max_[GK::Y] = v[GK::Y]) :
				(this->min_[GK::Y] = v[GK::Y], this->max_[GK::Y] = u[GK::Y]);
	}

	void set_(const vector_type& u, const vector_type& v,
			dimension_tag<GK::GK_3D>) {
		set_(u, v, dimension_tag<GK::GK_2D>());

		(u[GK::Z] < v[GK::Z]) ?
				this->min_[GK::Z] = u[GK::Z], this->max_[GK::Z] = v[GK::Z] :
				this->min_[GK::Z] = v[GK::Z], this->max_[GK::Z] = u[GK::Z];
	}
};

template<typename Vector>
bool is_include(const aabb<Vector>& box, const Vector& v) {
	return false;
}

template<typename Vector>
aabb<Vector> operator&(const aabb<Vector>& a, const aabb<Vector>& b);

template<typename Vector>
aabb<Vector> operator|(const aabb<Vector>& a, const aabb<Vector>& b);

namespace inner {

template<typename Vector>
bool is_intersect_impl(const aabb<Vector>& a, const aabb<Vector>& b,
		const typename vector_traits<Vector>::value_type& tolerance,
		dimension_tag<GK::GK_2D>) {
	const Vector u = a.min() - b.max();
	const Vector v = b.min() - a.max();

	const bool ux_flag = u[GK::X] < tolerance;
	const bool uy_flag = u[GK::Y] < tolerance;

	const bool vx_flag = v[GK::X] < tolerance;
	const bool vy_flag = v[GK::Y] < tolerance;

	return (ux_flag & uy_flag & vx_flag & vy_flag);
}

template<typename Vector>
bool is_intersect_impl(const aabb<Vector>& a, const aabb<Vector>& b,
		const typename vector_traits<Vector>::value_type& tolerance,
		dimension_tag<GK::GK_3D>) {
	const Vector u = a.min() - b.max();
	const Vector v = b.min() - a.max();

	const bool ux_flag = u[GK::X] < tolerance;
	const bool uy_flag = u[GK::Y] < tolerance;
	const bool uz_flag = u[GK::Z] < tolerance;

	const bool vx_flag = v[GK::X] < tolerance;
	const bool vy_flag = v[GK::Y] < tolerance;
	const bool vz_flag = v[GK::Z] < tolerance;

	return (ux_flag & uy_flag & uz_flag & vx_flag & vy_flag & vz_flag);
}

}  // namespace inner

template<typename Vector, typename Tolerance>
bool is_intersect(const aabb<Vector>& a, const aabb<Vector>& b,
		const Tolerance& epsilon) {
	return inner::is_intersect_impl(a, b, epsilon,
			dimension_tag<vector_traits<Vector>::Dimension>());
}

template<typename Geometry>
aabb<typename geometry_traits<Geometry>::vector_type> make_aabb(
		const Geometry& x);

template<typename Vector>
aabb<Vector> make_boundary(const Vector& a, const Vector& b) {
	return aabb<Vector>(a, b);
}


}  // namespace gk

//template<GK_Dimension DimensionNumber>
//bool is_crossover(const aabb<DimensionNumber>& a, const aabb<DimensionNumber>& b,
//        const gktolerance& epsilon) {
//    //	const gktolerance& epsilon = gkoption::Tolerance();
//
//    const gkvector<DimensionNumber> p = a.min() - b.max();
//    const gkvector<DimensionNumber> q = b.min() - a.max();
//
//    const bool px_flag = p.x() < epsilon;
//    const bool py_flag = p.y() < epsilon;
//    const bool pz_flag = p.z() < epsilon;
//
//    const bool qx_flag = q.x() < epsilon;
//    const bool qy_flag = q.y() < epsilon;
//    const bool qz_flag = q.z() < epsilon;
//
//    return (px_flag & py_flag & pz_flag & qx_flag & qy_flag & qz_flag);
//}

//template<GK_Dimension D>
//bool is_crossover(const aabb<D>& a, const aabb<D>& b) {
//    return is_crossover(a, b, gkoption::Tolerance());
//}

//bool is_crossover(const aabb<GK_2D>& a, const aabb<GK_2D>& b);
//bool is_crossover(const aabb<GK_3D>& a, const aabb<GK_3D>& b);
//
//bool is_inclusion(const aabb<GK_2D>& aabb, const gkvector<GK_2D>& v);
//bool is_inclusion(const aabb<GK_3D>& aabb, const gkvector<GK_3D>& v);

///**
// * @brief Constant AABB objects.
// */
//template<GK_Dimension D>
//class const_gkaabb {
//public:
//	static const aabb<D>& Invalid() {
//		static const aabb<D> x;
//		return x;
//	}
//};

#endif /* GKAABB_H_ */
