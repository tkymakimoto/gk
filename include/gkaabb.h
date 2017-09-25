/*
 * gkaabb.h
 *
 *  Created on: 2014/06/24
 *      Author: Takuya Makimoto
 */

#ifndef GKAABB_H_
#define GKAABB_H_

#include "gkvector.h"
#include "gkgeometry.h"

#include <typeinfo>
#include <algorithm>

namespace gk {

/**
 * @brief Axis-aligned bounding box (AABB).
 *
 * @author Takuya Makimoto
 * @date 2017/09/25
 */
template<typename _T, std::size_t _Dimension>
class aabb: public geometry<closed_surface_tag, _T, _Dimension> {
public:

	typedef geometry<closed_surface_tag, _T, _Dimension> base;

	GK_GEOMETRY_TYPEDEF(base)

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

	/**
	 * @brief Destructor.
	 */
	~aabb() {
	}

	vector_type min() const {
		return this->min_;
	}

	vector_type max() const {
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

template<typename _T, std::size_t _Dimension>
bool is_include(const aabb<_T, _Dimension>& box,
		const vector<_T, _Dimension>& v) {
	return false;
}

template<typename _T, std::size_t _Dimension>
aabb<_T, _Dimension> operator&(const aabb<_T, _Dimension>& a,
		const aabb<_T, _Dimension>& b);

template<typename _T, std::size_t _Dimension>
aabb<_T, _Dimension> operator|(const aabb<_T, _Dimension>& a,
		const aabb<_T, _Dimension>& b);

namespace inner {

template<typename _T>
bool is_intersect_impl(const aabb<_T, GK::GK_2D>& a,
		const aabb<_T, GK::GK_2D>& b, const _T& tolerance) {
	const vector<_T, GK::GK_2D> u = a.min() - b.max();
	const vector<_T, GK::GK_2D> v = b.min() - a.max();

	const bool ux_flag = u[GK::X] < tolerance;
	const bool uy_flag = u[GK::Y] < tolerance;

	const bool vx_flag = v[GK::X] < tolerance;
	const bool vy_flag = v[GK::Y] < tolerance;

	return (ux_flag & uy_flag & vx_flag & vy_flag);
}

template<typename _T>
bool is_intersect_impl(const aabb<_T, GK::GK_3D>& a,
		const aabb<_T, GK::GK_3D>& b, const _T& tolerance) {
	const vector<_T, GK::GK_3D> u = a.min() - b.max();
	const vector<_T, GK::GK_3D> v = b.min() - a.max();

	const bool ux_flag = u[GK::X] < tolerance;
	const bool uy_flag = u[GK::Y] < tolerance;
	const bool uz_flag = u[GK::Z] < tolerance;

	const bool vx_flag = v[GK::X] < tolerance;
	const bool vy_flag = v[GK::Y] < tolerance;
	const bool vz_flag = v[GK::Z] < tolerance;

	return (ux_flag & uy_flag & uz_flag & vx_flag & vy_flag & vz_flag);
}

}  // namespace inner

template<typename _T, std::size_t _Dimension>
bool is_intersect(const aabb<_T, _Dimension>& a, const aabb<_T, _Dimension>& b,
		const _T& epsilon) {
	return inner::is_intersect_impl(a, b, epsilon);
}

template<typename _T, std::size_t _Dimension, typename Geometry>
aabb<_T, _Dimension> make_aabb(const Geometry& x);

}  // namespace gk

#endif /* GKAABB_H_ */
