/*
 * triangle.h
 *
 *  Created on: 2015/03/06
 *      Author: Takuya Makimoto
 */

#ifndef PRIMITIVE_TRIANGLE_H_
#define PRIMITIVE_TRIANGLE_H_

#include "../gkvector.h"
#include "../gkgeometry.h"

namespace gk {

struct triangle_tag: public plane_tag {
};

GK_GEOMETRY_BASE_TEMPLATE_CLASS(triangle_tag)

/**
 * @brief This class represents a triangle.
 *
 * @tparam Vector Type of a vector in a linear space.
 *
 * @author Takuya Makimoto
 * @date 2017/06/29
 *
 * @version 0.X
 */
template<typename _T, std::size_t _Dimension>
struct triangle: public geometry<triangle_tag, _T, _Dimension,
		vector<_T, GK::GK_2D> > {
public:

	typedef geometry<triangle_tag, _T, _Dimension, vector<_T, GK::GK_2D> > base;

	GK_GEOMETRY_TYPEDEF(base);

	enum {
		First, Second, Third, EdgePointSize,
	};

public:
	/**
	 * @brief Default constructor.
	 */
	triangle() : x_() {
	}

	/**
	 * @brief Copy constructor.
	 * @param other
	 */
	triangle(const triangle& other) : x_() {
		std::copy(other.x_, other.x_ + EdgePointSize, this->x_);
	}

	/**
	 * @brief Destructor.
	 */
	~triangle() {
	}

	void reverse() {
		std::swap(this->x_[Second], this->x_[Third]);
	}

	/**
	 * @brief Computes u-component.
	 * @return The u-component vector..
	 */
	vector_type u() const {
		return this->u_();
	}

	/**
	 * @brief Computes v-component.
	 * @return The v-component vector.
	 */
	vector_type v() const {
		return this->v_();
	}

	const vector_type& operator[](std::size_t component) const {
		return this->x_[component];
	}

	vector_type& operator[](std::size_t component) {
		return this->x_[component];
	}

	/**
	 * @brief
	 * @param s $\in [0:1]$
	 * @param t $\in [0:1]$
	 * @return
	 */
	template<typename Parameter>
	vector_type operator()(const Parameter& s, const Parameter& t) const {
		return this->x_[First] + s * this->u_() + t * this->v_();
	}

	/**
	 *
	 * @param rhs A RHS triangle object.
	 * @return
	 */
	triangle& operator=(const triangle& rhs) {
		if (&rhs == this) {
			return *this;
		}

		std::copy(rhs.x_, rhs.x_ + EdgePointSize, this->x_);
		return *this;
	}

private:
	vector_type x_[EdgePointSize];

private:
	vector_type u_() const {
		return this->x_[Second] - this->x_[First];
	}

	vector_type v_() const {
		return this->x_[Third] - this->x_[First];
	}

};

/**
 * @brief Computes a direction of a triangle. This function can run in 3D.
 * @param a
 * @return
 *
 */
template<typename _T>
direction<GK::GK_3D> direction_of(const triangle<_T, GK::GK_3D>& a) {
	return direction<GK::GK_3D>(cross(a.u(), a.v()));
}

namespace inner {

/**
 * @brief Calculates the area of the triangle made by three points in 2D.
 *
 * @param first
 * @param second
 * @param third
 * @param
 * @return
 *
 * @see area_of_triangle(const Vector&, const Vector&, const Vector&, dimension_category<GK::GK_3D>)
 */
template<typename _T, typename AreaValue>
AreaValue area_of_triangle(const vector<_T, GK::GK_2D>& first,
		const vector<_T, GK::GK_2D>& second, const vector<_T, GK::GK_2D>& third,
		dimension_tag<GK::GK_2D>) {

}

/**
 *
 * @param first
 * @param second
 * @param third
 * @param
 * @return
 */
template<typename _T, typename AreaValue>
AreaValue area_of_triangle(const vector<_T, GK::GK_3D>& first,
		const vector<_T, GK::GK_3D>& second, const vector<_T, GK::GK_3D>& third,
		dimension_tag<GK::GK_3D>) {

}

}  // namespace inner

template<typename _T, typename AreaValue>
struct area_function {
	AreaValue operator()(const _T&) const;
};

template<typename _T, std::size_t _Dimension, typename AreaValue>
struct area_function<triangle<_T, _Dimension>, AreaValue> {
	AreaValue operator()(const triangle<_T, _Dimension>& a) const {
//		const dot<Vector, Vector, AreaValue> dot;
		const AreaValue dot_u = dot(a.u(), a.u());
		const AreaValue dot_v = dot(a.v(), a.v());
		const AreaValue dot_uv = dot(a.u(), a.v());
		return 0.5 * std::sqrt(dot_u * dot_v - dot_uv * dot_uv);
	}
};

template<typename _T, std::size_t _Dimension>
product<_T, _T> area(const triangle<_T, _Dimension>& a) {

}

}  // namespace gk

#endif /* PRIMITIVE_TRIANGLE_H_ */
