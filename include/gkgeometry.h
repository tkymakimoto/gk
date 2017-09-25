/*
 * gkgeometry.h
 *
 *  Created on: 2015/06/01
 *      Author: tmakimoto
 */

#ifndef GKGEOMETRY_H_
#define GKGEOMETRY_H_

#include "gkdef.h"
#include "gkvector.h"

/**
 * @defgroup geometry_tags Geometry Tags
 *
 */

namespace gk {

/*
 * Classification of traits of a curve.
 */

/**
 * @brief Direction category.
 */
struct direction_tag {
};

/**
 * @brief Point category.
 */
struct point_tag {
};

/**
 * @brief Curve category.
 */
struct curve_tag {
};

/**
 * @brief Line category.
 */
struct line_tag: public curve_tag, public direction_tag {
};

/**
 * @brief Ray category.
 */
struct ray_tag: public line_tag {
};

/**
 * @brief Segment category.
 */
struct segment_tag: public ray_tag {
};

/**
 * @brief Circle category.
 */
struct circle_tag: public curve_tag, public direction_tag {
};

/**
 * @brief Free curve category.
 */
struct free_curve_tag: public curve_tag {
};

/*
 * Classification of traits of a surface.
 */

/**
 * @brief Surface category.
 */
struct surface_tag {
};

/**
 * @brief Plane category.
 */
struct plane_tag: public surface_tag, public direction_tag {
};

/**
 * @brief Closed surface category.
 */
struct closed_surface_tag: public surface_tag {
};

/**
 * @brief Sphere category.
 */
struct sphere_tag: public closed_surface_tag {
};

#if __cplusplus >= 201103L
template<typename _Category, typename _T, std::size_t _Dimension,
		typename _Parameter = void>
struct geometry {
	using category = _Category;
	using value_type = _T;
	using parameter = _Parameter;
	using vector_type = vector<_T, _Dimension>;
private:
	geometry();

};

#else
template<typename _Category, typename _T, std::size_t _Dimension,
typename _Parameter = void>
struct geometry {
	typedef _Category category;
	typedef _T value_type;
	typedef _Parameter parameter;
	typedef vector<_T, _Dimension> vector_type;
private:
	geometry();

};

#endif

#if __cplusplus >= 201103L
#define GK_GEOMETRY_BASE_TEMPLATE_CLASS(CATEGORY) \
	template<typename _T, std::size_t _Dimension, typename _Parameter> \
	struct geometry<CATEGORY, _T, _Dimension, _Parameter> {\
		using category = CATEGORY;\
		using value_type = _T;\
		using parameter = _Parameter;\
		using vector_type = vector<_T, _Dimension>;\
	};

#else
#define GK_GEOMETRY_BASE_TEMPLATE_CLASS(CATEGORY) \
	template<typename _T, std::size_t _Dimension, typename _Parameter = void> \
	struct geometry<CATEGORY, _T, _Dimension, _Parameter> { \
		typedef CATEGORY category; \
		typedef _T value_type; \
		typedef _Parameter parameter; \
		typedef vector<_T, _Dimension> vector_type; \
	};

#endif

GK_GEOMETRY_BASE_TEMPLATE_CLASS(point_tag)
GK_GEOMETRY_BASE_TEMPLATE_CLASS(direction_tag)
GK_GEOMETRY_BASE_TEMPLATE_CLASS(line_tag)
GK_GEOMETRY_BASE_TEMPLATE_CLASS(circle_tag)
GK_GEOMETRY_BASE_TEMPLATE_CLASS(free_curve_tag)
GK_GEOMETRY_BASE_TEMPLATE_CLASS(plane_tag)
GK_GEOMETRY_BASE_TEMPLATE_CLASS(surface_tag)
GK_GEOMETRY_BASE_TEMPLATE_CLASS(closed_surface_tag)
GK_GEOMETRY_BASE_TEMPLATE_CLASS(sphere_tag)

/**
 * @brief Traits of a geometry.
 *
 */
template<typename _Geometry>
struct geometry_traits {
	typedef typename _Geometry::category category;
	typedef typename _Geometry::value_type value_type;
	typedef typename _Geometry::parameter parameter;
	static const std::size_t Dimension = _Geometry::Dimension;
	typedef typename _Geometry::vector_type vector_type;
};

#if __cplusplus >= 201103L
#define GK_GEOMETRY_TYPEDEF(GEOMETRY)\
		using traits_type = geometry_traits<GEOMETRY>;\
		using category = typename traits_type::category;\
		using value_type = typename traits_type::value_type;\
		using parameter = typename traits_type::parameter;\
		using vector_type = typename traits_type::vector_type;
#else
#define GK_GEOMETRY_TYPEDEF(GEOMETRY)\
		typedef geometry_traits<GEOMETRY> traits_type;\
		typedef typename traits_type::category category;\
		typedef typename traits_type::value_type value_type;\
		typedef typename traits_type::parameter parameter;\
		typedef typename traits_type::vector_type vector_type;
#endif

/**
 * @brief Outputs a direction of a geometry, @a a.
 *
 * @tparam _T The type of the geometry.
 * @param a
 * @return
 */
template<typename _Geometry>
direction<geometry_traits<_Geometry>::Dimension> direction_of(
		const _Geometry& a);

} // namespace gk

#endif /* GKGEOMETRY_H_ */
