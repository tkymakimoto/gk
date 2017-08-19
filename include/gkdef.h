/*
 * gkdef.h
 *
 *  Created on: 2013/04/09
 *      Author: Takuya Makimoto
 */

/**
 * @file gkdef.h
 * @brief
 *
 * @date 2017/08/18
 */

#ifndef GKDEF_H_
#define GKDEF_H_

#include <cstdlib>
#include <stdint.h>
#include <limits>

#include "config/gkconfig.h"

typedef bool gkselection;

typedef bool gkminmax;
const bool GK_Min = false;
const bool GK_Max = true;

#define GK_FLOAT_ZERO 0.0
#define GK_FLOAT_ONE 1.0
#define GK_FLOAT_NEGATIVE_ONE -1.0

namespace gk {

typedef std::size_t size_t;

/*
 * Floating Value Type
 */
#ifdef GK_SIZEOF_FLOAT
#	if GK_SIZEOF_FLOAT == __SIZEOF_FLOAT__
#		define GK_FLOAT_TYPE float
#	elif GK_SIZEOF_FLOAT == __SIZEOF_DOUBLE__
#		define GK_FLOAT_TYPE double
#	else
#		error "GK_FLOAT_TYPE" must set 4 or 8.
#	endif
#else
#	define GK_SIZEOF_FLOAT 8
#		define GK_FLOAT_TYPE double
#endif
typedef GK_FLOAT_TYPE float_type;
#undef GK_FLOAT_TYPE

template<std::size_t _N>
struct number_tag {
	static const std::size_t Value = _N;
};

template<std::size_t _Dimension>
struct dimension_tag: public number_tag<_Dimension> {
	static const std::size_t Size = _Dimension;
};

template<typename _Scalar>
struct scalar_traits {
	static const std::size_t Precision = 0;
};

template<std::size_t _Precision>
struct float_traits {
	typedef void value_type;
};

template<>
struct float_traits<__SIZEOF_FLOAT__> {
	typedef float value_type;
};

template<>
struct float_traits<__SIZEOF_DOUBLE__> {
	typedef double value_type;
};

template<>
struct float_traits<__SIZEOF_LONG_DOUBLE__> {
	typedef long double value_type;
};

/**
 * @brief Struct of constant objects.
 *
 */
struct GK {

	/**
	 * @brief Enum about space dimensions.
	 */
	typedef enum {
		GK_NonDim = 0, ///< Non-dimension.
		GK_1D = 1, ///< 1 dimension.
		GK_2D = 2, ///< 2 dimension.
		GK_3D = 3 ///< 3 dimension.
	} DimensionNumber;

	static const std::size_t Dimension2 = 2;
	static const std::size_t Dimension3 = 3;

	typedef enum {
		GK_SinglePrecision = __SIZEOF_FLOAT__,
		GK_DoublePrecision = __SIZEOF_DOUBLE__,
		GK_LongDoublePrecision = __SIZEOF_LONG_DOUBLE__,
	} Precision;

	typedef enum {
		StartEdge, EndEdge, EdgeSize
	} Edge;

	static const std::size_t X = 0;
	static const std::size_t Y = 1;
	static const std::size_t Z = 2;
	static const std::size_t W = 3;

	static const std::size_t U = 0;
	static const std::size_t V = 1;

	static const std::size_t MinTag = 0;
	static const std::size_t MaxTag = 1;

//	static const gkfloat Nan;
//	static const gkfloat Zero;
//	static const gkfloat Unit;
//	static const gkfloat Infinity;

	static const bool Upper = false;
	static const bool Lower = true;

	static const bool InfiniteLength = false;
	static const bool FiniteLength = true;
};

class GK_Propeties {

};

class GK_Set {

};

template<std::size_t _Dimension1, std::size_t _Dimension2>
struct check_dimension;

template<std::size_t _Dimension>
struct check_dimension<_Dimension, _Dimension> {
};

}  // namespace gk

#endif /* GKDEF_H_ */
