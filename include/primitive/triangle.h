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
 * @see inner::direction_of_triangle()
 */
template<typename _T, std::size_t _Dimension>
direction<_Dimension> direction_of(const triangle<_T, _Dimension>& a) {
	return cross<direction < _Dimension>()(normalize(a.u()), normalize(a.v()));
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
template<typename Vector, typename AreaValue>
AreaValue area_of_triangle(const Vector& first, const Vector& second,
		const Vector& third, dimension_tag<GK::GK_2D>) {

}

/**
 *
 * @param first
 * @param second
 * @param third
 * @param
 * @return
 */
template<typename Vector, typename AreaValue>
AreaValue area_of_triangle(const Vector& first, const Vector& second,
		const Vector& third, dimension_tag<GK::GK_3D>) {

}

}  // namespace inner

template<typename _T, typename AreaValue>
struct area {
	AreaValue operator()(const _T&) const;
};

template<typename _T, std::size_t _Dimension, typename AreaValue>
struct area<triangle<_T, _Dimension>, AreaValue> {
	AreaValue operator()(const triangle<_T, _Dimension>& a) const {
//		const dot<Vector, Vector, AreaValue> dot;
		const AreaValue dot_u = dot(a.u(), a.u());
		const AreaValue dot_v = dot(a.v(), a.v());
		const AreaValue dot_uv = dot(a.u(), a.v());
		return 0.5 * std::sqrt(dot_u * dot_v - dot_uv * dot_uv);
	}
};

}  // namespace gk

//#include <vector/gkvector.h>
//
//class gktriangle_parameter {
//
//};
//
///**
// * @brief
// *
// * @author Takuya Makimoto
// * @date 2015
// */
//template<GK::DimensionNumber Size>
//class gktriangle {
//public:
//#if __cplusplus >= 201103L
//#else
//    typedef gkvector<Size>* iterator;
//    typedef const gkvector<Size>* const_iterator;
//    typedef std::reverse_iterator<iterator> reverse_iterator;
//    typedef std::reverse_iterator<const_iterator> const_reverse_iterator;
//
//#endif
//
//    typedef gkvector<Size> vector_type;
//
//    enum {
//        First, Second, Third, Size
//    };
//
//    /**
//     * @brief
//     */
//    struct area_parameter {
//        gkfloat first;
//        gkfloat second;
//        gkfloat third;
//
//        area_parameter() :
//                first(), second(), third() {
//        }
//
//        area_parameter(const area_parameter& other) :
//                first(other.first), second(other.second), third(other.third) {
//
//        }
//
//        area_parameter(gkfloat first, gkfloat second, gkfloat third) :
//                first(first), second(second), third(third) {
//
//        }
//
//        ~area_parameter() {
//
//        }
//    };
//
//    typedef area_parameter parameter;
//
//public:
//    gktriangle() :
//            vertices_() {
//
//    }
//
//    gktriangle(const gktriangle& other) :
//            vertices_() {
//        this->vertices_[First] = other.vertices_[First];
//        this->vertices_[Second] = other.vertices_[Second];
//        this->vertices_[Third] = other.vertices_[Third];
//    }
//
//    gktriangle(const gkinvalid& invalid) :
//            vertices_() {
//
//    }
//
//    gktriangle(const gkvector<Size>& first,
//            const gkvector<Size>& second, const gkvector<Size>& third) :
//            vertices_() {
//        this->vertices_[First] = first;
//        this->vertices_[Second] = second;
//        this->vertices_[Third] = third;
//    }
//
//    ~gktriangle() {
//
//    }
//
//    bool valid() const;
//
//    const gkvector<Size>& first() const {
//        return this->vertices_[First];
//    }
//
//    gkvector<Size>& first() {
//        return this->vertices_[First];
//    }
//
//    const gkvector<Size>& second() const {
//        return this->vertices_[Second];
//    }
//
//    gkvector<Size>& second() {
//        return this->vertices_[Second];
//    }
//
//    const gkvector<Size>& third() const {
//        return this->vertices_[Third];
//    }
//
//    gkvector<Size>& third() {
//        return this->vertices_[Third];
//    }
//
//    void set(const vector_type& first, const vector_type& second,
//            const vector_type& third) {
//        this->set_(first, second, third);
//    }
//
//    const_iterator begin() const;
//    iterator begin();
//
//    const_iterator end() const;
//    iterator end();
//
//    const_reverse_iterator rbegin() const;
//    reverse_iterator rbegin();
//
//    const_reverse_iterator rend() const;
//    reverse_iterator rend();
//
//    void reverse() {
//        std::swap(this->vertices_[First], this->vertices_[Third]);
//    }
//
//    gkfloat area() const {
//        return this->area_();
//    }
//
//    void translate(const gkvector<Size>& translation) {
//        this->vertices_[First] += translation;
//        this->vertices_[Second] += translation;
//        this->vertices_[Third] += translation;
//    }
//
//    void rotate(const gkrotation<Size>& rotation) {
//        Eigen::Matrix<gkfloat, Size, Size> R = rotation_matrix(
//                rotation);
//        this->vertices_[First] *= R;
//        this->vertices_[Second] *= R;
//        this->vertices_[Third] *= R;
//    }
//
//    void scale(gkfloat factor) {
//        this->vertices_[First] *= factor;
//        this->vertices_[Second] *= factor;
//        this->vertices_[Third] *= factor;
//    }
//
//    const vector_type& operator[](gksize index) const {
//        return this->vertices_[index];
//    }
//
//    vector_type& operator[](gksize index) {
//        return this->vertices_[index];
//    }
//
//    /**
//     * @brief Computes the position.
//     * @param t
//     * @return
//     */
//    vector_type operator()(const area_parameter& t) const {
//        const gkfloat f = 1.0 / (t.first + t.second + t.third);
//
//        const vector_type u = f * t.second
//                * (this->vertices_[Second] - this->vertices_[First])
//                + this->vertices_[First];
//
//        const vector_type v = f * t.third
//                * (this->vertices_[Third] - this->vertices_[First])
//                + this->vertices_[First];
//
//        return v + u;
//    }
//
//    gktriangle& operator=(const gktriangle& rhs) {
//        if (&rhs == this) {
//            return *this;
//        }
//
//        this->vertices_[First] = rhs.vertices_[First];
//        this->vertices_[Second] = rhs.vertices_[Second];
//        this->vertices_[Third] = rhs.vertices_[Third];
//
//        return *this;
//    }
//
//private:
//    void set_(const vector_type& first, const vector_type& second,
//            const vector_type& third) {
//        this->vertices_[First] = first;
//        this->vertices_[Second] = second;
//        this->vertices_[Third] = third;
//    }
//
//    gkfloat area_() const;
//
//private:
//#if __cplusplus>=201103L
//    std::array<gkvector<DimensionNumber>,GK_Triangle> vertices_;
//#else
//    vector_type vertices_[Size];
//#endif
//
//};
//
//template<>
//gkfloat gktriangle<GK::GK_2D>::area_() const {
//    return gkfabs(
//            0.5
//                    * cross(this->vertices_[Second] - this->vertices_[First],
//                            this->vertices_[Third] - this->vertices_[First]));
//}
//
//template<>
//gkfloat gktriangle<GK::GK_3D>::area_() const {
//    return 0.5
//            * norm(
//                    cross(this->vertices_[Second] - this->vertices_[First],
//                            this->vertices_[Third] - this->vertices_[First]));
//}
//
//template<GK::DimensionNumber Size>
//bool operator==(const gktriangle<Size>& triangle,
//        const gkinvalid& invalid) {
//    return (triangle.first() == invalid || triangle.second() == invalid
//            || triangle.third() == invalid);
//}
//
//template<GK::DimensionNumber Size>
//bool operator==(const gkinvalid& invalid,
//        const gktriangle<Size>& triangle) {
//    return (triangle == invalid);
//}
//
//template<GK::DimensionNumber Size>
//bool operator!=(const gktriangle<Size>& triangle,
//        const gkinvalid& invalid) {
//    return !(triangle == invalid);
//}
//
//template<GK::DimensionNumber Size>
//bool operator!=(const gkinvalid& invalid,
//        const gktriangle<Size>& triangle) {
//    return (triangle != invalid);
//}
//
//template<GK::DimensionNumber Size>
//bool operator==(const gktriangle<Size>& a,
//        const gktriangle<Size>& b) {
//    return (a.first() == b.first() && a.second() == b.second(), a.third()
//            == b.third());
//}
//
//template<GK::DimensionNumber Size>
//bool operator!=(const gktriangle<Size>& a,
//        const gktriangle<Size>& b) {
//    return !(a == b);
//}
//
//bool orientation(const gktriangle<GK::GK_2D>& triangle) {
//    return !gksignbit(
//            cross(triangle.second() - triangle.first(),
//                    triangle.third() - triangle.first()));
//}
//
//gkdirection<GK::GK_3D> normal(const gktriangle<GK::GK_3D>& triangle) {
//    const gkvector<GK::GK_3D> v = cross(triangle.second() - triangle.first(),
//            triangle.third() - triangle.first());
//
//    return normalize(v);
//}
//
////gkdirection<GK::GK_3D> normal(const gktriangle<GK::GK_3D>& triangle);
//
//template<GK::DimensionNumber Size>
//bool predict_intersect(const gktriangle<Size>& a,
//        const gktriangle<Size>& b);
//
//template<GK::DimensionNumber Size>
//gkfloat area(const gktriangle<Size>& triangle);

#endif /* PRIMITIVE_TRIANGLE_H_ */
