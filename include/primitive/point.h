/*
 * point.h
 *
 *  Created on: 2015/10/30
 *      Author: Takuya Makimoto
 */

#ifndef PRIMITIVE_POINT_H_
#define PRIMITIVE_POINT_H_

#include "../gkvector.h"

namespace gk {

/**
 * @brief This class represents point.
 *
 * @author Takuya Makiomto
 * @date  2017/06/25
 */
template<typename _T, std::size_t _Dimension>
class point: public geometry<point_tag, _T, _Dimension, _T> {
public:

	GK_VECTOR_TYPEDEF(_T, _Dimension)

public	:
	point() : x_() {
	}

	point(const point& other) : x_(other.x_) {
	}

	template<typename Vector>
	point(const Vector& v) : x_() {
		this->assign_(v);
	}

	~point() {
	}

	/**
	 *
	 * @param n
	 * @return
	 */
	const _T& operator[](std::size_t n) const {
		return this->x_[n];
	}

	_T& operator[](std::size_t n) {
		return this->x_[n];
	}

	point& operator=(const point& q) {
		if (*q == this) {
			return *this;
		}

		this->x_ = q.x_;
		return *this;
	}

	template<typename Vector>
	point& operator=(const Vector& v) {
		this->assign_(v);
	}

private:
	vector_type x_; /// position vector.

private:
	template<typename _Vector>
	void assign_(const _Vector& v) {
		for (std::size_t i = 0; i < _Dimension; ++i) {
			this->x_[i] = v[i];
		}
	}
};

}
 // namespace gk

#endif /* PRIMITIVE_POINT_H_ */
