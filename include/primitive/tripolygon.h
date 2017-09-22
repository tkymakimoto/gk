/*
 * tripolygon.h
 *
 *  Created on: 2017/09/22
 *      Author: J0115775
 */

#ifndef PRIMITIVE_TRIPOLYGON_H_
#define PRIMITIVE_TRIPOLYGON_H_

#include "../gkgeometry.h"
#include "triangle.h"

#include <vector>

namespace gk {

template<typename _T, std::size_t _Dimension>
class tripolygon: geometry<surface_tag, _T, _Dimension, void> {
	typedef geometry<surface_tag, _T, _Dimension, void> base;

	GK_GEOMETRY_TYPEDEF(base)

#if __cplusplus >= 201103L
	using index_type = std::size_t;
#else
	typedef std::size_t index_type;
#endif

	const std::size_t VertexSize = 3;

#if __cplusplus >= 201103L
	typedef std::array<index_type, 3> indices;
#else
	struct indices {
		index_type id[3];
	};

#endif

public:
	tripolygon() :
			vertices_() {
	}

	tripolygon(const tripolygon& other) :
			vertices_(other) {
	}

	~tripolygon() {
	}

	void add(const vector_type& first, const vector_type& second,
			const vector_type& third) {

	}

	void add(const triangle<_T, _Dimension>& a) {

	}

	void remove(std::size_t index) {

	}

	triangle<_T, _Dimension> operator[](std::size_t index) const {
		std::size_t k[] = this->indices_set[index];
		triangle<_T, _Dimension> r(this->vertices_[k[0]], this->vertices_[k[1]],
				this->vertices_[k[2]]);
		return r;
	}

	tripolygon& operator=(const tripolygon& rhs) {
		if (&rhs == this) {
			return *this;
		}

		return *this;
	}

private:
	std::vector<vector_type> vertices_;
	std::vector<indices> indices_set;
};

} // namespace gk

#endif /* PRIMITIVE_TRIPOLYGON_H_ */
