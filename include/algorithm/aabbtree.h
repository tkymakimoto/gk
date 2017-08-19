/*
 * aabbtree.h
 *
 *  Created on: 2015/10/08
 *      Author: makitaku
 */

#ifndef ALGORITHM_AABBTREE_H_
#define ALGORITHM_AABBTREE_H_

#include <list>

#include <gkaabb.h>

namespace gk {

template<typename T, typename Vector = typename geometry_traits<T>::vector_type>
class aabbtree {
public:
	static const std::size_t ChildrenSize = 2;

	typedef T value_type;
	typedef Vector vector_type;

	typedef std::list<T> data_container_type;

	struct node {
		typedef typename data_container_type::iterator iterator;
		iterator position;
		aabb<vector_type> aabb;

		node() :
				aabb() {
		}
	};

	typedef std::vector<node> node_container_type;

public:
	aabbtree() :
			X_() {
	}

	aabbtree(const aabbtree& other) {
	}

	aabbtree(const T& x) :
			X_() {
	}

	template<typename InputIterator>
	aabbtree(InputIterator first, InputIterator last) :
			X_(first, last) {

	}

	~aabbtree() {
	}

	void insert(const value_type& x);

	template<typename InputIterator>
	void insert(InputIterator first, InputIterator last);

// std::pair<iterator,iterator> intersect(const aabb& x) const;
private:
	data_container_type X_;
	node_container_type Y_;

private:
	void generate(typename data_container_type::const_iterator first,
			typename data_container_type::const_iterator last) {

		std::vector<aabb<vector_type> > boxies;
		boxies.reserve(std::distance(first, last));
		for (typename data_container_type::const_iterator p = first; p != last;
				++p) {
			boxies.push_back(make_aabb(*p));
		}
	}
};

}  // namespace gk

#endif /* ALGORITHM_AABBTREE_H_ */
