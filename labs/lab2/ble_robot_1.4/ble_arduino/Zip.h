#ifndef ZIP_H
#define ZIP_H

#include <utility>

template <typename Iterator> inline void advance_all(Iterator &iterator) {
  ++iterator;
}

template <typename Iterator, typename... Iterators>
inline void advance_all(Iterator &iterator, Iterators &...iterators) {
  ++iterator;
  advance_all(iterators...);
}

template <typename Function, typename Iterator, typename... Iterators>
inline Function zip(Function &&func, Iterator begin, Iterator end,
                    Iterators... iterators) {
  for (; begin != end; ++begin, advance_all(iterators...))
    func(*begin, *(iterators)...);
  return std::forward<Function>(func);
}

#endif // ZIP_H
