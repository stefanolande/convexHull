#ifndef POINTD_HASH_H
#define POINTD_HASH_H

#include <boost/functional/hash.hpp>

namespace std {
template<>
class hash<Pointd> {
public:
    size_t operator()(const Pointd& k) const
    {
        using std::size_t;
        using boost::hash_combine;

        size_t seed = 0;

        hash_combine(seed, k.x());
        hash_combine(seed, k.y());
        hash_combine(seed, k.z());

        return seed;
    }
};

template<>
class hash<pair<Pointd, Dcel::Face*>> {
public:
    size_t operator()(const pair<Pointd, Dcel::Face*>& k) const
    {
        using std::size_t;
        using std::hash;
        using boost::hash_combine;

        size_t seed = 0;

        seed = hash<Pointd>()(k.first);
        hash_combine(seed, k.second);

        return seed;
    }
};
}

#endif // POINTD_HASH_H
