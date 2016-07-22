#ifndef POINTD_HASH_H
#define POINTD_HASH_H

namespace std {
template<>
class hash<Pointd> {
public:
    size_t operator()(const Pointd& k) const
    {
        using std::size_t;
        using std::hash;

        return ((hash<double>()(k.x())
                 ^ (hash<double>()(k.y()) << 1)) >> 1)
                ^ (hash<double>()(k.z()) << 1);
    }
};

template<>
class hash<pair<Pointd, Dcel::Face*>> {
public:
    size_t operator()(const pair<Pointd, Dcel::Face*>& k) const
    {
        using std::size_t;
        using std::hash;

        return (hash<Pointd>()(k.first)
                 ^ (hash<Dcel::Face*>()(k.second) << 1));
    }
};
}

#endif // POINTD_HASH_H
