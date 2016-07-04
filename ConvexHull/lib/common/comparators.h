/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#ifndef CCOMPARATORS_H
#define CCOMPARATORS_H

#include <QColor>

/**
 * \~English
 * @struct cmpQColor
 * @brief The cmpQColor structure implements a comparator for the QColor class.
 *
 * It defines a minus operator on QColor, ordering on the components red, green and blue. It can be used, for example,
 * when you need to store QColor on ordered containers like std::set or std::map.
 *
 * Example: std::map<QColor, int, cmpQColor> map;
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 *
 * \~Italian
 * @struct cmpQColor
 * @brief La struttura cmpQColor implementa un comparatore per classe QColor.
 *
 * È in grado di stabilire se un QColor è minore di un altro o meno, ordinando rispettivamente per componente
 * red, poi green e poi blue. Può essere utilizzato come parametro nel caso in cui si voglia usare un QColor su
 * una struttura dati che necessita di un ordinamento.
 *
 * Esempio: std::map<QColor, int, cmpQColor> map;
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
struct cmpQColor {
    bool operator()(const QColor& a, const QColor& b) const {
        if (a == b) return false;
        if (a.red() == b.red() && a.green() == b.green()) return (a.blue() < b.blue());
        if (a.red() == b.red()) return (a.green() < b.green());
        return (a.red() < b.red());
    }
};

/**
 * \~English
 * @struct cmpUnorderedStdPair
 * @brief la struttura cmpUnorderedStdPair implementa un comparatore per la classe std::pair<T,T>
 *
 * It compares a couple of elements and decides if one is less than the other without taking account on the elements order.
 * It can be used if you need a set of unordered couples, where you can't have couples with inverted elements.
 *
 * Example: the couple (0,2) is not less than (2,0), and viceversa.
 *
 * Example: the couple (2,0) is less than the couple (0,3).
 *
 * Example of usage: std::set<std::pair<int, int>, cmpUnorderedStdPair<int> > set;
 * Note that the type of the elements of the pair must be the same, and must match to the comparator type.
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 *
 * \~Italian
 * @struct cmpUnorderedStdPair
 * @brief The cmpUnorderedStdPair structure implements a comparator for the std::pair<T,T> class.
 *
 * Confronta una coppia di elementi e stabilisce se una è minore dell'altra senza tener conto dell'ordine
 * degli elementi.
 * Può essere utile se si vuole fare in modo che in un insieme di coppie non si possano avere due coppie
 * con gli stessi elementi invertiti.
 *
 * Esempio: La coppia (0,2) non risulta essere minore della coppia (2,0), e viceversa.
 *
 * Esempio: la coppia (2,0) risulta essere minore della coppia (0,3).
 *
 * Esempio: std::set<std::pair<int, int>, cmpUnorderedStdPair<int> > set;
 * Si noti che il tipo dei due elementi della coppia deve essere lo stesso, e deve coincidere con quello del comparator.
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
template <class T>
struct cmpUnorderedStdPair {
    bool operator()(const std::pair<T,T>& a, const std::pair<T,T>& b) const {
        T amin, bmin, amax, bmax;
        if (a.first < a.second) {
            amin = a.first;
            amax = a.second;
        }
        else {
            amin = a.second;
            amax = a.first;
        }
        if (b.first < b.second){
            bmin = b.first;
            bmax = b.second;
        }
        else {
            bmin = b.second;
            bmax = b.first;
        }
        if (amin < bmin) return true;
        else if (amin == bmin) return (amax < bmax);
        return false;
    }
};

#endif // CCOMPARATORS_H

