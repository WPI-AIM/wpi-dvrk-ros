#ifndef CCONVERSION_H
#define CCONVERSION_H

#include <boost/function.hpp>
#include <boost/bind.hpp>

template<typename D>
class Conversion{
public:
    Conversion():_is_set(false){}
    ~Conversion(){}

    template<typename C>
    void assign_conversion_fcn(void (C::*conversion_fcn)(D), C *obj){
        fcn_handle = boost::bind(conversion_fcn, obj, _1);
        _is_set = true;
    }

    boost::function<void (D)> fcn_handle;
    bool _is_set;
};

#endif
