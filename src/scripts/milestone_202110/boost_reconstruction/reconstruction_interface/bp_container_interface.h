//
// Created by tamp on 20. 11. 25..
//

#ifndef MOVEIT_PLAN_COMPACT_BP_CONTAINER_INTERFACE_H
#define MOVEIT_PLAN_COMPACT_BP_CONTAINER_INTERFACE_H

template<class T>
struct std_item
{
    typedef typename T::value_type V;
    static V& get(T & x, int i)
    {
        if( i<0 ) i+=x.size();
        if( i>=0 && i<x.size() ) return x[i];
        else throw;
    }
    static void set(T & x, int i, V const& v)
    {
        if( i<0 ) i+=x.size();
        if( i>=0 && i<x.size() ) x[i]=v;
        else throw;
    }
    static void del(T & x, int i)
    {
        if( i<0 ) i+=x.size();
        if( i>=0 && i<x.size() ) x.erase(x.begin() + i);
        else throw;
    }
    static void add(T & x, V const& v)
    {
        x.push_back(v);
    }
};

#endif //MOVEIT_PLAN_COMPACT_BP_CONTAINER_INTERFACE_H
