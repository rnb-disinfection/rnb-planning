/*
 * expressiontree_velocities.cpp
 *
 *
* 
* Copyright 2016 Erwin Aertbelien - KU Leuven - Dep. of Mechanical Engineering
*
* Licensed under the EUPL, Version 1.1 only (the "Licence");
* You may not use this work except in compliance with the Licence.
* You may obtain a copy of the Licence at:
*
* http://ec.europa.eu/idabc/eupl 
*
* Unless required by applicable law or agreed to in writing, software 
* distributed under the Licence is distributed on an "AS IS" basis,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the Licence for the specific language governing permissions and 
* limitations under the Licence.
*/

#ifndef KDL_EXPRESSIONGRAPH_VELOCITIES_HPP
#define KDL_EXPRESSIONGRAPH_VELOCITIES_HPP

#include <kdl/expressiontree_expressions.hpp>
#include <kdl/frames.hpp>


namespace KDL {

/**
 * an expressiongraph node that remembers the previous value of an expression and computes its derivative using 
 * numerical differentiation.  Useful to formulate expressions for the magnitude of a velocity or to formulate
 * constraints that track the orientation of a velocity.
 */
template <typename ValueType>
class Previous_Velocity:
    public BinaryExpression<typename AutoDiffTrait<ValueType>::DerivType, double, ValueType>
{
public:
    typedef BinaryExpression<typename AutoDiffTrait<ValueType>::DerivType, double, ValueType> BinExpr;
    typedef typename AutoDiffTrait<ValueType>::DerivType ResultType;
    typedef typename AutoDiffTrait< typename AutoDiffTrait<ValueType>::DerivType >::DerivType ResultDerivType;
    ValueType prev_value;
    double    prev_time;
    double    cur_time;
    ValueType cur_value; 
    ResultType  deriv_value;
    bool        first_time;
public:
    Previous_Velocity(
                const  typename BinExpr::Argument1Expr::Ptr& argtime,
                const  typename BinExpr::Argument2Expr::Ptr& argexpr
                ):
                BinExpr("Previous_Velocity",argtime,argexpr),
                prev_time(0),
                first_time(true)
                {}
    Previous_Velocity(
                const  typename BinExpr::Argument1Expr::Ptr& argtime,
                const  typename BinExpr::Argument2Expr::Ptr& argexpr,
                double _prev_time,
                const ValueType& _prev_value,
                const ResultType& _deriv_value
                ):
                BinExpr("Previous_Velocity",argtime,argexpr),
                prev_time(0),
                first_time(true)
                {}


    virtual ResultType value() {
        cur_time  = this->argument1->value();
        cur_value   = this->argument2->value();
        if (!first_time) {
            if (cur_time > prev_time) {
                deriv_value = diff(prev_value, cur_value,cur_time-prev_time);
            }         
        } else {
            deriv_value = AutoDiffTrait<ValueType>::zeroDerivative();
            first_time = false;
        }
        prev_value   = cur_value;
        prev_time    = cur_time;
        return deriv_value;
    }

    virtual ResultDerivType derivative(int i) {
        return AutoDiffTrait< ResultType  >::zeroDerivative();
    }

    virtual typename Expression<ResultDerivType>::Ptr derivativeExpression(int i) {
        return Constant<ResultDerivType>( AutoDiffTrait< ResultType  >::zeroDerivative());
    }

    virtual  typename BinExpr::Ptr clone() {
        typename Expression<ResultType>::Ptr expr(
            new Previous_Velocity(this->argument1->clone(),
                                  this->argument2->clone(), prev_time, prev_value, deriv_value)
        );
        return expr;
    }
};


template<typename ValueType>
inline typename Expression< typename Previous_Velocity<ValueType>::ResultType   >::Ptr 
previous_velocity(const Expression<double>::Ptr  time, const typename Expression<ValueType>::Ptr e) {
    typename Expression< typename Previous_Velocity<ValueType>::ResultType   >::Ptr expr(
        new Previous_Velocity<ValueType>(time,e)
    );
    return expr;
}

} // end of namespace KDL
#endif
