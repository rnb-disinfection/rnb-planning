#ifndef EXPRESSIONGRAPH_COLLISION_BULLET_COLLISION_HPP
#define EXPRESSIONGRAPH_COLLISION_BULLET_COLLISION_HPP
/**
 * collision between two shapes as an expressiongraph, implemented using
 * the Bullet library
 *
 * (c) 2015, Erwin Aertbelien, Dep. Of Mech. Eng., KULeuven.
 */


#include <iostream>
#include "btBulletCollisionCommon.h"
#include <BulletCollision/NarrowPhaseCollision/btGjkEpa2.h>
#include <BulletCollision/CollisionShapes/btMinkowskiSumShape.h>

//#include "fcl/narrowphase/gjk_libccd.h"
#include <exception>
#include <stdexcept>
#include <kdl/expressiontree.hpp>


//#define DEBUG_OUTPUT 1

#define BULLET_SCALE 1000.0

typedef boost::shared_ptr< btConvexShape > btConvexShapePtr;

namespace KDL {

    inline btTransform to_btTransform( const Frame& F) {
            double x,y,z,w;
            F.M.GetQuaternion(x,y,z,w); 
            btTransform T1(btQuaternion(x,y,z,w),btVector3(BULLET_SCALE*F.p[0],BULLET_SCALE*F.p[1], BULLET_SCALE*F.p[2]));
            return T1;
    };

    inline KDL::Vector to_KDL(const btVector3& v) {
        return KDL::Vector(v[0],v[1],v[2])/BULLET_SCALE;
    }

    inline KDL::Vector to_KDL_no_scale(const btVector3& v) {
        return KDL::Vector(v[0],v[1],v[2]);
    }


    class DistanceNode:
        public BinaryExpression<double,Frame,Frame>
    {
        btConvexShapePtr s1;
        btConvexShapePtr s2;
        btVector3 guess;
        Frame tf1;
        Frame tf2;
        Vector p1_local;
        Vector p2_local;
        Vector normalv;
        double radius1;
        double margin1;
        double radius2;
        double margin2;
        double distance;
    public:

        DistanceNode(   Expression<Frame>::Ptr a1, 
                        btConvexShapePtr _s1,
                        double _radius1,
                        double _margin1,
                        Expression<Frame>::Ptr a2,
                        btConvexShapePtr _s2,
                        double _radius2,
                        double _margin2):
            BinaryExpression<double,Frame,Frame>("collision",a1,a2),
            s1(_s1), 
            s2(_s2),
            guess(sqrt(2),sqrt(2),0),
            radius1(_radius1),
            margin1(_margin1),
            radius2(_radius2),
            margin2(_margin2) {
                if (margin1 > 0) {
                    s1->setMargin(margin1);
                }
                if (margin2 > 0) {
                    s2->setMargin(margin2);
                }
        }

        virtual double value() {
            using namespace std;
            tf1 = this->argument1->value();
            tf2 = this->argument2->value();
            btTransform T1 = to_btTransform(tf1);
            btTransform T2 = to_btTransform(tf2);
            btGjkEpaSolver2::sResults results;
            bool b=btGjkEpaSolver2::SignedDistance (&(*s1), T1, &(*s2), T2, guess, results);
            guess = results.normal;
            assert(results.status!=3);
            Vector p1_world = to_KDL( results.witnesses[0]);
            p1_local = tf1.Inverse(p1_world);
            Vector p2_world = to_KDL( results.witnesses[1]);
            p2_local = tf2.Inverse(p2_world);
            normalv  = tf1.M*to_KDL_no_scale(results.normal);
#ifdef DEBUG_OUTPUT
            cout << "   status      \t" << results.status << endl;
            cout << "   witnesses 1 \t" << results.witnesses[0][0] << "\t" << results.witnesses[0][1] << "\t"<<  results.witnesses[0][2]  << endl;
            cout << "   witnesses 2 \t" << results.witnesses[1][0] << "\t" << results.witnesses[1][1] << "\t"<<  results.witnesses[1][2]  << endl;
            btVector3 tmp = results.witnesses[0]-results.witnesses[1];
            tmp /= tmp.length();
            cout << "   my normal      \t" << tmp[0] << "\t" << tmp[1] << "\t" << tmp[2]  << endl;
            cout << "   normal      \t" << results.normal[0] << "\t" << results.normal[1] << "\t" << results.normal[2]  << endl;
            cout << "   normal trfst\t" << normalv<< endl;
            cout << "   distance    \t" << results.distance << endl;
#endif
            return results.distance/BULLET_SCALE-radius1-radius2;
        } 

        virtual double derivative(int i) {
            using namespace std;
            /*return distance->derivative(i);*/
            KDL::Vector de;
            KDL::Twist  t1 = argument1->derivative(i);
            KDL::Twist  t2 = argument2->derivative(i);
            de = t1.rot * (tf1.M*p1_local)  + t1.vel - t2.rot*(tf2.M*p2_local) - t2.vel;
            double der= dot(normalv, de); 
#ifdef DEBUG_OUTPUT
            cout << "   Twist 1 " << t1 << endl;
            cout << "   Twist 2 " << t2 << endl;
            cout << "   Vel p1  " <<  (t1.rot * (tf1.M*p1_local)  + t1.vel) << endl;
            cout << "   Vel p2  " <<  (t2.rot * (tf2.M*p2_local)  + t2.vel) << endl;
            cout << "   normal  " << normalv << endl;
            cout << "   derivative " << der << endl;
#endif
            return der;
        } 

        virtual Expression<double>::Ptr derivativeExpression(int i) {
            // TODO
            assert(0 /* not implemented */);
            return Expression<double>::Ptr();
        }

        virtual Expression<double>::Ptr clone() {
            return boost::make_shared< DistanceNode >( 
                    this->argument1, 
                    s1,
                    radius1,margin1,
                    this->argument2,
                    s2,
                    radius2,margin2
                    );
        } 
        ~DistanceNode() {}
    };
    /**
     * a1: location of the origin of convex volume 1 as an expressiongraph
     * s1: description of the polyhedron/shape underlying convex volume 1
     * radius1: spherical extension of the polyhedron in s1 
     * margin1: margin in the distance computations for object 1
     * a2,s2,radius2, margin2 : idem for volume 2
     */
    inline Expression<double>::Ptr distance_between( 
                        Expression<Frame>::Ptr  a1, 
                        btConvexShapePtr            s1,
                        double                  radius1,
                        double                  margin1,
                        Expression<Frame>::Ptr  a2,
                        btConvexShapePtr            s2,
                        double                  radius2,
                        double                  margin2) {
        if (!a1) {
            throw std::domain_error("MotionProfileTrapezoidal:: first shape pointer cannot be NULL");
        }
        if (!s2) {
            throw std::domain_error("MotionProfileTrapezoidal:: second shape pointer cannot be NULL");
        }
        return boost::make_shared< DistanceNode >(
                    a1,s1,radius1, margin1, a2,s2,radius2, margin2
               );
    }







};// namespace KDL



#endif
