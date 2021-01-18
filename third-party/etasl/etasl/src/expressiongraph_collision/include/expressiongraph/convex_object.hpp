#ifndef EXPRESSIONGRAPH_COLLISION_CONVEX_OBJECT_HPP
#define EXPRESSIONGRAPH_COLLISION_CONVEX_OBJECT_HPP

/**
 * (c) 2015, Erwin Aertbelien, Dep. of Mech. Eng, KULeuven
 */

#include <exception>
#include <kdl/expressiontree.hpp>
#include <expressiongraph/bullet_collision.hpp>

namespace KDL {

    /**
     * Creates a convex object from a .OBJ wavefront input file.  If the file specifies a non convex object,
     * the distance algorithm will only consider the convex hull of this object.
     *
     * \param filename : name of the file to read,  throws an exception for invalid files or I/O errors
     */ 
    btConvexShapePtr create_convex(const std::string filename);

    /**
     * Creates a convex object from a .OBJ wavefront input file.  If the file specifies a non convex object,
     * the distance algorithm will only consider the convex hull of this object.
     *
     * \param filename : name of the file to read,  throws an exception for invalid files or I/O errors
     * \param scalex, scaley, scalez: scale factor in the x-, y- and z-direction. If the scale-factor is negative,
     * then abs(scale) indicates the size of the maximal extend of the object in that direction.  If scaley==0 and
     * scalez==0, then there is one unique scale factor.  Also this unique scale factor can be < 0 to indicate 
     * scaling relative to the maximal extend.
     */
    btConvexShapePtr create_convex_scale(const std::string filename, double scalex, double scaley, double scalez);

}; // namespace KDL


#endif
