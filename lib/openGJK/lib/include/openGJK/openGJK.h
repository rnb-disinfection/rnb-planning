/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  *
 *                                   #####        # #    #                *
 *       ####  #####  ###### #    # #     #       # #   #                 *
 *      #    # #    # #      ##   # #             # #  #                  *
 *      #    # #    # #####  # #  # #  ####       # ###                   *
 *      #    # #####  #      #  # # #     # #     # #  #                  *
 *      #    # #      #      #   ## #     # #     # #   #                 *
 *       ####  #      ###### #    #  #####   #####  #    #                *
 *                                                                        *
 *  This file is part of openGJK.                                         *
 *                                                                        *
 *  openGJK is free software: you can redistribute it and/or modify       *
 *   it under the terms of the GNU General Public License as published by *
 *   the Free Software Foundation, either version 3 of the License, or    *
 *   any later version.                                                   *
 *                                                                        *
 *   openGJK is distributed in the hope that it will be useful,           *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of       *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See The        *
 *   GNU General Public License for more details.                         *
 *                                                                        *
 *  You should have received a copy of the GNU General Public License     *
 *   along with Foobar.  If not, see <https://www.gnu.org/licenses/>.     *
 *                                                                        *
 *       openGJK: open-source Gilbert-Johnson-Keerthi algorithm           *
 *            Copyright (C) Mattia Montanari 2018 - 2019                  *
 *              http://iel.eng.ox.ac.uk/?page_id=504                      *
 *                                                                        *
 * - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  *
 *                                                                        *
 *  This is the header file for the openGJK.c file. It defines the        *
 *	 openGJK function and its structures.							      *
 *                                                                        *
 * - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  */

#ifndef __OPENGJK_H__
#define __OPENGJK_H__

#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include <vector>


/**
 * @brief Macro that implements the CompareSign function (see paper).
 */
 #define SAMESIGN( a, b ) ( (a>0) == (b>0) )

class Point3: public std::vector<double> {
public:
    Point3(): std::vector<double>(){}
    Point3(double x, double y, double z): std::vector<double>(){
        this->push_back(x);
        this->push_back(y);
        this->push_back(z);
    }
};

typedef std::vector<Point3> PointList;

/**
 * @brief Structure of a body.
 */
struct bd {
  int numpoints;    /**< Number of points defining the body.            */
  PointList coord;  /**< Pointer to pointer to the points' coordinates. */
  double  s [3];    /**< Support mapping computed last.                 */
};

/**
 * @brief Structure for a simplex.
 */
struct simplex {
  int    nvrtx   ;       /**< Number of simplex's vertices. 			*/  
  double vrtx    [4][3]; /**< Coordinates of simplex's vertices. 		*/ 
  int    wids    [4];    /**< Label of the simplex's vertices. 			*/  
  double lambdas [4];    /**< Barycentric coordiantes for each vertex.  */
  double  p [4][3];     /**< Points of P that form the simplex  */  
  double  q [4][3];	    /**< Points of Q that form the simplex  */

  void clear(){
      nvrtx=0;
      vrtx[0][0]=0;
      vrtx[1][0]=0;
      vrtx[2][0]=0;
      vrtx[3][0]=0;
      vrtx[0][1]=0;
      vrtx[1][1]=0;
      vrtx[2][1]=0;
      vrtx[3][1]=0;
      vrtx[0][2]=0;
      vrtx[1][2]=0;
      vrtx[2][2]=0;
      vrtx[3][2]=0;
      wids[0]=0;
      wids[1]=0;
      wids[2]=0;
      wids[3]=0;
      lambdas[0]=0;
      lambdas[1]=0;
      lambdas[2]=0;
      lambdas[3]=0;
      p[0][0]=0;
      p[1][0]=0;
      p[2][0]=0;
      p[3][0]=0;
      p[0][1]=0;
      p[1][1]=0;
      p[2][1]=0;
      p[3][1]=0;
      p[0][2]=0;
      p[1][2]=0;
      p[2][2]=0;
      p[3][2]=0;
      q[0][0]=0;
      q[1][0]=0;
      q[2][0]=0;
      q[3][0]=0;
      q[0][1]=0;
      q[1][1]=0;
      q[2][1]=0;
      q[3][1]=0;
      q[0][2]=0;
      q[1][2]=0;
      q[2][2]=0;
      q[3][2]=0;
  }
};

/**
 * @brief The GJK algorithm which returns the minimum distance between 
 * two bodies.
 */
extern double gjk( struct bd, struct bd, struct simplex * );

//extern void gjk_batch(struct bd *, int*, int*, int, double *);

#endif