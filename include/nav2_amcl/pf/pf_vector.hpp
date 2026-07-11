/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef NAV2_AMCL__PF__PF_VECTOR_HPP_
#define NAV2_AMCL__PF__PF_VECTOR_HPP_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

typedef struct
{
  double v[3];
} pf_vector_t;

typedef struct
{
  double m[3][3];
} pf_matrix_t;

pf_vector_t pf_vector_zero(void);
pf_vector_t pf_vector_sub(pf_vector_t a, pf_vector_t b);
pf_vector_t pf_vector_coord_add(pf_vector_t a, pf_vector_t b);
pf_matrix_t pf_matrix_zero(void);
void pf_matrix_unitary(pf_matrix_t * r, pf_matrix_t * d, pf_matrix_t a);

#ifdef __cplusplus
}
#endif

#endif  // NAV2_AMCL__PF__PF_VECTOR_HPP_
