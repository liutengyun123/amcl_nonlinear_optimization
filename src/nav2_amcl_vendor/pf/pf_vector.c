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

#include <math.h>

#include "nav2_amcl/pf/eig3.hpp"
#include "nav2_amcl/pf/pf_vector.hpp"

pf_vector_t pf_vector_zero(void)
{
  pf_vector_t c;

  c.v[0] = 0.0;
  c.v[1] = 0.0;
  c.v[2] = 0.0;

  return c;
}

pf_vector_t pf_vector_sub(pf_vector_t a, pf_vector_t b)
{
  pf_vector_t c;

  c.v[0] = a.v[0] - b.v[0];
  c.v[1] = a.v[1] - b.v[1];
  c.v[2] = a.v[2] - b.v[2];

  return c;
}

pf_vector_t pf_vector_coord_add(pf_vector_t a, pf_vector_t b)
{
  pf_vector_t c;

  c.v[0] = b.v[0] + a.v[0] * cos(b.v[2]) - a.v[1] * sin(b.v[2]);
  c.v[1] = b.v[1] + a.v[0] * sin(b.v[2]) + a.v[1] * cos(b.v[2]);
  c.v[2] = b.v[2] + a.v[2];
  c.v[2] = atan2(sin(c.v[2]), cos(c.v[2]));

  return c;
}

pf_matrix_t pf_matrix_zero(void)
{
  int i, j;
  pf_matrix_t c;

  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++) {
      c.m[i][j] = 0.0;
    }
  }

  return c;
}

void pf_matrix_unitary(pf_matrix_t * r, pf_matrix_t * d, pf_matrix_t a)
{
  int i, j;
  double aa[3][3];
  double eval[3];
  double evec[3][3];

  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++) {
      aa[i][j] = a.m[i][j];
    }
  }

  eigen_decomposition(aa, evec, eval);

  *d = pf_matrix_zero();
  for (i = 0; i < 3; i++) {
    d->m[i][i] = eval[i];
    for (j = 0; j < 3; j++) {
      r->m[i][j] = evec[i][j];
    }
  }
}
