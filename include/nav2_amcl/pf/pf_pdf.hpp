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

#ifndef NAV2_AMCL__PF__PF_PDF_HPP_
#define NAV2_AMCL__PF__PF_PDF_HPP_

#include "nav2_amcl/pf/pf_vector.hpp"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  pf_vector_t x;
  pf_matrix_t cx;
  double cxdet;
  pf_matrix_t cr;
  pf_vector_t cd;
} pf_pdf_gaussian_t;

pf_pdf_gaussian_t * pf_pdf_gaussian_alloc(pf_vector_t x, pf_matrix_t cx);
void pf_pdf_gaussian_free(pf_pdf_gaussian_t * pdf);
double pf_ran_gaussian(double sigma);
pf_vector_t pf_pdf_gaussian_sample(pf_pdf_gaussian_t * pdf);

#ifdef __cplusplus
}
#endif

#endif  // NAV2_AMCL__PF__PF_PDF_HPP_
