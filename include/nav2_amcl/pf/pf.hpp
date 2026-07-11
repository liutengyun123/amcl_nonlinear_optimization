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

#ifndef NAV2_AMCL__PF__PF_HPP_
#define NAV2_AMCL__PF__PF_HPP_

#include <stdint.h>

#include "nav2_amcl/pf/pf_vector.hpp"
#include "nav2_amcl/pf/pf_kdtree.hpp"

#ifdef __cplusplus
extern "C" {
#endif

struct _pf_t;
struct _rtk_fig_t;
struct _pf_sample_set_t;

typedef pf_vector_t (* pf_init_model_fn_t) (void * init_data);
typedef void (* pf_action_model_fn_t) (
  void * action_data,
  struct _pf_sample_set_t * set);
typedef double (* pf_sensor_model_fn_t) (
  void * sensor_data,
  struct _pf_sample_set_t * set);

typedef struct
{
  pf_vector_t pose;
  double weight;
} pf_sample_t;

typedef struct
{
  int count;
  double weight;
  pf_vector_t mean;
  pf_matrix_t cov;
  double m[4], c[2][2];
} pf_cluster_t;

typedef struct _pf_sample_set_t
{
  int sample_count;
  pf_sample_t * samples;
  pf_kdtree_t * kdtree;
  int cluster_count, cluster_max_count;
  pf_cluster_t * clusters;
  pf_vector_t mean;
  pf_matrix_t cov;
  int converged;
} pf_sample_set_t;

typedef struct _pf_t
{
  int min_samples, max_samples;
  double pop_err, pop_z;
  int current_set;
  pf_sample_set_t sets[2];
  double w_slow, w_fast;
  double alpha_slow, alpha_fast;
  pf_init_model_fn_t random_pose_fn;
  double dist_threshold;
  int converged;
} pf_t;

pf_t * pf_alloc(
  int min_samples, int max_samples,
  double alpha_slow, double alpha_fast,
  pf_init_model_fn_t random_pose_fn);
void pf_free(pf_t * pf);
void pf_init(pf_t * pf, pf_vector_t mean, pf_matrix_t cov);
void pf_init_model(pf_t * pf, pf_init_model_fn_t init_fn, void * init_data);
void pf_update_sensor(pf_t * pf, pf_sensor_model_fn_t sensor_fn, void * sensor_data);
void pf_update_resample(pf_t * pf, void * random_pose_data);
int pf_get_cluster_stats(
  pf_t * pf, int cluster, double * weight,
  pf_vector_t * mean, pf_matrix_t * cov);
void pf_cluster_stats(pf_t * pf, pf_sample_set_t * set);
void pf_draw_samples(pf_t * pf, struct _rtk_fig_t * fig, int max_samples);
void pf_draw_hist(pf_t * pf, struct _rtk_fig_t * fig);
void pf_draw_cluster_stats(pf_t * pf, struct _rtk_fig_t * fig);
int pf_update_converged(pf_t * pf);
void pf_init_converged(pf_t * pf);

#ifdef __cplusplus
}
#endif

#endif  // NAV2_AMCL__PF__PF_HPP_
