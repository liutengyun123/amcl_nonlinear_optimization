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

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>

#include "nav2_amcl/pf/pf.hpp"
#include "nav2_amcl/pf/pf_kdtree.hpp"
#include "nav2_amcl/pf/pf_pdf.hpp"
#include "nav2_amcl/portable_utils.hpp"

static int pf_resample_limit(pf_t * pf, int k);

pf_t * pf_alloc(
  int min_samples, int max_samples,
  double alpha_slow, double alpha_fast,
  pf_init_model_fn_t random_pose_fn)
{
  int i, j;
  pf_t * pf;
  pf_sample_set_t * set;
  pf_sample_t * sample;

  pf = calloc(1, sizeof(pf_t));

  pf->random_pose_fn = random_pose_fn;
  pf->min_samples = min_samples;
  pf->max_samples = max_samples;
  pf->pop_err = 0.01;
  pf->pop_z = 3;
  pf->dist_threshold = 0.5;
  pf->current_set = 0;

  for (j = 0; j < 2; j++) {
    set = pf->sets + j;
    set->sample_count = max_samples;
    set->samples = calloc(max_samples, sizeof(pf_sample_t));

    for (i = 0; i < set->sample_count; i++) {
      sample = set->samples + i;
      sample->pose.v[0] = 0.0;
      sample->pose.v[1] = 0.0;
      sample->pose.v[2] = 0.0;
      sample->weight = 1.0 / max_samples;
    }

    set->kdtree = pf_kdtree_alloc(3 * max_samples);
    set->cluster_count = 0;
    set->cluster_max_count = max_samples;
    set->clusters = calloc(set->cluster_max_count, sizeof(pf_cluster_t));
    set->mean = pf_vector_zero();
    set->cov = pf_matrix_zero();
  }

  pf->w_slow = 0.0;
  pf->w_fast = 0.0;
  pf->alpha_slow = alpha_slow;
  pf->alpha_fast = alpha_fast;

  pf_init_converged(pf);

  return pf;
}

void pf_free(pf_t * pf)
{
  int i;

  for (i = 0; i < 2; i++) {
    free(pf->sets[i].clusters);
    pf_kdtree_free(pf->sets[i].kdtree);
    free(pf->sets[i].samples);
  }
  free(pf);
}

void pf_init(pf_t * pf, pf_vector_t mean, pf_matrix_t cov)
{
  int i;
  pf_sample_set_t * set;
  pf_sample_t * sample;
  pf_pdf_gaussian_t * pdf;

  set = pf->sets + pf->current_set;
  pf_kdtree_clear(set->kdtree);
  set->sample_count = pf->max_samples;
  pdf = pf_pdf_gaussian_alloc(mean, cov);

  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;
    sample->weight = 1.0 / pf->max_samples;
    sample->pose = pf_pdf_gaussian_sample(pdf);
    pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
  }

  pf->w_slow = pf->w_fast = 0.0;
  pf_pdf_gaussian_free(pdf);
  pf_cluster_stats(pf, set);
  pf_init_converged(pf);
}

void pf_init_model(pf_t * pf, pf_init_model_fn_t init_fn, void * init_data)
{
  int i;
  pf_sample_set_t * set;
  pf_sample_t * sample;

  set = pf->sets + pf->current_set;
  pf_kdtree_clear(set->kdtree);
  set->sample_count = pf->max_samples;

  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;
    sample->weight = 1.0 / pf->max_samples;
    sample->pose = (*init_fn)(init_data);
    pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
  }

  pf->w_slow = pf->w_fast = 0.0;
  pf_cluster_stats(pf, set);
  pf_init_converged(pf);
}

void pf_init_converged(pf_t * pf)
{
  pf_sample_set_t * set;
  set = pf->sets + pf->current_set;
  set->converged = 0;
  pf->converged = 0;
}

int pf_update_converged(pf_t * pf)
{
  int i;
  pf_sample_set_t * set;
  pf_sample_t * sample;

  set = pf->sets + pf->current_set;
  double mean_x = 0, mean_y = 0;

  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;
    mean_x += sample->pose.v[0];
    mean_y += sample->pose.v[1];
  }
  mean_x /= set->sample_count;
  mean_y /= set->sample_count;

  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;
    if (fabs(sample->pose.v[0] - mean_x) > pf->dist_threshold ||
      fabs(sample->pose.v[1] - mean_y) > pf->dist_threshold)
    {
      set->converged = 0;
      pf->converged = 0;
      return 0;
    }
  }
  set->converged = 1;
  pf->converged = 1;
  return 1;
}

void pf_update_sensor(pf_t * pf, pf_sensor_model_fn_t sensor_fn, void * sensor_data)
{
  int i;
  pf_sample_set_t * set;
  pf_sample_t * sample;
  double total;

  set = pf->sets + pf->current_set;
  total = (*sensor_fn)(sensor_data, set);

  if (total > 0.0) {
    double w_avg = 0.0;
    for (i = 0; i < set->sample_count; i++) {
      sample = set->samples + i;
      w_avg += sample->weight;
      sample->weight /= total;
    }
    w_avg /= set->sample_count;
    if (pf->w_slow == 0.0) {
      pf->w_slow = w_avg;
    } else {
      pf->w_slow += pf->alpha_slow * (w_avg - pf->w_slow);
    }
    if (pf->w_fast == 0.0) {
      pf->w_fast = w_avg;
    } else {
      pf->w_fast += pf->alpha_fast * (w_avg - pf->w_fast);
    }
  } else {
    for (i = 0; i < set->sample_count; i++) {
      sample = set->samples + i;
      sample->weight = 1.0 / set->sample_count;
    }
  }
}

void pf_update_resample(pf_t * pf, void * random_pose_data)
{
  int i;
  double total;
  pf_sample_set_t * set_a, * set_b;
  pf_sample_t * sample_a, * sample_b;
  double * c;
  double w_diff;

  set_a = pf->sets + pf->current_set;
  set_b = pf->sets + (pf->current_set + 1) % 2;

  c = (double *)malloc(sizeof(double) * (set_a->sample_count + 1));
  c[0] = 0.0;
  for (i = 0; i < set_a->sample_count; i++) {
    c[i + 1] = c[i] + set_a->samples[i].weight;
  }

  pf_kdtree_clear(set_b->kdtree);
  total = 0;
  set_b->sample_count = 0;

  w_diff = 1.0 - pf->w_fast / pf->w_slow;
  if (w_diff < 0.0) {
    w_diff = 0.0;
  }

  while (set_b->sample_count < pf->max_samples) {
    sample_b = set_b->samples + set_b->sample_count++;

    if (drand48() < w_diff) {
      sample_b->pose = (pf->random_pose_fn)(random_pose_data);
    } else {
      double r;
      r = drand48();
      for (i = 0; i < set_a->sample_count; i++) {
        if ((c[i] <= r) && (r < c[i + 1])) {
          break;
        }
      }
      assert(i < set_a->sample_count);

      sample_a = set_a->samples + i;
      assert(sample_a->weight > 0);
      sample_b->pose = sample_a->pose;
    }

    sample_b->weight = 1.0;
    total += sample_b->weight;
    pf_kdtree_insert(set_b->kdtree, sample_b->pose, sample_b->weight);

    if (set_b->sample_count > pf_resample_limit(pf, set_b->kdtree->leaf_count)) {
      break;
    }
  }

  if (w_diff > 0.0) {
    pf->w_slow = pf->w_fast = 0.0;
  }

  for (i = 0; i < set_b->sample_count; i++) {
    sample_b = set_b->samples + i;
    sample_b->weight /= total;
  }

  pf_cluster_stats(pf, set_b);
  pf->current_set = (pf->current_set + 1) % 2;
  pf_update_converged(pf);

  free(c);
}

int pf_resample_limit(pf_t * pf, int k)
{
  double a, b, c, x;
  int n;

  if (k <= 1) {
    return pf->max_samples;
  }

  a = 1;
  b = 2 / (9 * ((double) k - 1));
  c = sqrt(2 / (9 * ((double) k - 1))) * pf->pop_z;
  x = a - b + c;

  n = (int) ceil((k - 1) / (2 * pf->pop_err) * x * x * x);

  if (n < pf->min_samples) {
    return pf->min_samples;
  }
  if (n > pf->max_samples) {
    return pf->max_samples;
  }

  return n;
}

void pf_cluster_stats(pf_t * pf, pf_sample_set_t * set)
{
  (void)pf;
  int i, j, k, cidx;
  pf_sample_t * sample;
  pf_cluster_t * cluster;
  double m[4], c[2][2];
  double weight;

  pf_kdtree_cluster(set->kdtree);
  set->cluster_count = 0;

  for (i = 0; i < set->cluster_max_count; i++) {
    cluster = set->clusters + i;
    cluster->weight = 0;
    cluster->mean = pf_vector_zero();
    cluster->cov = pf_matrix_zero();

    for (j = 0; j < 4; j++) {
      cluster->m[j] = 0.0;
    }
    for (j = 0; j < 2; j++) {
      for (k = 0; k < 2; k++) {
        cluster->c[j][k] = 0.0;
      }
    }
  }

  weight = 0.0;
  set->mean = pf_vector_zero();
  set->cov = pf_matrix_zero();
  for (j = 0; j < 4; j++) {
    m[j] = 0.0;
  }
  for (j = 0; j < 2; j++) {
    for (k = 0; k < 2; k++) {
      c[j][k] = 0.0;
    }
  }

  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;
    cidx = pf_kdtree_get_cluster(set->kdtree, sample->pose);
    assert(cidx >= 0);
    if (cidx >= set->cluster_max_count) {
      continue;
    }
    if (cidx + 1 > set->cluster_count) {
      set->cluster_count = cidx + 1;
    }

    cluster = set->clusters + cidx;
    cluster->weight += sample->weight;
    weight += sample->weight;

    cluster->m[0] += sample->weight * sample->pose.v[0];
    cluster->m[1] += sample->weight * sample->pose.v[1];
    cluster->m[2] += sample->weight * cos(sample->pose.v[2]);
    cluster->m[3] += sample->weight * sin(sample->pose.v[2]);

    m[0] += sample->weight * sample->pose.v[0];
    m[1] += sample->weight * sample->pose.v[1];
    m[2] += sample->weight * cos(sample->pose.v[2]);
    m[3] += sample->weight * sin(sample->pose.v[2]);

    for (j = 0; j < 2; j++) {
      for (k = 0; k < 2; k++) {
        cluster->c[j][k] += sample->weight * sample->pose.v[j] * sample->pose.v[k];
        c[j][k] += sample->weight * sample->pose.v[j] * sample->pose.v[k];
      }
    }
  }

  for (i = 0; i < set->cluster_count; i++) {
    cluster = set->clusters + i;

    cluster->mean.v[0] = cluster->m[0] / cluster->weight;
    cluster->mean.v[1] = cluster->m[1] / cluster->weight;
    cluster->mean.v[2] = atan2(cluster->m[3], cluster->m[2]);
    cluster->cov = pf_matrix_zero();

    for (j = 0; j < 2; j++) {
      for (k = 0; k < 2; k++) {
        cluster->cov.m[j][k] = cluster->c[j][k] / cluster->weight -
          cluster->mean.v[j] * cluster->mean.v[k];
      }
    }

    cluster->cov.m[2][2] = -2 * log(
      sqrt(
        cluster->m[2] * cluster->m[2] +
        cluster->m[3] * cluster->m[3]));
  }

  set->mean.v[0] = m[0] / weight;
  set->mean.v[1] = m[1] / weight;
  set->mean.v[2] = atan2(m[3], m[2]);

  for (j = 0; j < 2; j++) {
    for (k = 0; k < 2; k++) {
      set->cov.m[j][k] = c[j][k] / weight - set->mean.v[j] * set->mean.v[k];
    }
  }

  set->cov.m[2][2] = -2 * log(sqrt(m[2] * m[2] + m[3] * m[3]));
}

int pf_get_cluster_stats(
  pf_t * pf, int clabel, double * weight,
  pf_vector_t * mean, pf_matrix_t * cov)
{
  pf_sample_set_t * set;
  pf_cluster_t * cluster;

  set = pf->sets + pf->current_set;

  if (clabel >= set->cluster_count) {
    return 0;
  }
  cluster = set->clusters + clabel;

  *weight = cluster->weight;
  *mean = cluster->mean;
  *cov = cluster->cov;

  return 1;
}
