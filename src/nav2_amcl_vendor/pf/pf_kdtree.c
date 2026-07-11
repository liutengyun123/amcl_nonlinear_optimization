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
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "nav2_amcl/pf/pf_kdtree.hpp"
#include "nav2_amcl/pf/pf_vector.hpp"

static int pf_kdtree_equal(pf_kdtree_t * self, int key_a[], int key_b[]);
static pf_kdtree_node_t * pf_kdtree_insert_node(
  pf_kdtree_t * self, pf_kdtree_node_t * parent,
  pf_kdtree_node_t * node, int key[], double value);
static pf_kdtree_node_t * pf_kdtree_find_node(
  pf_kdtree_t * self, pf_kdtree_node_t * node,
  int key[]);
static void pf_kdtree_cluster_node(
  pf_kdtree_t * self, pf_kdtree_node_t * node, int depth);

pf_kdtree_t * pf_kdtree_alloc(int max_size)
{
  pf_kdtree_t * self;

  self = calloc(1, sizeof(pf_kdtree_t));
  self->size[0] = 0.50;
  self->size[1] = 0.50;
  self->size[2] = (10 * M_PI / 180);
  self->root = NULL;
  self->node_count = 0;
  self->node_max_count = max_size;
  self->nodes = calloc(self->node_max_count, sizeof(pf_kdtree_node_t));
  self->leaf_count = 0;

  return self;
}

void pf_kdtree_free(pf_kdtree_t * self)
{
  free(self->nodes);
  free(self);
}

void pf_kdtree_clear(pf_kdtree_t * self)
{
  self->root = NULL;
  self->leaf_count = 0;
  self->node_count = 0;
}

void pf_kdtree_insert(pf_kdtree_t * self, pf_vector_t pose, double value)
{
  int key[3];

  key[0] = floor(pose.v[0] / self->size[0]);
  key[1] = floor(pose.v[1] / self->size[1]);
  key[2] = floor(pose.v[2] / self->size[2]);

  self->root = pf_kdtree_insert_node(self, NULL, self->root, key, value);
}

int pf_kdtree_get_cluster(pf_kdtree_t * self, pf_vector_t pose)
{
  int key[3];
  pf_kdtree_node_t * node;

  key[0] = floor(pose.v[0] / self->size[0]);
  key[1] = floor(pose.v[1] / self->size[1]);
  key[2] = floor(pose.v[2] / self->size[2]);

  node = pf_kdtree_find_node(self, self->root, key);
  if (node == NULL) {
    return -1;
  }
  return node->cluster;
}

int pf_kdtree_equal(pf_kdtree_t * self, int key_a[], int key_b[])
{
  (void)self;

  if (key_a[0] != key_b[0]) {
    return 0;
  }
  if (key_a[1] != key_b[1]) {
    return 0;
  }
  if (key_a[2] != key_b[2]) {
    return 0;
  }

  return 1;
}

pf_kdtree_node_t * pf_kdtree_insert_node(
  pf_kdtree_t * self, pf_kdtree_node_t * parent,
  pf_kdtree_node_t * node, int key[], double value)
{
  int i;
  int split, max_split;

  if (node == NULL) {
    assert(self->node_count < self->node_max_count);
    node = self->nodes + self->node_count++;
    memset(node, 0, sizeof(pf_kdtree_node_t));

    node->leaf = 1;
    if (parent == NULL) {
      node->depth = 0;
    } else {
      node->depth = parent->depth + 1;
    }

    for (i = 0; i < 3; i++) {
      node->key[i] = key[i];
    }

    node->value = value;
    self->leaf_count += 1;
  } else if (node->leaf) {
    if (pf_kdtree_equal(self, key, node->key)) {
      node->value += value;
    } else {
      max_split = 0;
      node->pivot_dim = -1;
      for (i = 0; i < 3; i++) {
        split = abs(key[i] - node->key[i]);
        if (split > max_split) {
          max_split = split;
          node->pivot_dim = i;
        }
      }
      assert(node->pivot_dim >= 0);

      node->pivot_value = (key[node->pivot_dim] + node->key[node->pivot_dim]) / 2.0;

      if (key[node->pivot_dim] < node->pivot_value) {
        node->children[0] = pf_kdtree_insert_node(self, node, NULL, key, value);
        node->children[1] = pf_kdtree_insert_node(self, node, NULL, node->key, node->value);
      } else {
        node->children[0] = pf_kdtree_insert_node(self, node, NULL, node->key, node->value);
        node->children[1] = pf_kdtree_insert_node(self, node, NULL, key, value);
      }

      node->leaf = 0;
      self->leaf_count -= 1;
    }
  } else {
    assert(node->children[0] != NULL);
    assert(node->children[1] != NULL);

    if (key[node->pivot_dim] < node->pivot_value) {
      pf_kdtree_insert_node(self, node, node->children[0], key, value);
    } else {
      pf_kdtree_insert_node(self, node, node->children[1], key, value);
    }
  }

  return node;
}

pf_kdtree_node_t * pf_kdtree_find_node(
  pf_kdtree_t * self, pf_kdtree_node_t * node, int key[])
{
  if (node->leaf) {
    if (pf_kdtree_equal(self, key, node->key)) {
      return node;
    } else {
      return NULL;
    }
  } else {
    assert(node->children[0] != NULL);
    assert(node->children[1] != NULL);

    if (key[node->pivot_dim] < node->pivot_value) {
      return pf_kdtree_find_node(self, node->children[0], key);
    } else {
      return pf_kdtree_find_node(self, node->children[1], key);
    }
  }

  return NULL;
}

void pf_kdtree_cluster(pf_kdtree_t * self)
{
  int i;
  int queue_count, cluster_count;
  pf_kdtree_node_t ** queue, * node;

  queue_count = 0;
  queue = calloc(self->node_count, sizeof(queue[0]));

  for (i = 0; i < self->node_count; i++) {
    node = self->nodes + i;
    if (node->leaf) {
      node->cluster = -1;
      assert(queue_count < self->node_count);
      queue[queue_count++] = node;
      assert(node == pf_kdtree_find_node(self, self->root, node->key));
    }
  }

  cluster_count = 0;

  while (queue_count > 0) {
    node = queue[--queue_count];

    if (node->cluster >= 0) {
      continue;
    }

    node->cluster = cluster_count++;
    pf_kdtree_cluster_node(self, node, 0);
  }

  free(queue);
}

void pf_kdtree_cluster_node(pf_kdtree_t * self, pf_kdtree_node_t * node, int depth)
{
  int i;
  int nkey[3];
  pf_kdtree_node_t * nnode;

  (void)depth;

  for (i = 0; i < 3 * 3 * 3; i++) {
    nkey[0] = node->key[0] + (i / 9) - 1;
    nkey[1] = node->key[1] + ((i % 9) / 3) - 1;
    nkey[2] = node->key[2] + ((i % 9) % 3) - 1;

    nnode = pf_kdtree_find_node(self, self->root, nkey);
    if (nnode == NULL) {
      continue;
    }

    assert(nnode->leaf);

    if (nnode->cluster >= 0) {
      assert(nnode->cluster == node->cluster);
      continue;
    }

    nnode->cluster = node->cluster;
    pf_kdtree_cluster_node(self, nnode, depth + 1);
  }
}
