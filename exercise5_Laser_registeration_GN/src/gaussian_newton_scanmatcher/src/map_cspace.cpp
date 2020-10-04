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

#include <queue>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "map.h"

class CellData
{
  public:
    map_t* map_;
    unsigned int i_, j_;
    unsigned int src_i_, src_j_;
};

/*
 * CachedDistanceMap is actually a distance map which is centered at (0,0)
 * each cell in this map stores the distance between the origin
 * this is for computation of distance between each grid and obstacles, we only need to check the table instead of computing sqrt in every iteration
*/
class CachedDistanceMap
{
  public:
    CachedDistanceMap(double scale, double max_dist) :
      distances_(NULL), scale_(scale), max_dist_(max_dist)
    {
      // cell number w.r.t maximum distance
      cell_radius_ = max_dist / scale;

      distances_ = new double *[cell_radius_+2];
      for(int i=0; i<=cell_radius_+1; i++)
      {
        distances_[i] = new double[cell_radius_+2];
        for(int j=0; j<=cell_radius_+1; j++)
        {
            distances_[i][j] = sqrt(i*i + j*j);
        }
      }
    }

    ~CachedDistanceMap()
    {
      if(distances_)
      {
        for(int i=0; i<=cell_radius_+1; i++)
            delete[] distances_[i];
        delete[] distances_;
      }
    }
    double** distances_;
    double scale_;
    double max_dist_;
    int cell_radius_;
};


bool operator<(const CellData& a, const CellData& b)
{
  return a.map_->cells[MAP_INDEX(a.map_, a.i_, a.j_)].occ_dist > a.map_->cells[MAP_INDEX(b.map_, b.i_, b.j_)].occ_dist;
}

CachedDistanceMap* get_distance_map(double scale, double max_dist)
{
  static CachedDistanceMap* cdm = NULL;

  if(!cdm || (cdm->scale_ != scale) || (cdm->max_dist_ != max_dist))
  {
    if(cdm)
      delete cdm;
    cdm = new CachedDistanceMap(scale, max_dist);
  }

  return cdm;
}

// compute the distance between the obstacle
/**
 * @brief enqueue
 * @param map       corresponding map
 * @param i         x coordinate in map
 * @param j         y coordinate in map
 * @param src_i     x coordinate of obstacle
 * @param src_j     y coordinate of obstacle
 * @param Q
 * @param cdm
 * @param marked
 */
void enqueue(map_t* map, unsigned int i, unsigned int j,
	     unsigned int src_i, unsigned int src_j,
	     std::priority_queue<CellData>& Q,
	     CachedDistanceMap* cdm,
	     unsigned char* marked)
{
  // if the cell is already visted, return
  if(marked[MAP_INDEX(map, i, j)])
    return;

  // find the distacne in distance map
  unsigned int di = abs(i - src_i);
  unsigned int dj = abs(j - src_j);
  double distance = cdm->distances_[di][dj];

  if(distance > cdm->cell_radius_)
    return;

  // convert to real distance
  map->cells[MAP_INDEX(map,i,j)].occ_dist = distance * map->resolution;

  double z = map->cells[MAP_INDEX(map,i,j)].occ_dist;
  map->cells[MAP_INDEX(map, i, j)].score = exp(-(z * z) /  (2 * map->likelihood_sigma * map->likelihood_sigma));

  CellData cell;
  cell.map_ = map;
  cell.i_ = i;
  cell.j_ = j;
  cell.src_i_ = src_i;
  cell.src_j_ = src_j;

  Q.push(cell);

  marked[MAP_INDEX(map, i, j)] = 1;
}


// Update the cspace distance values

/**
 * @brief map_update_cspace
 * update the distance in the map
 * the distance for obstacle is 0, the distance for other grids depend how far thay are from obstacle
 * use dfs to traverse the whole map
 * @param map
 * @param max_occ_dist
 */
void map_update_cspace(map_t *map, double max_occ_dist)
{
  unsigned char* marked;
  std::priority_queue<CellData> Q;

  marked = new unsigned char[map->size_x*map->size_y];
  memset(marked, 0, sizeof(unsigned char) * map->size_x*map->size_y);

  map->max_occ_dist = max_occ_dist;

  // get a distance map
  CachedDistanceMap* cdm = get_distance_map(map->resolution, map->max_occ_dist);

  map->min_score = exp(-max_occ_dist * max_occ_dist / (2 * map->likelihood_sigma * map->likelihood_sigma));

  // Enqueue all the obstacle cells
  CellData cell;
  cell.map_ = map;

  // for all grids, if it is a obstacle grid, set it to 0, otherwise set it to max_occ_dist
  for(int i=0; i<map->size_x; i++)
  {
      cell.src_i_ = cell.i_ = i;
      for(int j=0; j<map->size_y; j++)
      {
          if(map->cells[MAP_INDEX(map, i, j)].occ_state == CELL_STATUS_OCC)
          {
              map->cells[MAP_INDEX(map, i, j)].occ_dist = 0.0;
              map->cells[MAP_INDEX(map,i,j)].score = 1.0;
              cell.src_j_ = cell.j_ = j;
              marked[MAP_INDEX(map, i, j)] = 1;
              Q.push(cell);
          }
          else
              map->cells[MAP_INDEX(map, i, j)].occ_dist = max_occ_dist;
      }
  }

  while(!Q.empty())
  {
    CellData current_cell = Q.top();

    // dfs: up, down, left, right
    if(current_cell.i_ > 0)
      enqueue(map, current_cell.i_-1, current_cell.j_,
	      current_cell.src_i_, current_cell.src_j_,
	      Q, cdm, marked);

    if(current_cell.j_ > 0)
      enqueue(map, current_cell.i_, current_cell.j_-1,
	      current_cell.src_i_, current_cell.src_j_,
	      Q, cdm, marked);

    if((int)current_cell.i_ < map->size_x - 1)
      enqueue(map, current_cell.i_+1, current_cell.j_,
	      current_cell.src_i_, current_cell.src_j_,
	      Q, cdm, marked);

    if((int)current_cell.j_ < map->size_y - 1)
      enqueue(map, current_cell.i_, current_cell.j_+1,
	      current_cell.src_i_, current_cell.src_j_,
	      Q, cdm, marked);

    Q.pop();
  }

  delete[] marked;
}
