/**
 *  Copyright (C) 2012  
 *    Andreas Richtsfeld, Johann Prankl, Thomas Mörwald
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1170 Vienn, Austria
 *    ari(at)acin.tuwien.ac.at
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/
 */

/**
 * @file GraphCut.cpp
 * @author Andreas Richtsfeld
 * @date July 2011
 * @version 0.1
 * @brief GraphCut algorithm to cut graphs into pieces for segmentation.
 */


#include "GraphCut.h"

#ifndef GC_DEBUG
#define GC_DEBUG false
#endif

namespace gc
{

/**
 * @brief Constructor of GraphCut
 */
GraphCut::GraphCut()
{
  initialized = false;
  processed = false;
  createAllRelations = false;     // create fully connected graph to avoid segmentation fault
  print = false;
}


/**
 * @brief Destructor of GraphCut
 */
GraphCut::~GraphCut()
{}


bool GraphCut::init(surface::View *_view)
{
  view = _view;
  if(view->relations.size() < 1) {
    printf("[GraphCut::init] Warning: No relations available!\n");
    return false;
  }
  
  if(createAllRelations) {
    unsigned maxID = 0;
    for(unsigned i=0; i<view->relations.size(); i++) {
      if(view->relations[i].id_0 > maxID)
        maxID = view->relations[i].id_0;
      if(view->relations[i].id_1 > maxID)
        maxID = view->relations[i].id_1;
    }
    for(unsigned i=0; i<maxID; i++) {
      for(unsigned j=i+1; j<=maxID; j++) {
        bool foundRelation = false;
        for(int k=0; k<(int)view->relations.size(); k++) {
          if((view->relations[k].id_0 == i && view->relations[k].id_1 == j) ||
             (view->relations[k].id_1 == i && view->relations[k].id_0 == j))
            foundRelation = true;
        }
        if(!foundRelation) {
          surface::Relation r;
          r.id_0 = i; 
          r.id_1 = j;
          r.rel_probability.push_back(1.0f);
          r.rel_probability.push_back(0.0f);
          r.groundTruth = -1;
          view->relations.push_back(r);
        }
      }
    }
  }
  
  std::vector<gc::Edge> e;
  Graph graph(view->surfaces.size(), view->relations);
  graph.BuildFromSVM(e, num_edges);

  edges = new gc::Edge[num_edges];
  for(unsigned i=0; i<num_edges; i++)
    edges[i] = e[i];

  if(num_edges == 0) { 
    initialized = false;
    return false;
  } 
  else {
    u = new universe(num_edges);
    initialized = true;
    if(GC_DEBUG) 
      printf("[GraphCut::Initialize] num_edges: %u\n", num_edges);
  }
  return true;
}

/**
 * @brief Compare edges for sort function.
 * @param a Edge a
 * @param b Edge b
 * @return Returns true if a.w < b.w
 */
bool smallerEdge(const gc::Edge &a, const gc::Edge &b)
{
  return a.w < b.w;
}

/**
 * @brief Graph Cutting
 */
void GraphCut::process()
{
  if(!initialized) {
    printf("[GraphCut::process] Error: Graph is not initialized!\n");
    return;
  }

  if(GC_DEBUG) printf("[GraphCut::process] Start processing.\n");

  // sort edges by weight  
  std::sort(edges, edges + num_edges, smallerEdge);

  // init thresholds
  float *threshold = new float[num_edges];
  for (unsigned i = 0; i < num_edges; i++)
    threshold[i] = THRESHOLD(1, THRESHOLD_CONSTANT);
  
  if(GC_DEBUG) printf("THRESHOLD: %4.3f\n", threshold[0]);
  
  // for each edge, in non-decreasing weight order...
  for (unsigned i = 0; i < num_edges; i++) {
     if(GC_DEBUG) {
      u->printAll();
      for (unsigned j = 0; j < num_edges; j++) {
        printf("get edge %i\n", j);
        gc::Edge *pedge = &edges[j];
        printf("  all edges: %u:", j);
        printf("  %u-%u with thrd: %4.3f", pedge->a, pedge->b, threshold[j]);
        printf("  => universe: %u-%u\n", u->find(pedge->a), u->find(pedge->b));
      }
    }
    
    if(GC_DEBUG) printf("\nstart with edge %u: ", i);
    gc::Edge *pedge = &edges[i];
    int a = u->find(pedge->a);    // components conected by this edge
    int b = u->find(pedge->b);
    if(GC_DEBUG) printf("node: %u-%u / universe: %u-%u: ", pedge->a, pedge->b, a, b);
    
    if (a != b) {
      if(GC_DEBUG)
        printf("weight: %4.3f and thds: %4.3f-%4.3f\n", pedge->w, threshold[a], threshold[b]);
      
      if ((pedge->w <= threshold[a]) && (pedge->w <= threshold[b])) 
      {
        u->join(a, b);
        a = u->find(a);
        if(GC_DEBUG)
          printf("  => join: threshold[%u] = w(%4.3f) + %4.3f => ", a, pedge->w, threshold[a]);
        threshold[a] = pedge->w + THRESHOLD(u->size(a), THRESHOLD_CONSTANT);
        if(GC_DEBUG)
          printf("%4.3f (size: %u)\n", threshold[a], u->size(a));
      }
    }
  }

  int num_components = u->num_sets();
  if(GC_DEBUG) printf("[GraphCut::process] Number of components: %u => What does that mean?\n", num_components);

  // copy graph cut groups
  view->graphCutGroups.clear();
  unsigned cut_labels[view->surfaces.size()];             // cut-ids for all models
  std::set<unsigned> graphCutLabels;                      // all graph cut labels
  for(unsigned i=0; i<view->surfaces.size(); i++) {
    int cut_id = u->find(i);
    cut_labels[i] = cut_id;
    if(graphCutLabels.find(cut_id) == graphCutLabels.end())
      graphCutLabels.insert(cut_id);
  }
  
  std::set<unsigned>::iterator it;
  for(it = graphCutLabels.begin(); it != graphCutLabels.end(); it++) {
    std::vector<unsigned> cluster;
    for(unsigned i=0; i< view->surfaces.size(); i++)
      if(*it == cut_labels[i])
        cluster.push_back(i);
    view->graphCutGroups.push_back(cluster);
  }
    
  for(unsigned i=0; i<view->graphCutGroups.size(); i++)
    for(unsigned j=0; j<view->graphCutGroups[i].size(); j++)
      view->surfaces[view->graphCutGroups[i][j]]->label = i;
  
  if(print) {
    printf("[GraphCut::process] Resulting groups:\n");
    for(unsigned i=0; i<view->relations.size(); i++) {
      if(view->relations[i].rel_probability[1] != 0.0)
        printf("  p(%u, %u) = %4.3f\n", view->relations[i].id_0, view->relations[i].id_1, view->relations[i].rel_probability[1]);
    }
    
    printf("[GraphCut::process] Resulting groups:\n");
    for(unsigned i=0; i<view->graphCutGroups.size(); i++) {
      printf("  Group %u: ", i);
      for(unsigned j=0; j<view->graphCutGroups[i].size(); j++)
        printf("%u ", view->graphCutGroups[i][j]);
      printf("\n");
    }
  }
      
  delete threshold;
  delete []edges;
  initialized = false;
  processed = true;
}

} 











