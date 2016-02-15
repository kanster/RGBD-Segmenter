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
 * @file SVMPredictorSingle.h
 * @author Andreas Richtsfeld
 * @date August 2011
 * @version 0.1
 * @brief Predict results from a trained SVM.
 */

#ifndef SVM_SVM_PREDICTOR_SINGLE_H
#define SVM_SVM_PREDICTOR_SINGLE_H

#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <stdlib.h>
#include <stdio.h>
#include <float.h>
#include <ostream>

#include "svm.h"
#include "SurfaceModel.hpp"

namespace svm
{
  
  
/**
 * @brief Class SVMPredictorSingle: 
 */
class SVMPredictorSingle
{
 
public:
  
private:

  bool scale;                                   ///< set scaling on/off
  char *scline;
  int max_line_len;
  double lower, upper;                          ///< lower/upper limits
  std::vector<double> feature_max;              ///< maximum feature value for scaling
  std::vector<double> feature_min;              ///< minimum feature value for scaling
  
  struct svm_node *node;                        ///< node of svm
  int max_nr_attr;                              ///< Maximum attributes = maximum size of feature vector

  struct svm_model *model;                      ///< SVM model
  
  bool predict_probability;                     ///< Predict with probability values

  char* readSCline(FILE *input);
  bool process(int type, std::vector<double> &vec, std::vector<double> &prob);
  void scaleValues(std::vector<double> &val);
  void CheckSmallPatches(surface::View *view, unsigned max_size);
  bool getResult(int type, std::vector<double> &val, std::vector<double> &prob);
  
public:
  SVMPredictorSingle(std::string path, std::string filename); 
  ~SVMPredictorSingle();
  
  /** Set scaling of result vector **/
  void setScaling(bool _scale, std::string path, std::string filename);

  /** Classification of all feature vectors of a view of a specific type (1=neighboring / 2=non-neighboring **/
  bool classify(surface::View *view, unsigned type);
};

}

#endif

