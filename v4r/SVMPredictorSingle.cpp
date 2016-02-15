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
 * @file SVMPredictorSingle.cpp
 * @author Andreas Richtsfeld
 * @date August 2011
 * @version 0.1
 * @brief Predict results from a trained SVM.
 */

#include "SVMPredictorSingle.h"

namespace svm
{

#define max(x,y) (((x)>(y))?(x):(y))
#define min(x,y) (((x)<(y))?(x):(y))

  
/**
 * @brief Constructor of SVMPredictorSingle
 */
SVMPredictorSingle::SVMPredictorSingle(std::string path, std::string filename)
{
  if(path.size() == 0)
    path = SVM_MODEL;
  filename = path + filename;
  
  max_nr_attr = 64;
  predict_probability = true;

  if((model = svm_load_model(filename.c_str())) == 0)
    printf("[SVMPredictorSingle::SVMPredictorSingle] Error: Can't open model file: %s\n", filename.c_str());
  
#ifdef DEBUG
  else
    printf("[SVMPredictorSingle] Open model file: %s\n", filename.c_str());
#endif
    
  // allocate memory for model (and nodes)
  node = (struct svm_node *) malloc(max_nr_attr*sizeof(struct svm_node));

  if(predict_probability) {
    if(svm_check_probability_model(model) == 0) {
      printf("[SVMPredictorSingle::SVMPredictorSingle] Error: Model does not support probability estimates.\n");
      return;
    }
  }
  else {
    if(svm_check_probability_model(model) == 0)
      printf("[SVMPredictorSingle::SVMPredictorSingle] Warning: Model supports probability estimates, but disabled in prediction.");
  }
}

/**
 * @brief Destructor of SVMPredictorSingle
 */
SVMPredictorSingle::~SVMPredictorSingle()
{
  svm_free_and_destroy_model(&model);
  free(node);
}


/**
 * @brief Read a line from file.
 * @param input File to read from.
 * @return Returns one line of the file.
 */
char* SVMPredictorSingle::readSCline(FILE *input)
{
  int len;
  
  if(fgets(scline, max_line_len, input) == NULL)
    return NULL;

  while(strrchr(scline,'\n') == NULL) {
    max_line_len *= 2;
    scline = (char *) realloc(scline, max_line_len);
    len = (int) strlen(scline);
    if(fgets(scline+len,max_line_len-len,input) == NULL)
      break;
  }
  return scline;
}

/**
 * @brief Predict with SVM for a given vector.
 * @param type Type of feature vector: Type of svm-model (1, 2, ...)
 * @param vec Feature vector for the SVM.
 * @param prob Probability of correct prediction for each class.
 * @return Returns the prediction label
 */
bool SVMPredictorSingle::process(int type, std::vector<double> &vec, std::vector<double> &prob)
{
  int svm_type = svm_get_svm_type(model);
  int nr_class = svm_get_nr_class(model);
  double *prob_estimates = NULL;
  int j;

  if(predict_probability) {
    if (svm_type == NU_SVR || svm_type == EPSILON_SVR)
      printf("Prob. model for test data: target value = predicted value + z,\nz: Laplace distribution e^(-|z|/sigma)/(2sigma),sigma=%g\n", svm_get_svr_probability(model));
    else
      prob_estimates = (double *) malloc(nr_class*sizeof(double));
  }

  // we copy now the feature vector
  double predict_label;
  for(unsigned idx = 0; idx < vec.size(); idx++) {
    node[idx].index = idx+1;
    node[idx].value = vec[idx];
    node[idx+1].index = -1;
  }

  if (predict_probability && (svm_type==C_SVC || svm_type==NU_SVC)) {
    predict_label = svm_predict_probability(model, node, prob_estimates);
    for(j=0;j<nr_class;j++)
      prob.push_back(prob_estimates[j]);
  }
  else
    predict_label = svm_predict(model, node);

  if(predict_probability)
    free(prob_estimates);
  
  return predict_label;
}


/**
 * @brief Set scaling for structural level with supplied model.
 * @param _scale on/off
 */
void SVMPredictorSingle::setScaling(bool _scale, std::string path, std::string filename)
{
  if(path == "")
    path = SVM_MODEL;
  filename = path + filename;

    scale = _scale;
  if(!scale) return;
  
  double y_lower, y_upper;
  double y_max, y_min;

  scline = NULL;
  max_line_len = 1024;
  lower = -1.0;
  upper = 1.0;
  y_max = -DBL_MAX;
  y_min = DBL_MAX;
  
  int idx, c;
  int max_index = 0;
  double fmin, fmax;
  std::FILE *fp_restore = NULL;

  scline = (char *) malloc(max_line_len*sizeof(char)); // allocate line memory

  fp_restore = fopen(filename.c_str(), "r");
  if(fp_restore==NULL) {
    printf("[SVMPredictorSingle::setScaling] Error: Can't open file: %s.\n", filename.c_str());
    scale = false;
    return;
  }
#ifdef DEBUG
  else
    printf("[SVMPredictorSingle] Opened scaling parameter file: %s\n", filename.c_str());
#endif
  c = fgetc(fp_restore);
  if(c == 'y') {
    readSCline(fp_restore);
    readSCline(fp_restore);
    readSCline(fp_restore);
  }
  readSCline(fp_restore);
  readSCline(fp_restore);

  while(fscanf(fp_restore,"%d %*f %*f\n",&idx) == 1)
    max_index = max(idx,max_index);
  rewind(fp_restore);
  
  feature_max.resize(max_index);
  feature_min.resize(max_index);

  for(int i=0;i<max_index;i++) {
    feature_max[i]=-DBL_MAX;
    feature_min[i]=DBL_MAX;
  }
  
  if((c = fgetc(fp_restore)) == 'y') {
    fscanf(fp_restore, "%lf %lf\n", &y_lower, &y_upper);
    fscanf(fp_restore, "%lf %lf\n", &y_min, &y_max);
  }
  else
    ungetc(c, fp_restore);

  if (fgetc(fp_restore) == 'x') {
    fscanf(fp_restore, "%lf %lf\n", &lower, &upper);
    while(fscanf(fp_restore,"%d %lf %lf\n",&idx,&fmin,&fmax)==3)
    {
      if(idx<=max_index) {
        feature_min[idx-1] = fmin;
        feature_max[idx-1] = fmax;
      }
    }
  }
  fclose(fp_restore);
  free(scline);
}

void SVMPredictorSingle::scaleValues(std::vector<double> &val)
{
  std::vector<double> scaled_val;
  for(unsigned index=0; index<val.size(); index++)
  {
    double value = val[index];
    if(feature_max[index] == feature_min[index]) {
      printf("[SVMPredictorSingle::scaleValues] Warning: feature_max[index] == feature_min[index]: %4.3f\n", feature_max[index]);
      return;
    }

    if(value == feature_min[index])
      value = lower;
    else if(value == feature_max[index])
      value = upper;
    else
      value = lower + (upper-lower) * 
        (value-feature_min[index])/
        (feature_max[index]-feature_min[index]);

    scaled_val.push_back(value);
  }
  val = scaled_val;
}

/** HACK: We do not allow small patches to be connected to two big patches. **/
void SVMPredictorSingle::CheckSmallPatches(surface::View *view, unsigned max_size)
{
  for(unsigned i=0; i<view->surfaces.size(); i++) {
    if(view->surfaces[i]->indices.size() < max_size) {
      int biggest = -1;
      double biggest_value = 0.0;
      for(unsigned j=0; j<view->relations.size(); j++) {
        if(view->relations[j].id_0 == i || view->relations[j].id_1 == i) {
          if(view->relations[j].rel_probability[1] < biggest_value) {
            view->relations[j].rel_probability[0] = 0.999;
            view->relations[j].rel_probability[1] = 0.001;
          }
          else {
            biggest_value = view->relations[j].rel_probability[1];
            if(biggest != -1) {
              view->relations[biggest].rel_probability[0] = 0.999;
              view->relations[biggest].rel_probability[1] = 0.001;
            }
            biggest = j;
          }
        }
      }
    }
  }      
}


/**
 * @brief Process the relation extraction algorithm
 * @param type Type of SVM relation
 * @param val Feature vector of relation
 * @param prob Probability for correct predicton
 */
bool SVMPredictorSingle::getResult(int type,
                             std::vector<double> &val,
                             std::vector<double> &prob)
{
  if(scale)
    scaleValues(val);
  
  return process(type, val, prob);
}

/** 
 * @brief Classify all feature vectors of a view
 * @param view View with surface models 
 * @param type Type of relations to process (1=structural / 2= assembly level)
 */
bool SVMPredictorSingle::classify(surface::View *view, unsigned type)
{
  for(unsigned i=0; i<view->relations.size(); i++) {
    if(view->relations[i].type == type)
      view->relations[i].prediction = getResult(view->relations[i].type, 
                                                view->relations[i].rel_value, 
                                                view->relations[i].rel_probability);
  }  
  CheckSmallPatches(view, 30);
  return true;
}


} 











