/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 * Modified on: Jul. 7, 2019
 % by Bo Lin
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

static std::default_random_engine generator;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 0;  // TODO: Set the number of particles
  num_particles = 100;
  
  // Random Gaussian noise for particle
  std::normal_distribution<float> x_gaussian_par(0.0, std[0]);
  std::normal_distribution<float> y_gaussian_par(0.0, std[1]);
  std::normal_distribution<float> theta_gaussian_par(0.0, std[2]);
  
  // For each particle, initialize it with position estimate and noise, weight
  for (int i=0; i<num_particles; ++i){
    Particle p;
    p.id=i;  
    p.x=x;
    p.x+=x_gaussian_par(generator);
    p.y=y;
    p.y+=y_gaussian_par(generator);
    p.theta=theta;
    p.theta+=theta_gaussian_par(generator);
    p.weight=1.0;
    particles.push_back(p);
  }
  // Set the initialized flag
  is_initialized=true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  
  // Random Gaussian noise for motion 
  std::normal_distribution<double> x_gaussian_meas(0.0, std_pos[0]);
  std::normal_distribution<double> y_gaussian_meas(0.0, std_pos[1]);
  std::normal_distribution<double> theta_gaussian_meas(0.0, std_pos[2]);

  // For each particle, update its position based on motion model as a function of velocity and yaw rate
  for(int i=0; i<num_particles; ++i){
    // if the yaw_rate is too small, treat it separately to avoid divide by zero problem
    if (fabs(yaw_rate)<1e-4) {
      particles[i].x+=velocity*delta_t*cos(particles[i].theta);
      particles[i].y+=velocity*delta_t*sin(particles[i].theta);
      particles[i].theta+=0;
    } else {
      particles[i].x+=velocity/yaw_rate*(sin(particles[i].theta+yaw_rate*delta_t)-sin(particles[i].theta));
      particles[i].y+=velocity/yaw_rate*(cos(particles[i].theta)-cos(particles[i].theta+yaw_rate*delta_t));
      particles[i].theta+=yaw_rate*delta_t;
    }
    // Add motion noise
    particles[i].x+=x_gaussian_meas(generator);
    particles[i].y+=y_gaussian_meas(generator);
    particles[i].theta+=theta_gaussian_meas(generator);  
  }  
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  // For each observed measurement (transformed landmark location to world CS), find the nearest neighbor in predicted measurement (landmark location in world CS) 
  // by using the dist function from help_functions.h
  for (unsigned i=0; i<observations.size(); ++i){
    double x_obs=observations[i].x;
    double y_obs=observations[i].y;
    double nearest_neighbor_dist=std::numeric_limits<double>::max();
    // Initialzed the nearest landmark id to an infeasible number 
    int nearest_LandMark_id=-1;
    for (unsigned j=0; j<predicted.size(); ++j){
      double current_dist = dist(x_obs, y_obs, predicted[j].x, predicted[j].y);
      if (current_dist<nearest_neighbor_dist){
        nearest_neighbor_dist=current_dist;
        nearest_LandMark_id=predicted[j].id;
      }
    }
    // Assign the landmark id to observation 
    observations[i].id=nearest_LandMark_id;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  
  // For each particle, update the its weight
  for (int i=0; i<num_particles; ++i){
    double p_x = particles[i].x;
    double p_y = particles[i].y;
    double p_theta = particles[i].theta;
    
    // Predicted measurement (landmark location in world CS) that falls in the sensor range of current vehicle location
    vector<LandmarkObs> predicted;
    for (unsigned j=0; j<map_landmarks.landmark_list.size(); ++j){
      int LandMark_id=map_landmarks.landmark_list[j].id_i;
      float LandMark_x=map_landmarks.landmark_list[j].x_f;
      float LandMark_y=map_landmarks.landmark_list[j].y_f;
      // If the landmark falls in the sensor range, add it the the predicted
      if (dist(p_x,p_y,LandMark_x,LandMark_y)<sensor_range){
        predicted.push_back(LandmarkObs{LandMark_id, LandMark_x, LandMark_y});
      }
    }
    
    // Conduct homogeneous transformation of the observed measurement to get their location in world CS 
    vector<LandmarkObs> trans_observation;
    for (unsigned j=0; j<observations.size(); ++j){ 
      double x_obs = observations[j].x;
      double y_obs = observations[j].y;
      // Homogeneous transformation
      double trans_x=cos(p_theta)*x_obs-sin(p_theta)*y_obs+p_x;
      double trans_y=sin(p_theta)*x_obs+cos(p_theta)*y_obs+p_y;
      trans_observation.push_back(LandmarkObs{observations[j].id,trans_x,trans_y});
    }
   
    // Conduct dataAssociation for each particle
    // For each observed measurement (transformed landmark location to world CS), find the nearest neighbor in predicted measurement (landmark location in world CS) 
    dataAssociation(predicted, trans_observation);
    
    // Reset each particle's weight as it changes from run to run (VERY IMPORTANT!!!) 
    particles[i].weight = 1.0;  
    
    // Calculate the weights of each particle using a mult-variate Gaussian distribution
    double sigma_x=std_landmark[0];
    double sigma_y=std_landmark[1];
    for (unsigned j=0; j<trans_observation.size(); ++j){
      double obs_j_x=trans_observation[j].x;
      double obs_j_y=trans_observation[j].y;
      double predicted_j_x,predicted_j_y;
      for (unsigned k=0; k<predicted.size(); ++k){
        if (predicted[k].id==trans_observation[j].id){
          predicted_j_x=predicted[k].x;
          predicted_j_y=predicted[k].y;
        }
      }
      double obs_weight=1/(2*M_PI*sigma_x*sigma_y)*exp(-(pow(obs_j_x-predicted_j_x,2)/2/pow(sigma_x,2)+pow(obs_j_y-predicted_j_y,2)/2/pow(sigma_y,2)));
      particles[i].weight*=obs_weight;  
    }  
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  
  // Resample particles with probability proportional to their weight using "Resample wheel"
  // Create a new instance called resampled_particles
  vector<Particle> resampled_particles;
  vector<double> all_weights;
  for (int i=0; i<num_particles; ++i){
    all_weights.push_back(particles[i].weight);
  }
  
  // Find the maximum weight
  double max_weight=*max_element(all_weights.begin(), all_weights.end());
  // Initialze index
  std::uniform_int_distribution<int> random_int_number(0, num_particles-1);
  int index=random_int_number(generator);
  // "Resample wheel"
  double beta=0.0;
  std::uniform_real_distribution<double> random_real_01(0.0, 1.0);
  for (int i=0; i<num_particles; ++i){
    beta+=random_real_01(generator)*2.0*max_weight;
    while (beta>all_weights[index]){
      beta-=all_weights[index];
      index=(index+1)%num_particles;
    }
    resampled_particles.push_back(particles[index]);
  }
  particles=resampled_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}