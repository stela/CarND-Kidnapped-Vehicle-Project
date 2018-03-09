/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedMacroInspection"
#define _USE_MATH_DEFINES
#include <cmath>
#pragma clang diagnostic pop

#include <random>
#include <iostream>
#include <sstream>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, const double std[]) {
    // Initialize all particles to first position (based on estimates of
    // x, y, theta and their uncertainties from GPS) and all weights to 1.
    // Add random Gaussian noise to each particle.
    // NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    // Based on Lesson 15: 5. Program Gaussian Sampling: Code Solution
    // initial measurement uncertainties are in std (double array [x, y, theta])
    // x, y, theta are initial sensor measurements, from GPS
    default_random_engine gen;
    double std_x = std[0];
    double std_y = std[1];
    double std_theta = std[2];

    // normal (Gaussian) distributions for x, y and theta
    normal_distribution<double> dist_x(x, std_x);
    normal_distribution<double> dist_y(y, std_y);
    normal_distribution<double> dist_theta(theta, std_theta);

    // create the initial gaussian-distributed particles
    for (int i = 0; i < num_particles; ++i) {
        const double sample_x = dist_x(gen);
        const double sample_y = dist_y(gen);
        const double sample_theta = dist_theta(gen);

        const Particle particle(i, sample_x, sample_y, sample_theta);
        particles.push_back(particle);
        weights.push_back(1.0);

        // cout << "init(): Sample " << i + 1 << " " << sample_x << " " << sample_y << " " << sample_theta << endl;
    }

    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, const double std_pos[], double velocity, double yaw_rate) {
    // Adds measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/

    default_random_engine gen;
    for (auto &p : particles) {
        double new_x;
        double new_y;
        double new_theta;
      
        if (yaw_rate == 0.0) {
            // Formulas in Lesson 13 Motion Models - 3. Yaw Rate and Velocity
            new_x = p.x + velocity * delta_t * cos(p.theta);
            new_y = p.y + velocity * delta_t * sin(p.theta);
            // if zero yaw, theta stays the same
            new_theta = p.theta;
        } else {
            // Formulas in Lesson 15: 8. Calculate Prediction Step Quiz Explanation
            new_x = p.x + velocity / yaw_rate * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
            // looks numerically unstable when theta is small, if problematic approximate zero-comparison above
            new_y = p.y + velocity / yaw_rate * (cos(p.theta) - cos(p.theta + yaw_rate * delta_t));
            new_theta = p.theta + yaw_rate * delta_t;
        }

        double std_x = std_pos[0];
        double std_y = std_pos[1];
        double std_theta = std_pos[2];
        normal_distribution<double> N_x(new_x, std_x);
        normal_distribution<double> N_y(new_y, std_y);
        normal_distribution<double> N_theta(new_theta, std_theta);

        p.x = N_x(gen);
        p.y = N_y(gen);
        p.theta =  N_theta(gen);
    }
}

#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-parameter"
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
#pragma clang diagnostic pop
    // Find the predicted measurement that is closest to each observed measurement and assign the
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
    //   implement this method and use it as a helper during the updateWeights phase.

}
#pragma clang diagnostic pop

void ParticleFilter::updateWeights(double sensor_range, const double std_landmark[],
        const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
    // Update the weights of each particle using a multi-variate Gaussian distribution. You can read
    //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
    // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
    //   according to the MAP'S coordinate system. You will need to transform between the two systems.
    //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
    //   The following is a good resource for the theory:
    //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
    //   and the following is a good resource for the actual equation to implement (look at equation 
    //   3.33
    //   http://planning.cs.uiuc.edu/node99.html


    // for each particle and associated weight:
    //   Use setAssociations() to set associations
    //   Update weights[i] and particles[i].weight
    for (int p = 0; p < num_particles; p++) {
        Particle &particle = particles[p];
        
        vector<LandmarkObs> trans_observations;
        for (int i = 0; i < observations.size(); i++) {
            const LandmarkObs &obs = observations[i];

            // vehicle-to-map coordinate space transformation
            const double theta = particle.theta;
            double obs_x = particle.x + (obs.x * cos(theta) - obs.y * sin(theta));
            double obs_y = particle.y + (obs.x * sin(theta) + obs.y * cos(theta));
            LandmarkObs trans_obs(i, obs_x, obs_y);
            trans_observations.push_back(trans_obs);
        }

        vector<int> associations;
        vector<double> sense_x;
        vector<double> sense_y;

        // Initialize weight to 1.0, then scale it per-observation further down
        particle.weight = 1.0;

        // if there are no observations, keep previous weights
        for (const auto &trans_observation : trans_observations) {
            double closest_dist = sensor_range;
            int association = -1;

            for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {
                double landmark_x = map_landmarks.landmark_list[j].x_f;
                double landmark_y = map_landmarks.landmark_list[j].y_f;

                // using euclidean distance for simplicity, could be adjusted for sensor-noise-distribution
                double calc_dist =
                        sqrt(pow(trans_observation.x - landmark_x, 2.0)
                             + pow(trans_observation.y - landmark_y, 2.0));
                if (calc_dist < closest_dist) {
                    closest_dist = calc_dist;
                    association = j;
                }
            }

            if (association != -1) {
                double meas_x = trans_observation.x;
                double meas_y = trans_observation.y;
                double mu_x = map_landmarks.landmark_list[association].x_f;
                double mu_y = map_landmarks.landmark_list[association].y_f;
                // From Lesson 15: Implementation of a Particle Filter - 18. Quiz: Particle Weights
                const double std_x = std_landmark[0];
                const double std_y = std_landmark[1];
                const long double multiplier =
                        1 / (2.0 * M_PI * std_x * std_y)
                            * exp(-(pow(meas_x - mu_x, 2.0) / (2 * pow(std_x, 2.0))
                                    + pow(meas_y - mu_y, 2.0) / (2 * pow(std_y, 2.0))));
                if (multiplier >= 0.0) {
                    particle.weight *= multiplier;
                }
                associations.push_back(association + 1);
                sense_x.push_back(trans_observation.x);
                sense_y.push_back(trans_observation.y);
            } else {
                // remove any non-associated particles
                particle.weight = 0.0;
            }
        }

        particle = SetAssociations(particle, associations, sense_x, sense_y);
        weights[p] = particle.weight;
    }
}

void ParticleFilter::resample() {
    // Resamples particles with replacement with probability proportional to their weight.
    // Based on python resampling-algorithm in
    // Lesson 14: Particle filters - 20. Quiz: Resampling Wheel - answer

    // if gen and distribution are too heavy-weight, turn them into singletons
    default_random_engine gen;
    // See http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    discrete_distribution<int> distribution(weights.begin(), weights.end());

    // double-buffer particles instead in case allocating and copying proves too slow
    vector<Particle> resample_particles;
    resample_particles.reserve(particles.size());
    for (const auto &ignore : particles) {
        // distribution.operator() is overridden to return a random (weighted) index
        resample_particles.push_back(particles[distribution(gen)]);
    }

    particles = resample_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;

    return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
    vector<int> v = best.associations;
    stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
    vector<double> v = best.sense_x;
    stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
    vector<double> v = best.sense_y;
    stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
