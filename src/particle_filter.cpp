/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, const double std[]) {
    // TODO: Initialize all particles to first position (based on estimates of
    //   x, y, theta and their uncertainties from GPS) and all weights to 1. 
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

        cout << "init(): Sample " << i + 1 << " " << sample_x << " " << sample_y << " " << sample_theta << endl;
    }

    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // TODO: Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/

    // TODO Formulas in Lesson 15: 8. Calculate Prediction Step Quiz Explanation

    // TODO load std_post[0..2] into std_dev into normal_distributions here and in init(), extract fn

    default_random_engine gen;
    for (int i = 0; i < num_particles; i++) {
        double new_x;
        double new_y;
        double new_theta;

        // TODO for really small yaw_rate
        if (yaw_rate == 0.0) {
            new_x = particles[i].x + velocity * delta_t * cos(particles[i].theta);
            new_y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
        } else {
            // TODO lookup complete formula from lesson
            new_x = particles[i].x + velocity / yaw_rate * sin(particles[i].theta + yaw_rate * delta_t) - sin();
            // TODO looks numerically unstable? dividing by very small yaw_rate if theta low
            new_y = particles[i].y + velocity / yaw_rate * cos(particles[i].theta) - cos(particles[i].theta) ...;
            new_theta = particles[i].theta + yaw_rate * delta_t;
        }

        normal_distribution<double> N_x(new_x, std_pos[0]);
        normal_distribution<double> N_y(new_y, std_pos[1]);
        normal_distribution<double> N_theta(new_theta, std_pos[2]);

        particles[i].x = N_x(gen);
        particles[i].y = N_y(gen);
        particles[i].theta =  N_theta(gen);
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    // TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
    //   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
        const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
    // TODO: Update the weights of each particle using a multi-variate Gaussian distribution. You can read
    //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
    // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
    //   according to the MAP'S coordinate system. You will need to transform between the two systems.
    //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
    //   The following is a good resource for the theory:
    //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
    //   and the following is a good resource for the actual equation to implement (look at equation 
    //   3.33
    //   http://planning.cs.uiuc.edu/node99.html


    // TODO for each particle and associated weight:
    //   TODO Use setAssociations() to set associations
    //   TODO Update weights[i] and particles[i].weight
    for (int p = 0; p < num_particles; p++) {

        // TODO missing code?

        vector<int> associations;
        vector<double> sense_x;
        vector<double> sense_y;

        vector<LandmarkObs> trans_observations;
        LandmarkObs obs;
        for (int i = 0; i < observations.size(); i++) {
            LandmarkObs trans_obs;
            obs = observations[]

            // vehicle-to-map coordinate space transformation
            // TODO Look into using real matrix + vector here? overkill? Put offset last instead
            trans_obs.y = particles[p].x + (obs.x * cos(particles[p].theta) - obs.y * sin(particles[p].theta));
            trans_obs.x = particles[p].y + (obs.x * sin(particles[p].theta) + obs.y * cos(particles[p].theta));
            trans_observations.push_back(trans_obs);
        }

        particles[p].weight = 1.0;

        for (int i = 0; i < trans_observations.size(); i++) {
            double closest_dist = sensor_range;
            int association = 0;

            for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {
                double landmark_x = map_landmarks.landmark_list[j].x_f;
                double landmark_y = map_landmarks.landmark_list[j].y_f;

                // TODO lookup distance calculation from lessons
                double calc_dist = sqrt(pow(trans_observations[i].x - landmark_x, 2.0) + pow(trans_observations[i].?));
                if (calc_dist < closest_dist) {
                    closest_dist = calc_dist;
                    association = j;
                }
            }

            if (association != 0) {
                double meas_x = trans_observations[i].x;
                double meas_y = trans_observations[i].y;
                double mu_x = map_landmarks.landmark_list[association].x_f;
                double mu_y = map_landmarks.landmark_list[association].y_f;
                // TODO look up multiplier calculation from lessons: (Transformations and Associations)
                const long double multiplier =
                        1 / (2_M_PI * std_landmark[0] * std_landmark[1]) * exp(-(pow(meas_x - )));
                if (multiplier > 0) {
                    particles[p].weight *= multiplier;
                }
                associations.push_back(association + 1);
                sense_x.push_back(trans_observations[i].x);
                sense_y.push_back(trans_observations[i].y);
            }

            particles[p] = SetAssociations(particles[p], associations, sense_x, sense_y);
            weights[p] = particles[p].weight;
        }
    }
}

void ParticleFilter::resample() {
    // Resamples particles with replacement with probability proportional to their weight.
    // Based on python resampling-algorithm in
    // Lesson 14: Particle filters - 20. Quiz: Resampling Wheel - answer

    // TODO if gen and distribution are heavy-weight(?) turn them into singletons
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

    // TODO are .clear() really required? assignment overwrites everything?
    particle.associations.clear();
    particle.sense_x.clear();
    particle.sense_y.clear();

    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;

    // TODO return particle. The argument one?
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

