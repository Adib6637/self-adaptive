//sc_in<double> observed_data[10];
//sc_out<double> constraint_value[10];

#include "constraint_tuner.h"
#include "parameter.h"
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <fstream>

// K-Means clustering
std::vector<double> kmeans_1d(const std::vector<double>& data, int k, int max_iters = 100);
std::vector<std::pair<double, double>> kmeans_2d(const std::vector<std::pair<double, double>>& data, int k, int max_iters = 100);
bool is_anomaly_2d(const std::pair<double, double>& point,
                   const std::vector<std::pair<double, double>>& centroids,
                   double threshold);

void Constraint_Tuner::tune() {
    if (!CONSTRAINT_TUNER_ON){
    return;
    }
    if (observed_data[19].read() == 0){
        return;
    }
    if (counter == observed_data[19].read()) {
        return; // No new data to process
    }else {
        counter = observed_data[19].read(); // Update counter to the latest data
    }

    // Collect observed data
    speed_data.push_back(observed_data[5].read());
    power_data.push_back(observed_data[6].read());

    if (counter < 20) {
        counter +=1 ;
        return; // Not enough data to process
    }
    counter = 0; // Reset counter after processing



    int k = 2; // Number of clusters

    // --- 1D Clustering on speed ---
    if (speed_data.size() >= k) {
        std::vector<double> speed_centroids = kmeans_1d(speed_data, k);

        // Store centroids to CSV
        std::ofstream speed_centroids_file("speed_centroids.csv");
        for (const auto& c : speed_centroids) {
            speed_centroids_file << c << "\n";
        }
        speed_centroids_file.close();
    }

    // --- 2D Clustering on (speed, power actuator) ---
    if (speed_data.size() == power_data.size() && speed_data.size() >= k) {
        std::vector<std::pair<double, double>> observed_data;
        for (size_t i = 0; i < speed_data.size(); ++i) {
            observed_data.emplace_back(speed_data[i], power_data[i]);
        }
        std::vector<std::pair<double, double>> centroids = kmeans_2d(observed_data, k);

        // Store centroids to CSV
        std::ofstream centroids_file("../dataset/speed_power_centroids.csv");
        for (const auto& c : centroids) {
            centroids_file << c.first << "," << c.second << "\n";
        }
        centroids_file.close();

        // Optionally, store all observed data for plotting
        std::ofstream observed_file("../dataset/speed_power_observed.csv");
        for (const auto& d : observed_data) {
            observed_file << d.first << "," << d.second << "\n";
        }
        observed_file.close();

        // --- Anomaly Detection ---
        double anomaly_threshold = 0.0863; // Tune this value for your data
        std::pair<double, double> latest_point = observed_data.back();
        if (is_anomaly_2d(latest_point, centroids, anomaly_threshold)) {
            std::ofstream anomaly_file("../dataset/anomaly_detected.csv", std::ios::app);
            anomaly_file << latest_point.first << "," << latest_point.second << "\n";
            anomaly_file.close();
        }
    }

}

//K-Means clustering for 1D data
std::vector<double> kmeans_1d(const std::vector<double>& data, int k, int max_iters) {
    std::vector<double> centroids(data.begin(), data.begin() + k);
    std::vector<int> labels(data.size(), 0);

    for (int iter = 0; iter < max_iters; ++iter) {
        // Assign clusters
        for (size_t i = 0; i < data.size(); ++i) {
            double min_dist = std::numeric_limits<double>::max();
            int best = 0;
            for (int c = 0; c < k; ++c) {
                double dist = std::abs(data[i] - centroids[c]);
                if (dist < min_dist) {
                    min_dist = dist;
                    best = c;
                }
            }
            labels[i] = best;
        }
        // Update centroids
        std::vector<double> new_centroids(k, 0.0);
        std::vector<int> counts(k, 0);
        for (size_t i = 0; i < data.size(); ++i) {
            new_centroids[labels[i]] += data[i];
            counts[labels[i]]++;
        }
        for (int c = 0; c < k; ++c) {
            if (counts[c] > 0)
                new_centroids[c] /= counts[c];
            else
                new_centroids[c] = centroids[c]; // No change if no points
        }
        if (centroids == new_centroids) break;
        centroids = new_centroids;
    }
    return centroids;
}

// 2D K-Means clustering
std::vector<std::pair<double, double>> kmeans_2d(const std::vector<std::pair<double, double>>& data, int k, int max_iters) {
    std::vector<std::pair<double, double>> centroids(data.begin(), data.begin() + k);
    std::vector<int> labels(data.size(), 0);

    for (int iter = 0; iter < max_iters; ++iter) {
        // Assign clusters
        for (size_t i = 0; i < data.size(); ++i) {
            double min_dist = std::numeric_limits<double>::max();
            int best = 0;
            for (int c = 0; c < k; ++c) {
                double dx = data[i].first - centroids[c].first;
                double dy = data[i].second - centroids[c].second;
                double dist = std::sqrt(dx * dx + dy * dy);
                if (dist < min_dist) {
                    min_dist = dist;
                    best = c;
                }
            }
            labels[i] = best;
        }
        // Update centroids
        std::vector<std::pair<double, double>> new_centroids(k, {0.0, 0.0});
        std::vector<int> counts(k, 0);
        for (size_t i = 0; i < data.size(); ++i) {
            new_centroids[labels[i]].first += data[i].first;
            new_centroids[labels[i]].second += data[i].second;
            counts[labels[i]]++;
        }
        for (int c = 0; c < k; ++c) {
            if (counts[c] > 0) {
                new_centroids[c].first /= counts[c];
                new_centroids[c].second /= counts[c];
            } else {
                new_centroids[c] = centroids[c];
            }
        }
        if (centroids == new_centroids) break;
        centroids = new_centroids;
    }
    return centroids;
}

// Compute distance from a point to the nearest centroid
bool is_anomaly_2d(const std::pair<double, double>& point,
                   const std::vector<std::pair<double, double>>& centroids,
                   double threshold)
{
    double min_dist = std::numeric_limits<double>::max();
    for (const auto& c : centroids) {
        double dx = point.first - c.first;
        double dy = point.second - c.second;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist < min_dist) min_dist = dist;
    }
    return min_dist > threshold;
}