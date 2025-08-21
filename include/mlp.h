#ifndef MLP_H
#define MLP_H

#include <Eigen/Dense>
#include <vector>

// General Multi-Layer Perceptron (MLP) for regression
class MLP {
public:
    MLP(const std::vector<int>& layers, double lr = 0.01);
    
    // Forward passes
    double forward(const Eigen::VectorXd& x); // Returns single output
    Eigen::VectorXd forward_vector(const Eigen::VectorXd& x); // Returns vector output
    
    // Backward passes
    void backward(double pred, double target); // For single output
    void backward_with_gradient(const Eigen::VectorXd& grad); // For vector output with custom gradient
    
    // Save weights and biases to file
    void save_weights(const std::string& filename) const;
    // Load weights and biases from file
    void load_weights(const std::string& filename);
    
    // Set learning rate
    void set_learning_rate(double new_lr) {
        lr = new_lr;
    }
private:
    double lr;
    std::vector<Eigen::MatrixXd> weights;
    std::vector<Eigen::VectorXd> biases;
    std::vector<Eigen::VectorXd> activations;
    std::vector<Eigen::VectorXd> pre_activations;
};


#endif