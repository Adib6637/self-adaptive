#include "mlp.h"
#include <fstream>
#include <iostream>
#include <iomanip>

MLP::MLP(const std::vector<int>& layers, double lr) : lr(lr) {
    int n_layers = layers.size();
    
    //std::cout << "Initializing MLP with layers: ";
    for (int i = 0; i < n_layers; ++i) {
        std::cout << layers[i] << " ";
    }
    std::cout << std::endl;

    for (int i = 1; i < n_layers; ++i) {
        // Initialize weights with Xavier/Glorot initialization
        double scale = std::sqrt(2.0 / (layers[i-1] + layers[i]));
        weights.push_back(Eigen::MatrixXd::Random(layers[i], layers[i-1]) * scale);
        biases.push_back(Eigen::VectorXd::Zero(layers[i]));

        //std::cout << "Layer " << i << " weight matrix: " << layers[i] << "x" << layers[i-1] << std::endl;
    }
}

double MLP::forward(const Eigen::VectorXd& x) {
    return forward_vector(x)(0);
}

Eigen::VectorXd MLP::forward_vector(const Eigen::VectorXd& x) {
    activations.clear();
    pre_activations.clear();
    
    // Check input dimensions
    //std::cout << "Forward pass input dimension: " << x.size() << std::endl;
    
    Eigen::VectorXd a = x;
    activations.push_back(a);
    
    for (size_t i = 0; i < weights.size(); ++i) {
        //std::cout << "Layer " << i << " forward pass:" << std::endl;
        //std::cout << "Weight matrix: " << weights[i].rows() << "x" << weights[i].cols() << std::endl;
        //std::cout << "Input vector: " << a.size() << std::endl;
        //std::cout << "Bias vector: " << biases[i].size() << std::endl;
        
        if (weights[i].cols() != a.size()) {
            //std::cerr << "Dimension mismatch at layer " << i << std::endl;
            //std::cerr << "Weight matrix columns: " << weights[i].cols() << ", Input size: " << a.size() << std::endl;
            throw std::runtime_error("Matrix dimension mismatch in forward pass");
        }
        
        a = (weights[i] * a + biases[i]).array().tanh();
        pre_activations.push_back(a);
        activations.push_back(a);
    }
    
    //std::cout << "Final output dimension: " << a.size() << std::endl;
    return a;
}

void MLP::backward(double pred, double target) {
    // For scalar output, create a gradient vector of size equal to the last layer
    Eigen::VectorXd grad = Eigen::VectorXd::Zero(weights.back().rows());
    grad(0) = pred - target;
    backward_with_gradient(grad);
}

void MLP::backward_with_gradient(const Eigen::VectorXd& output_grad) {
    std::vector<Eigen::VectorXd> deltas(weights.size());
    
    // Ensure output gradient matches the output layer size
    if (output_grad.size() != weights.back().rows()) {
        std::cerr << "Output gradient size (" << output_grad.size() 
                  << ") doesn't match output layer size (" << weights.back().rows() << ")" << std::endl;
        throw std::runtime_error("Gradient size mismatch");
    }
    
    deltas.back() = output_grad;
    
    // Backpropagate through hidden layers
    for (int l = (int)weights.size() - 2; l >= 0; --l) {
        // Debug dimensions
        //std::cout << "Layer " << l << " dimensions:" << std::endl;
        //std::cout << "weights[" << l+1 << "] shape: " << weights[l+1].rows() << "x" << weights[l+1].cols() << std::endl;
        //std::cout << "deltas[" << l+1 << "] shape: " << deltas[l+1].size() << std::endl;
        //std::cout << "activations[" << l+1 << "] shape: " << activations[l+1].size() << std::endl;

        Eigen::VectorXd da;
        try {
            da = weights[l+1].transpose() * deltas[l+1];
        } catch (const std::exception& e) {
            std::cerr << "Matrix multiplication error at layer " << l << ": " << e.what() << std::endl;
            throw;
        }
        
        Eigen::VectorXd dz = da.array() * (1 - activations[l+1].array().square()); // tanh'
        deltas[l] = dz;
    }
    
    // Update weights and biases
    for (size_t l = 0; l < weights.size(); ++l) {
        Eigen::MatrixXd dW = deltas[l] * activations[l].transpose();
        weights[l] -= lr * dW;
        biases[l] -= lr * deltas[l];
    }
}

void MLP::save_weights(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) return;
    file << weights.size() << std::endl;
    for (size_t i = 0; i < weights.size(); ++i) {
        file << weights[i].rows() << " " << weights[i].cols() << std::endl;
        for (int r = 0; r < weights[i].rows(); ++r)
            for (int c = 0; c < weights[i].cols(); ++c)
                file << weights[i](r, c) << " ";
        file << std::endl;
        file << biases[i].size() << std::endl;
        for (int r = 0; r < biases[i].size(); ++r)
            file << biases[i](r) << " ";
        file << std::endl;
    }
}

void MLP::load_weights(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) return;
    size_t n_layers;
    file >> n_layers;
    weights.resize(n_layers);
    biases.resize(n_layers);
    for (size_t i = 0; i < n_layers; ++i) {
        int rows, cols;
        file >> rows >> cols;
        weights[i] = Eigen::MatrixXd(rows, cols);
        for (int r = 0; r < rows; ++r)
            for (int c = 0; c < cols; ++c)
                file >> weights[i](r, c);
        int bsize;
        file >> bsize;
        biases[i] = Eigen::VectorXd(bsize);
        for (int r = 0; r < bsize; ++r)
            file >> biases[i](r);
    }
}

// for full nn. for test and learn only
// Global MLP instances: [input, hidden1, hidden2, ..., output] 
//MLP actuator_nn({8, 32, 64, 64, 128, 128, 256, 128, 16, 1}, 0.001); // 2 hidden layers: 32, 16 units
//MLP sensor_nn({2, 8, 4, 1}, 0.001);    // 2 hidden layers: 8, 4 units