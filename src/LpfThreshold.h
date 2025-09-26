#pragma once
#include <Eigen/Dense>
#include <mc_control/GlobalPluginMacros.h>

class LpfThreshold {
public:
    LpfThreshold(void);

    void setValues(double offset, double filtering, int jointNumber);
    void setValues(Eigen::VectorXd offset, double filtering, int jointNumber);
    void setOffset(double offset);
    void setOffset(Eigen::VectorXd offset);
    void setFiltering(double filtering);
    double adaptiveThreshold(double newSignal, bool high);
    Eigen::VectorXd adaptiveThreshold(const Eigen::VectorXd& newSignal, bool high);
    
private:
    int jointNumber_;
    double filtered_signal_b_;
    Eigen::VectorXd filtered_signal_;
    // double offset_; // +/- threshold offset compared to the given signal
    double filtering_; // filtering factor must be between 0 and 1
    Eigen::VectorXd offset_;
};