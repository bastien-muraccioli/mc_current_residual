#include "LpfThreshold.h"

LpfThreshold::LpfThreshold(void) {}

void LpfThreshold::setValues(double offset, double filtering, int jointNumber) {
    offset_ = Eigen::VectorXd::Constant(jointNumber_, offset);
    filtering_ = filtering;
    jointNumber_ = jointNumber;
    filtered_signal_b_ = 0.0;
    filtered_signal_ = Eigen::VectorXd::Zero(jointNumber_);
}

double LpfThreshold::adaptiveThreshold(double newSignal, bool high) {
    filtered_signal_b_ = filtering_ * newSignal + (1 - filtering_) * filtered_signal_b_;
    return high ? filtered_signal_b_ + offset_[0]: filtered_signal_b_ - offset_[0];
}

Eigen::VectorXd LpfThreshold::adaptiveThreshold(const Eigen::VectorXd& newSignal, bool high) {
    filtered_signal_ = filtering_ * newSignal + (1 - filtering_) * filtered_signal_;
    if(high)
    {
        return filtered_signal_ + offset_;
    }
    else
    {
        return filtered_signal_ - offset_;
    }
}

void LpfThreshold::setOffset(double offset) {
    offset_ = Eigen::VectorXd::Constant(jointNumber_, offset);
}

void LpfThreshold::setFiltering(double filtering) {
    filtering_ = filtering;
}

void LpfThreshold::setValues(Eigen::VectorXd offset, double filtering, int jointNumber) {
    // Check if the size of the offset vector is the same as the joint number
    if(offset.size() != jointNumber)
    {
        mc_rtc::log::error("[LpfThreshold] The size of the offset vector {} must be the same as the joint number ({})", offset, jointNumber);
        return;
    }
    offset_ = offset;
    filtering_ = filtering;
    jointNumber_ = jointNumber;
    filtered_signal_b_ = 0.0;
    filtered_signal_ = Eigen::VectorXd::Zero(jointNumber_);
}

void LpfThreshold::setOffset(Eigen::VectorXd offset) {
    if(offset.size() != jointNumber_)
    {
        mc_rtc::log::error("[LpfThreshold] The size of the offset vector {} must be the same as the joint number ({})", offset, jointNumber_);
        return;
    }
    offset_ = offset;
}