#ifndef LEARNED_RECOVERY_H
#define LEARNED_RECOVERY_H

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <istream>
#include <string>
#include <vector>

// Portable inference for the exported two-hidden-layer PyTorch MLP. Parameters
// are stored as output-major dense matrices, exactly as torch.nn.Linear.weight.
class LearnedRecoveryNetwork {
public:
    static constexpr size_t input_size = 103;

    bool load(const std::string& path) {
        std::ifstream stream(path);
        return load(stream);
    }
    bool load(std::istream& stream) {
        ready_ = false;
        std::string magic;
        size_t input = 0, output = 0;
        if (!(stream >> magic >> input >> first_ >> second_ >> output) ||
            !((magic == "RECOVERY_MLP_V1" && input == input_size && output == 2) ||
              (magic == "RECOVERY_MLP_V2" && input >= 28 && input <= 4096 &&
               (output == 1 || output == 2))) ||
            first_ == 0 || first_ > 512 || second_ == 0 || second_ > 512) return false;
        if (!read(stream, w1_, input * first_) || !read(stream, b1_, first_) ||
            !read(stream, w2_, first_ * second_) || !read(stream, b2_, second_) ||
            !read(stream, w3_, second_ * output) || !read(stream, b3_, output)) return false;
        std::string extra;
        if (stream >> extra) return false;
        input_ = input;
        output_ = output;
        ready_ = true;
        return true;
    }
    bool ready() const { return ready_; }
    size_t feature_size() const { return ready_ ? input_ : input_size; }
    size_t output_size() const { return ready_ ? output_ : 0; }
    bool predict(const std::vector<float>& feature, std::array<float, 2>& output) const {
        if (!ready_ || feature.size() != input_) return false;
        for (float v : feature) if (!std::isfinite(v)) return false;
        std::vector<float> first(first_), second(second_);
        dense(feature, w1_, b1_, first, true);
        dense(first, w2_, b2_, second, true);
        output = {{0.0f, 0.0f}};
        for (size_t i = 0; i < output_; ++i) {
            float sum = b3_[i];
            for (size_t j = 0; j < second_; ++j) sum += w3_[i * second_ + j] * second[j];
            output[i] = sum;
            if (!std::isfinite(sum)) return false;
        }
        return true;
    }
    static float probability(float logit) {
        if (logit >= 0) return 1.0f / (1.0f + std::exp(-logit));
        const float e = std::exp(logit);
        return e / (1.0f + e);
    }
    static float predicted_steps(float log_steps) {
        return std::expm1(std::max(0.0f, std::min(12.0f, log_steps)));
    }
private:
    static bool read(std::istream& stream, std::vector<float>& values, size_t count) {
        values.resize(count);
        for (float& value : values) if (!(stream >> value) || !std::isfinite(value)) return false;
        return true;
    }
    static void dense(const std::vector<float>& input, const std::vector<float>& weights,
                      const std::vector<float>& bias, std::vector<float>& output, bool relu) {
        for (size_t i = 0; i < output.size(); ++i) {
            float sum = bias[i];
            for (size_t j = 0; j < input.size(); ++j) sum += weights[i * input.size() + j] * input[j];
            output[i] = relu ? std::max(0.0f, sum) : sum;
        }
    }
    bool ready_ = false;
    size_t input_ = input_size, output_ = 2, first_ = 0, second_ = 0;
    std::vector<float> w1_, b1_, w2_, b2_, w3_, b3_;
};
#endif
