// Vendored from esphome/components/micro_wake_word/streaming_model.h
// Source commit: esphome/esphome@dev (2026-05-11). Apache 2.0.
// Stripped:
//   - USE_ESP32 guard (always defined for us)
//   - esphome:: namespace
//   - ESPPreferenceObject persistence (we own the enabled flag elsewhere)
//   - internal_only / HomeAssistant exposure plumbing (irrelevant here)
//
// Two-model design:
//   - StreamingModel: shared TFLM interpreter wrapper. Owns op resolver,
//     tensor + variable arenas, sliding-window probability buffer.
//   - WakeWordModel: positive-detection model; `determine_detected` returns
//     true when the windowed average crosses the cutoff.
//   - VADModel: gating model; `determine_detected` returns true on max-prob.
//
// The XVF3800 firmware only uses WakeWordModel in v1 (VAD is handled by the
// energy-threshold VAD in components/vad). VADModel is kept here for parity
// with the upstream so a future v2 build can flip it on without re-vendoring.

#pragma once

#include "preprocessor_settings.h"

#include <memory>
#include <string>
#include <vector>

#include <tensorflow/lite/core/c/common.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/micro/micro_mutable_op_resolver.h>

static const uint8_t MIN_SLICES_BEFORE_DETECTION = 100;
static const uint32_t STREAMING_MODEL_VARIABLE_ARENA_SIZE = 1024;

struct DetectionEvent {
  std::string *wake_word;
  bool detected;
  bool partially_detection;
  uint8_t max_probability;
  uint8_t average_probability;
  bool blocked_by_vad = false;
};

class StreamingModel {
 public:
  virtual ~StreamingModel() = default;
  virtual void log_model_config() = 0;
  virtual DetectionEvent determine_detected() = 0;

  // Feed one slice of audio features. Returns false on inference error.
  bool perform_streaming_inference(const int8_t features[PREPROCESSOR_FEATURE_SIZE]);

  void reset_probabilities();
  void unload_model();

  virtual void enable() { this->enabled_ = true; }
  virtual void disable() { this->enabled_ = false; }
  bool is_enabled() const { return this->enabled_; }

  bool get_unprocessed_probability_status() const { return this->unprocessed_probability_status_; }

  uint8_t get_default_probability_cutoff() const { return this->default_probability_cutoff_; }
  uint8_t get_probability_cutoff() const { return this->probability_cutoff_; }
  void set_probability_cutoff(uint8_t probability_cutoff) { this->probability_cutoff_ = probability_cutoff; }

 protected:
  bool load_model_();
  size_t probe_arena_size_();
  bool register_streaming_ops_(tflite::MicroMutableOpResolver<20> &op_resolver);

  tflite::MicroMutableOpResolver<20> streaming_op_resolver_;

  bool loaded_{false};
  bool enabled_{true};
  bool tensor_arena_size_probed_{false};
  bool unprocessed_probability_status_{false};
  uint8_t current_stride_step_{0};
  int16_t ignore_windows_{-MIN_SLICES_BEFORE_DETECTION};

  uint8_t default_probability_cutoff_{0};
  uint8_t probability_cutoff_{0};
  size_t sliding_window_size_{0};

  size_t last_n_index_{0};
  size_t tensor_arena_size_{0};
  std::vector<uint8_t> recent_streaming_probabilities_;

  const uint8_t *model_start_{nullptr};
  uint8_t *tensor_arena_{nullptr};
  uint8_t *var_arena_{nullptr};
  std::unique_ptr<tflite::MicroInterpreter> interpreter_;
  tflite::MicroResourceVariables *mrv_{nullptr};
  tflite::MicroAllocator *ma_{nullptr};
};

class WakeWordModel final : public StreamingModel {
 public:
  WakeWordModel(const std::string &id,
                const uint8_t *model_start,
                uint8_t default_probability_cutoff,
                size_t sliding_window_average_size,
                const std::string &wake_word,
                size_t tensor_arena_size,
                bool default_enabled);

  void log_model_config() override;
  DetectionEvent determine_detected() override;

  const std::string &get_id() const { return this->id_; }
  const std::string &get_wake_word() const { return this->wake_word_; }

 protected:
  std::string id_;
  std::string wake_word_;
};

class VADModel final : public StreamingModel {
 public:
  VADModel(const uint8_t *model_start, uint8_t default_probability_cutoff,
           size_t sliding_window_size, size_t tensor_arena_size);

  void log_model_config() override;
  DetectionEvent determine_detected() override;
};
