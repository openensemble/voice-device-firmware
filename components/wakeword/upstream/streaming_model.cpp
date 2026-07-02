// Vendored from esphome/components/micro_wake_word/streaming_model.cpp
// Source commit: esphome/esphome@dev (2026-05-11). Apache 2.0.
// Stripped:
//   - USE_ESP32 guard, esphome:: namespace
//   - esphome/core/{helpers,log}.h (replaced with esp_log + heap_caps_malloc)
//   - ESPPreferenceObject pref_ in WakeWordModel constructor (the firmware
//     owns enable state, not the upstream)

#include "streaming_model.h"

#include <algorithm>
#include <cstring>

#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *const TAG = "mww";

// ---- RAM allocator helper -------------------------------------------------
// Upstream uses esphome::RAMAllocator which prefers SPIRAM if available and
// falls back to internal RAM. The same behaviour is achieved with
// heap_caps_malloc + MALLOC_CAP_SPIRAM (which silently falls back to internal
// when no PSRAM is present).

static uint8_t *mww_alloc(size_t bytes) {
  uint8_t *p = (uint8_t *) heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!p) p = (uint8_t *) heap_caps_malloc(bytes, MALLOC_CAP_8BIT);
  return p;
}

static void mww_free(uint8_t *p) {
  if (p) heap_caps_free(p);
}

// ---- StreamingModel -------------------------------------------------------

void WakeWordModel::log_model_config() {
  ESP_LOGI(TAG, "wake word: %s  cutoff=%.2f  window=%u",
           this->wake_word_.c_str(),
           this->probability_cutoff_ / 255.0f,
           (unsigned) this->sliding_window_size_);
}

void VADModel::log_model_config() {
  ESP_LOGI(TAG, "vad model  cutoff=%.2f  window=%u",
           this->probability_cutoff_ / 255.0f,
           (unsigned) this->sliding_window_size_);
}

bool StreamingModel::load_model_() {
  if (this->var_arena_ == nullptr) {
    this->var_arena_ = mww_alloc(STREAMING_MODEL_VARIABLE_ARENA_SIZE);
    if (this->var_arena_ == nullptr) {
      ESP_LOGE(TAG, "var_arena alloc failed");
      return false;
    }
    this->ma_ = tflite::MicroAllocator::Create(this->var_arena_, STREAMING_MODEL_VARIABLE_ARENA_SIZE);
    this->mrv_ = tflite::MicroResourceVariables::Create(this->ma_, 20);
  }

  const tflite::Model *model = tflite::GetModel(this->model_start_);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    ESP_LOGE(TAG, "tflite schema mismatch (got %u, expected %u)",
             (unsigned) model->version(), (unsigned) TFLITE_SCHEMA_VERSION);
    return false;
  }

  if (!this->tensor_arena_size_probed_) {
    size_t probed_size = this->probe_arena_size_();
    if (probed_size > 0) {
      ESP_LOGD(TAG, "probed arena: %zu bytes", probed_size);
      this->tensor_arena_size_ = probed_size;
    } else {
      ESP_LOGW(TAG, "arena probe failed, using manifest size %zu", this->tensor_arena_size_);
    }
    this->tensor_arena_size_probed_ = true;
  }

  if (this->tensor_arena_ == nullptr) {
    this->tensor_arena_ = mww_alloc(this->tensor_arena_size_);
    if (this->tensor_arena_ == nullptr) {
      ESP_LOGE(TAG, "tensor_arena alloc failed (%zu bytes)", this->tensor_arena_size_);
      return false;
    }
  }

  if (this->interpreter_ == nullptr) {
    this->interpreter_ = std::make_unique<tflite::MicroInterpreter>(
        tflite::GetModel(this->model_start_), this->streaming_op_resolver_,
        this->tensor_arena_, this->tensor_arena_size_, this->mrv_);
    if (this->interpreter_->AllocateTensors() != kTfLiteOk) {
      ESP_LOGE(TAG, "AllocateTensors failed");
      return false;
    }

    TfLiteTensor *input = this->interpreter_->input(0);
    if ((input->dims->size != 3) || (input->dims->data[0] != 1) ||
        (input->dims->data[2] != PREPROCESSOR_FEATURE_SIZE)) {
      ESP_LOGE(TAG, "input tensor shape unexpected (size=%d, dims=[%d,%d,%d])",
               input->dims->size, input->dims->data[0], input->dims->data[1], input->dims->data[2]);
      return false;
    }
    if (input->type != kTfLiteInt8) {
      ESP_LOGE(TAG, "input tensor is not int8");
      return false;
    }
    TfLiteTensor *output = this->interpreter_->output(0);
    if ((output->dims->size != 2) || (output->dims->data[0] != 1) || (output->dims->data[1] != 1)) {
      ESP_LOGE(TAG, "output tensor dim not 1x1");
      return false;
    }
    if (output->type != kTfLiteUInt8) {
      ESP_LOGE(TAG, "output tensor not uint8");
      return false;
    }
  }

  this->loaded_ = true;
  this->reset_probabilities();
  return true;
}

size_t StreamingModel::probe_arena_size_() {
  size_t attempt_sizes[] = {
      (this->tensor_arena_size_ + 15) & ~static_cast<size_t>(15),
      (this->tensor_arena_size_ * 3 / 2 + 15) & ~static_cast<size_t>(15),
      (this->tensor_arena_size_ * 2 + 15) & ~static_cast<size_t>(15),
  };

  for (size_t attempt_size : attempt_sizes) {
    uint8_t *probe_arena = mww_alloc(attempt_size);
    if (probe_arena == nullptr) {
      continue;
    }

    auto probe_interpreter = std::make_unique<tflite::MicroInterpreter>(
        tflite::GetModel(this->model_start_), this->streaming_op_resolver_,
        probe_arena, attempt_size, this->mrv_);

    if (probe_interpreter->AllocateTensors() != kTfLiteOk) {
      probe_interpreter.reset();
      mww_free(probe_arena);
      this->ma_ = tflite::MicroAllocator::Create(this->var_arena_, STREAMING_MODEL_VARIABLE_ARENA_SIZE);
      this->mrv_ = tflite::MicroResourceVariables::Create(this->ma_, 20);
      continue;
    }

    size_t lower = (probe_interpreter->arena_used_bytes() + 16 + 15) & ~static_cast<size_t>(15);
    probe_interpreter.reset();
    this->ma_ = tflite::MicroAllocator::Create(this->var_arena_, STREAMING_MODEL_VARIABLE_ARENA_SIZE);
    this->mrv_ = tflite::MicroResourceVariables::Create(this->ma_, 20);

    size_t upper = attempt_size;

    while (lower < upper) {
      auto test_interpreter = std::make_unique<tflite::MicroInterpreter>(
          tflite::GetModel(this->model_start_), this->streaming_op_resolver_,
          probe_arena, lower, this->mrv_);

      bool ok = test_interpreter->AllocateTensors() == kTfLiteOk;

      test_interpreter.reset();
      this->ma_ = tflite::MicroAllocator::Create(this->var_arena_, STREAMING_MODEL_VARIABLE_ARENA_SIZE);
      this->mrv_ = tflite::MicroResourceVariables::Create(this->ma_, 20);

      if (ok) {
        upper = lower + 16;
        break;
      }

      lower = ((lower + upper) / 2 + 15) & ~static_cast<size_t>(15);
    }

    mww_free(probe_arena);
    return upper;
  }

  return 0;
}

void StreamingModel::unload_model() {
  this->interpreter_.reset();

  if (this->tensor_arena_ != nullptr) {
    mww_free(this->tensor_arena_);
    this->tensor_arena_ = nullptr;
  }
  if (this->var_arena_ != nullptr) {
    mww_free(this->var_arena_);
    this->var_arena_ = nullptr;
  }
  this->loaded_ = false;
}

bool StreamingModel::perform_streaming_inference(const int8_t features[PREPROCESSOR_FEATURE_SIZE]) {
  if (this->enabled_ && !this->loaded_) {
    if (!this->load_model_()) return false;
  }

  if (!this->enabled_ && this->loaded_) {
    this->unload_model();
    return true;
  }

  if (this->loaded_) {
    TfLiteTensor *input = this->interpreter_->input(0);

    uint8_t stride = this->interpreter_->input(0)->dims->data[1];
    this->current_stride_step_ = this->current_stride_step_ % stride;

    std::memmove(
        (int8_t *) (tflite::GetTensorData<int8_t>(input)) + PREPROCESSOR_FEATURE_SIZE * this->current_stride_step_,
        features, PREPROCESSOR_FEATURE_SIZE);
    ++this->current_stride_step_;

    if (this->current_stride_step_ >= stride) {
      TfLiteStatus invoke_status = this->interpreter_->Invoke();
      if (invoke_status != kTfLiteOk) {
        ESP_LOGW(TAG, "interpreter invoke failed");
        return false;
      }

      TfLiteTensor *output = this->interpreter_->output(0);

      ++this->last_n_index_;
      if (this->last_n_index_ == this->sliding_window_size_) this->last_n_index_ = 0;
      this->recent_streaming_probabilities_[this->last_n_index_] = output->data.uint8[0];
      this->unprocessed_probability_status_ = true;
    }
    if (this->recent_streaming_probabilities_[this->last_n_index_] < this->probability_cutoff_) {
      this->ignore_windows_ = std::min<int16_t>(this->ignore_windows_ + 1, 0);
    }
  }
  return true;
}

void StreamingModel::reset_probabilities() {
  for (auto &prob : this->recent_streaming_probabilities_) prob = 0;
  this->ignore_windows_ = -MIN_SLICES_BEFORE_DETECTION;
}

WakeWordModel::WakeWordModel(const std::string &id,
                             const uint8_t *model_start,
                             uint8_t default_probability_cutoff,
                             size_t sliding_window_average_size,
                             const std::string &wake_word,
                             size_t tensor_arena_size,
                             bool default_enabled) {
  this->id_ = id;
  this->model_start_ = model_start;
  this->default_probability_cutoff_ = default_probability_cutoff;
  this->probability_cutoff_ = default_probability_cutoff;
  this->sliding_window_size_ = sliding_window_average_size;
  this->recent_streaming_probabilities_.resize(sliding_window_average_size, 0);
  this->wake_word_ = wake_word;
  this->tensor_arena_size_ = tensor_arena_size;
  this->register_streaming_ops_(this->streaming_op_resolver_);
  this->current_stride_step_ = 0;
  this->enabled_ = default_enabled;
}

DetectionEvent WakeWordModel::determine_detected() {
  DetectionEvent detection_event;
  detection_event.wake_word = &this->wake_word_;
  detection_event.max_probability = 0;
  detection_event.average_probability = 0;

  if ((this->ignore_windows_ < 0) || !this->enabled_) {
    detection_event.detected = false;
    return detection_event;
  }

  uint32_t sum = 0;
  for (auto &prob : this->recent_streaming_probabilities_) {
    detection_event.max_probability = std::max(detection_event.max_probability, prob);
    sum += prob;
  }

  detection_event.average_probability = sum / this->sliding_window_size_;
  detection_event.detected = sum > this->probability_cutoff_ * this->sliding_window_size_;

  this->unprocessed_probability_status_ = false;
  return detection_event;
}

VADModel::VADModel(const uint8_t *model_start, uint8_t default_probability_cutoff,
                   size_t sliding_window_size, size_t tensor_arena_size) {
  this->model_start_ = model_start;
  this->default_probability_cutoff_ = default_probability_cutoff;
  this->probability_cutoff_ = default_probability_cutoff;
  this->sliding_window_size_ = sliding_window_size;
  this->recent_streaming_probabilities_.resize(sliding_window_size, 0);
  this->tensor_arena_size_ = tensor_arena_size;
  this->register_streaming_ops_(this->streaming_op_resolver_);
}

DetectionEvent VADModel::determine_detected() {
  DetectionEvent detection_event;
  detection_event.max_probability = 0;
  detection_event.average_probability = 0;

  if (!this->enabled_) {
    detection_event.detected = true;
    return detection_event;
  }

  uint32_t sum = 0;
  for (auto &prob : this->recent_streaming_probabilities_) {
    detection_event.max_probability = std::max(detection_event.max_probability, prob);
    sum += prob;
  }

  detection_event.average_probability = sum / this->sliding_window_size_;
  detection_event.detected = sum > (this->probability_cutoff_ * this->sliding_window_size_);

  return detection_event;
}

bool StreamingModel::register_streaming_ops_(tflite::MicroMutableOpResolver<20> &op_resolver) {
  if (op_resolver.AddCallOnce() != kTfLiteOk) return false;
  if (op_resolver.AddVarHandle() != kTfLiteOk) return false;
  if (op_resolver.AddReshape() != kTfLiteOk) return false;
  if (op_resolver.AddReadVariable() != kTfLiteOk) return false;
  if (op_resolver.AddStridedSlice() != kTfLiteOk) return false;
  if (op_resolver.AddConcatenation() != kTfLiteOk) return false;
  if (op_resolver.AddAssignVariable() != kTfLiteOk) return false;
  if (op_resolver.AddConv2D() != kTfLiteOk) return false;
  if (op_resolver.AddMul() != kTfLiteOk) return false;
  if (op_resolver.AddAdd() != kTfLiteOk) return false;
  if (op_resolver.AddMean() != kTfLiteOk) return false;
  if (op_resolver.AddFullyConnected() != kTfLiteOk) return false;
  if (op_resolver.AddLogistic() != kTfLiteOk) return false;
  if (op_resolver.AddQuantize() != kTfLiteOk) return false;
  if (op_resolver.AddDepthwiseConv2D() != kTfLiteOk) return false;
  if (op_resolver.AddAveragePool2D() != kTfLiteOk) return false;
  if (op_resolver.AddMaxPool2D() != kTfLiteOk) return false;
  if (op_resolver.AddPad() != kTfLiteOk) return false;
  if (op_resolver.AddPack() != kTfLiteOk) return false;
  if (op_resolver.AddSplitV() != kTfLiteOk) return false;
  return true;
}
