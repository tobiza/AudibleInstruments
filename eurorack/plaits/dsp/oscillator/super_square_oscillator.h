// Copyright 2021 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// 
// See http://creativecommons.org/licenses/MIT/ for more information.
//
// -----------------------------------------------------------------------------
//
// Two hard-sync'ed square waves with a meta-parameter, also faking PWM.
// Based on VariableShapeOscillator, with hard-coded pulse width (0.5),
// waveshape (only square), and sync enabled by default.

#ifndef PLAITS_DSP_OSCILLATOR_SUPERSQUARE_OSCILLATOR_H_
#define PLAITS_DSP_OSCILLATOR_SUPERSQUARE_OSCILLATOR_H_

#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/parameter_interpolator.h"
#include "stmlib/dsp/polyblep.h"

#include <algorithm>

namespace plaits {

class SuperSquareOscillator {
 public:
  SuperSquareOscillator() { }
  ~SuperSquareOscillator() { }

  void Init() {
    master_phase_ = 0.0f;
    slave_phase_ = 0.0f;
    next_sample_ = 0.0f;
    high_ = false;
  
    master_frequency_ = 0.0f;
    slave_frequency_ = 0.01f;
  }
  
  void Render(
      float frequency,
      float shape,
      float* out,
      size_t size) {
    float master_frequency = frequency;
    frequency *= shape < 0.5f
        ? (0.51f + 0.98f * shape)
        : 1.0f + 16.0f * (shape - 0.5f) * (shape - 0.5f);

    if (master_frequency >= kMaxFrequency) {
      master_frequency = kMaxFrequency;
    }
    
    if (frequency >= kMaxFrequency) {
      frequency = kMaxFrequency;
    }
    
    stmlib::ParameterInterpolator master_fm(
        &master_frequency_, master_frequency, size);
    stmlib::ParameterInterpolator fm(&slave_frequency_, frequency, size);

    float next_sample = next_sample_;
    
    while (size--) {
      bool reset = false;
      bool transition_during_reset = false;
      float reset_time = 0.0f;

      float this_sample = next_sample;
      next_sample = 0.0f;
    
      const float master_frequency = master_fm.Next();
      const float slave_frequency = fm.Next();

      master_phase_ += master_frequency;
      if (master_phase_ >= 1.0f) {
        master_phase_ -= 1.0f;
        reset_time = master_phase_ / master_frequency;
      
        float slave_phase_at_reset = slave_phase_ + \
            (1.0f - reset_time) * slave_frequency;
        reset = true;
        if (slave_phase_at_reset >= 1.0f) {
          slave_phase_at_reset -= 1.0f;
          transition_during_reset = true;
        }
        if (!high_ && slave_phase_at_reset >= 0.5f) {
          transition_during_reset = true;
        }
        float value = slave_phase_at_reset < 0.5f ? 0.0f : 1.0f;
        this_sample -= value * stmlib::ThisBlepSample(reset_time);
        next_sample -= value * stmlib::NextBlepSample(reset_time);
      }
      
      slave_phase_ += slave_frequency;
      while (transition_during_reset || !reset) {
        if (!high_) {
          if (slave_phase_ < 0.5f) {
            break;
          }
          float t = (slave_phase_ - 0.5f) / slave_frequency;
          this_sample += stmlib::ThisBlepSample(t);
          next_sample += stmlib::NextBlepSample(t);
          high_ = true;
        }
      
        if (high_) {
          if (slave_phase_ < 1.0f) {
            break;
          }
          slave_phase_ -= 1.0f;
          float t = slave_phase_ / slave_frequency;
          this_sample -= stmlib::ThisBlepSample(t);
          next_sample -= stmlib::NextBlepSample(t);
          high_ = false;
        }
      }
    
      if (reset) {
        slave_phase_ = reset_time * slave_frequency;
        high_ = false;
      }
    
      next_sample += slave_phase_ < 0.5f ? 0.0f : 1.0f;
      *out++ = 2.0f * this_sample - 1.0f;
    }
    
    next_sample_ = next_sample;
  }

 private:
  // Oscillator state.
  float master_phase_;
  float slave_phase_;
  float next_sample_;
  bool high_;

  // For interpolation of parameters.
  float master_frequency_;
  float slave_frequency_;

  DISALLOW_COPY_AND_ASSIGN(SuperSquareOscillator);
};
  
}  // namespace plaits

#endif  // PLAITS_DSP_OSCILLATOR_SUPERSQUARE_OSCILLATOR_H_
