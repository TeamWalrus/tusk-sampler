// Example use:
// Show bits: (stdout)
// clang++ -O3 -o extract extract.cpp && ./extract < analog.csv 2> /dev/null
// Plot graph: (stderr)
// (clang++ -O3 -o extract extract.cpp && ./extract < analog.csv 2>&1
// >/dev/null) | head -n50000 | ./plot.py

#include <Arduino.h>
#include <cassert>
#include <cmath>
#include <cstdlib>
// #include <iostream>
// #include <queue>
// #include <string>
// #include <vector>

class GlitchFilter {
public:
  // `time` is how long a transition has to hold to be valid
  explicit GlitchFilter(const double time) : time_(time) {}

  // Takes a sample time and sample value; outputs glitch filtered sample
  unsigned int filter(const unsigned long t_us, const bool in) {
    if (in != out_) {
      // Transition detected, record time
      last_flip_in_ = in;
      last_flip_t_us_ = t_us;
    }

    if ((t_us - last_flip_t_us_) / 1e6 >= time_) {
      // In current state for at least `time_`, so latch it
      out_ = last_flip_in_;
    }

    // TODO: Wtf why do we have to invert this
    return !out_;
  }

private:
  const double time_;

  bool out_ = false;

  bool last_flip_in_ = false;
  unsigned long last_flip_t_us_ = 0;
};

class Extractor {
public:
  // `bitrate` is how many bits are expected per second
  explicit Extractor(const double bitrate) : bitrate_(bitrate) {}

  // Takes a sample time and sample value; outputs 0 or 1 if a bit is output,
  // else -1
  int maybe_extract(const unsigned long t_us, const bool high) {
    // Increase the current phase according to the difference between sample
    // time and last sample time
    phase_ += ((t_us - last_t_us_) / 1e6) / (1.0 / bitrate_);
    if (high != last_high_) {
      // Transition detected, so adjust phase to correct any error.
      // We want transitions to occur with phase = 0.5, so adjust phase by the
      // difference to 0.5, scaled by 1/10
      phase_ += (0.5 - phase_) / 10;
    }

    // Split off the integer part of the phase and keep the integral in the
    // phase
    double phase_int;
    phase_ = modf(phase_, &phase_int);
    // If the integer part is non-zero, the phase is at least 1.0, and it's time
    // to sample; output the given input if so
    const auto res = phase_int > 0 ? high : -1;

    last_high_ = high;
    last_t_us_ = t_us;

    return res;
  }

  // For debugging
  const double phase() const { return phase_; }

private:
  const double bitrate_;

  // The phase is a floating point value between 0 and 1; we want transitions to
  // occur at 0.5, and we sample when we overflow past 1 back to 0
  double phase_ = 0;

  unsigned long last_t_us_ = 0;
  bool last_high_ = false;
};

class Syncer {
public:
  // `pattern` is the pattern of bits to look for (as `unsigned int`s rather
  // than `bools` to avoid using `std::vector<bool>`. `len` is the length of
  // data to return after the pattern is seen
  Syncer(const unsigned int *pattern, const unsigned int pattern_len,
         const unsigned int len)
      : pattern_(pattern), pattern_len_(pattern_len),
        buf_(new unsigned int[pattern_len + len]), buf_len_(pattern_len + len) {
  }

  bool maybe_sync(const unsigned int bit, unsigned int *out,
                  unsigned int *out_len) {
    *out_len = 0;

    // Store the input bit in the circular buffer at the next position
    buf_[next_pos_] = bit;
    // Move the next position along, wrapping around
    next_pos_ = (next_pos_ + 1) % buf_len_;

    // Check all the pattern bits. If any of the bits in the circular buffer
    // just past the next position don't match the pattern, then we don't have a
    // match
    // Example: s = sync, d = data
    // buf_: s s s s d d d d d d d d s s s s s s
    //       -+-----                 ^----------
    //        |                      next_pos_
    //       bits that are checked
    for (auto i = 0; i < pattern_len_; ++i) {
      if (buf_[(next_pos_ + i) % buf_len_] != pattern_[i]) {
        return false;
      }
    }

    // We have a match, store the data in the output
    // Example: s = sync, d = data
    // buf_: s s s s d d d d d d d d s s s s s s
    //               -+------------- ^
    //                |              next_pos_
    //               bits that are output
    for (auto i = pattern_len_; i < buf_len_; ++i) {
      out[i - pattern_len_] = buf_[(next_pos_ + i) % buf_len_];
    }
    *out_len = buf_len_ - pattern_len_;

    return true;
  }

  // Used to create a `Syncer` the same as the current one, but with inverted
  // pattern. This is for debugging/dev purposes, so we can still find data even
  // if we have the polarity of the incoming samples the wrong way around
  Syncer inverse() const {
    unsigned int *inverse_pattern = new unsigned int[pattern_len_];
    for (auto i = 0; i < pattern_len_; ++i) {
      inverse_pattern[i] = !pattern_[i];
    }
    return Syncer(inverse_pattern, pattern_len_, buf_len_ - pattern_len_);
  }

private:
  const unsigned int *pattern_;
  const unsigned int pattern_len_;

  unsigned int *buf_;
  unsigned int buf_len_;
  unsigned int next_pos_ = 0;
};

// Decodes the manchester-encoded data in `in` to `out`. Returns whether or not
// it was succcessful
bool manchester_decode(const unsigned int *in, const unsigned int in_len,
                       unsigned int *out, unsigned int *out_len) {
  *out_len = 0;

  if (in_len % 2) {
    // Input isn't a multiple of two, can't ever be correct
    return false;
  }

  for (auto i = 0; i < in_len; i += 2) {
    if (!in[i] && in[i + 1]) {
      // Saw a 0 followed by a 1, output 0
      out[(*out_len)++] = 0;
    } else if (in[i] && !in[i + 1]) {
      // Saw a 1 followed by a 0, output 1
      out[(*out_len)++] = 1;
    } else {
      // Must have seen a 0 followed by a 0 or a 1 followed by a 1, the data is
      // corrupt
      *out_len = 0;
      return false;
    }
  }

  return true;
}

const int input_pin = 32;

void setup() {
  pinMode(input_pin, INPUT);
  Serial.begin(115200);
}

const unsigned int hid_prox_pattern[] = {0, 0, 0, 1, 1, 1, 0, 1};

#define ARRAY_SIZEOF(a) (sizeof(a) / sizeof((a)[0]))

void loop() {
#if 1
  // HID Prox: RF/50 = 2.5kb/s
  GlitchFilter glitch_filter((1 / (125000.0 / 50)) / 2);
  Extractor extractor(125000.0 / 50);

  Syncer syncer(hid_prox_pattern, ARRAY_SIZEOF(hid_prox_pattern),
                26 * 2); // 26-bit
#else
  // Gallagher: RF/32 ~= 3.9kb/s
  GlitchFilter glitch_filter((1 / (125000.0 / 32)) / 2);
  Extractor extractor(125000.0 / 32);

  Syncer syncer({0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0}, 16,
                (8 * (8 + 1) + 8) * 2);
#endif

  Syncer inverse_syncer = syncer.inverse();

  auto t_us_start = 0.0;
  auto first = true;

  /*for (;;) {
    const auto high = digitalRead(input_pin) == HIGH;
    Serial.println(high);
  }*/

  for (;;) {
    // Get the time in microseconds (1s = 100,000 us)
    auto t_us = micros();
    if (first) {
      // This is the first line, store the time so we can subtract this all
      // sample times so that the samples effectively start from time 0 (makes
      // the debug output graph data nicer)
      t_us_start = t_us;
      first = false;
    }
    // Subtract first sample time as per above comment
    t_us -= t_us_start;

    // We're reading high is voltage is >= 2.5V (assuming 5V logic)
    const auto high = digitalRead(input_pin) == HIGH;
    // Filter the read value though glitch filter
    const auto filtered_high = glitch_filter.filter(t_us, high);
    // Maybe extract a bit if it's time to sample
    const auto out = extractor.maybe_extract(t_us, filtered_high);

    if (out != -1) {
      // We've sampled a new bit
      // Output it to stdout
      Serial.print(out);

      unsigned int sync_out[128];
      unsigned int sync_out_len;

      // Try to sync to the positive version of the sync pattern
      if (syncer.maybe_sync(out, sync_out, &sync_out_len)) {
        // We've synced, output message
        Serial.println();
        Serial.print("Positive sync: ");

        // Try to manchester decode the data following the sync pattern
        unsigned int manchester_out[128];
        unsigned int manchester_out_len;
        if (manchester_decode(sync_out, sync_out_len, manchester_out,
                              &manchester_out_len)) {
          // Cool, it worked, output it
          for (auto i = 0; i < manchester_out_len; ++i) {
            Serial.print(manchester_out[i]);
          }
        } else {
          // Oops, couldn't manchester decode, just output the raw data for
          // debugging
          Serial.print("bad manchester: ");
          for (auto i = 0; i < sync_out_len; ++i) {
            Serial.print(sync_out[i]);
          }
        }
        Serial.println();
      }

      // Do the same as above, but with inverted sync pattern for debugging
      if (inverse_syncer.maybe_sync(out, sync_out, &sync_out_len)) {
        // We've synced, output message
        Serial.println();
        Serial.print("Negative sync: ");

        // Try to manchester decode the data following the sync pattern
        unsigned int manchester_out[128];
        unsigned int manchester_out_len;
        if (manchester_decode(sync_out, sync_out_len, manchester_out,
                              &manchester_out_len)) {
          // Cool, it worked, output it
          for (auto i = 0; i < manchester_out_len; ++i) {
            Serial.print(manchester_out[i]);
          }
        } else {
          // Oops, couldn't manchester decode, just output the raw data for
          // debugging
          Serial.print("bad manchester: ");
          for (auto i = 0; i < sync_out_len; ++i) {
            Serial.print(sync_out[i]);
          }
        }
        Serial.println();
      }
    }
  }
}
