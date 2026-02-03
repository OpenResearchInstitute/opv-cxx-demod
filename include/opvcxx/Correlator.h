// Copyright 2021 Rob Riggs <rob@mobilinkd.com>
// Copyright 2022-2026 Open Research Institute, Inc.
// All rights reserved.

#pragma once

#include "IirFilter.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstddef>
#include <type_traits>
#include <tuple>
#include <limits>
#include <iostream>

extern uint32_t debug_sample_count;

namespace mobilinkd {

template <typename FloatType>
struct Correlator
{
	// Changed from 8 to 12 symbols for new 24-bit OPV sync word (0x02B8DB)
	static constexpr size_t SYMBOLS = 12;
	static constexpr size_t SAMPLES_PER_SYMBOL = 10;

	using value_type = FloatType;
    using buffer_t = std::array<FloatType, SYMBOLS * SAMPLES_PER_SYMBOL>;
    using sync_t = std::array<int8_t, SYMBOLS>;
    using sample_filter_t = BaseIirFilter<FloatType, 3>;

    buffer_t buffer_;

    FloatType limit_ = 0.;
    size_t symbol_pos_ = 0;
    size_t buffer_pos_ = 0;
    size_t prev_buffer_pos_ = 0;
    int code = -1;

    // IIR with Nyquist of 1/240.
	// This is a Butterworth IIR lowpass filter, as designed by MATLAB's filter designer set for minimal taps.
	// The numerator (which is [1 2 1] in normal form) has the scaling factor incorporated for unity gain.
	static constexpr FloatType ugsf = 1.341681875516310e-06;	// unity gain scale factor
	static constexpr std::array<FloatType,3> b = {1.0 * ugsf, 2.0 * ugsf, 1.0 * ugsf};	//numerator
    static constexpr std::array<FloatType,3> a = {1.0, -1.99672112253324, 0.996726489260742};	// denominator

    sample_filter_t sample_filter{b, a};
    std::array<int, SYMBOLS> tmp;

    void sample(FloatType value)
    {
        limit_ = sample_filter(std::abs(value));

		// std::cerr << "@ " << debug_sample_count << " filtered_sample = " << value << " limit = " << limit_ << std::endl;	//!!!debug

        buffer_[buffer_pos_] = value;
        prev_buffer_pos_ = buffer_pos_;
        if (++buffer_pos_ == buffer_.size()) buffer_pos_ = 0;
    }

    FloatType correlate(sync_t sync)
    {
        FloatType result = 0.;
        size_t pos = prev_buffer_pos_ + SAMPLES_PER_SYMBOL;

        for (size_t i = 0; i != sync.size(); ++i)
        {
            if (pos >= buffer_.size())
                pos -= buffer_.size(); // wrapped
            result += sync[i] * buffer_[pos];
            pos += SAMPLES_PER_SYMBOL;
        }
        return result;
    }

    FloatType limit() const {return limit_;}
    size_t index() const {return prev_buffer_pos_ % SAMPLES_PER_SYMBOL;}

    /**
     * Get the average outer symbol levels at a given index.  This makes three
     * assumptions.
     *
     *  1. The max symbol value is above 0 and the min symbol value is below 0.
     *  2. The samples at the given index only contain outer symbols.
     *  3. The index is a peak correlation index.
     *
     * The first should hold true except for extreme frequency errors.  The
     * second holds true for the sync words used for M17 or OPV.  The third will
     * hold true if passed the timing index from a triggered sync word.
     */
    std::tuple<FloatType, FloatType> outer_symbol_levels(size_t sample_index)
    {
        FloatType min_sum = 0;
        FloatType max_sum = 0;
        size_t min_count = 0;
        size_t max_count = 0;
        size_t index = 0;
        for (size_t i = sample_index; i < buffer_.size(); i += SAMPLES_PER_SYMBOL)
        {
            tmp[index++] = buffer_[i] * 1000.;
            max_sum += buffer_[i] * ((buffer_[i] > 0.));
            min_sum += buffer_[i] * ((buffer_[i] < 0.));
            max_count += (buffer_[i] > 0.);
            min_count += (buffer_[i] < 0.);
        }

        return std::make_tuple(min_sum / min_count, max_sum / max_count);
    }


    template <typename F>
    void apply(F func, uint8_t index)
    {
    	for (size_t i = index; i < buffer_.size(); i += SAMPLES_PER_SYMBOL)
    	{
    		func(buffer_[i]);
    	}
    }
};

/**
 * SyncWord detector using correlation
 */
template <typename Correlator>
struct SyncWord
{
	static constexpr size_t SYMBOLS = Correlator::SYMBOLS;
	static constexpr size_t SAMPLES_PER_SYMBOL = Correlator::SAMPLES_PER_SYMBOL;
	using value_type = typename Correlator::value_type;

	using buffer_t = std::array<int8_t, SYMBOLS>;
	using sample_buffer_t = std::array<value_type, SAMPLES_PER_SYMBOL>;

	buffer_t sync_word_;
	sample_buffer_t samples_;
	size_t pos_ = 0;
	size_t timing_index_ = 0;
	bool triggered_ = false;
	int8_t updated_ = 0;
	value_type magnitude_1_ = 1.;
	value_type magnitude_2_ = -1.;

	SyncWord(buffer_t&& sync_word, value_type magnitude_1, value_type magnitude_2 = std::numeric_limits<value_type>::lowest())
	: sync_word_(std::move(sync_word)), magnitude_1_(magnitude_1), magnitude_2_(magnitude_2)
	{}

	value_type triggered(Correlator& correlator)
	{
		value_type limit_1 = correlator.limit() * magnitude_1_;
		value_type limit_2 = correlator.limit() * magnitude_2_;
		auto value = correlator.correlate(sync_word_);

		// std::cerr << "@ " << debug_sample_count << " triggered() value = " << value << " limit1 limit2 = " << limit_1 << " " << limit_2 << std::endl;

		return (value > limit_1 || value < limit_2) ? value : 0.0;
	}

	size_t operator()(Correlator& correlator)
	{
		auto value = triggered(correlator);

		value_type peak_value = 0;

		if (value != 0)
		{
			if (!triggered_)
			{
				samples_.fill(0);
				triggered_ = true;
			}
			samples_[correlator.index()] = value;
		}
		else
		{
			if (triggered_)
			{
				// Calculate the timing index on the falling edge.
				triggered_ = false;
				timing_index_ = 0;
				peak_value = value;
				uint8_t index = 0;
				for (auto f : samples_)
				{
					if (abs(f) > abs(peak_value))
					{
						peak_value = f;
						timing_index_ = index;
					}
					index += 1;
				}
				updated_ = peak_value > 0 ? 1 : -1;
			}
		}
		return timing_index_;
	}

	int8_t updated()
	{
		auto result = updated_;
		updated_ = 0;
		return result;
	}
};

} // mobilinkd
