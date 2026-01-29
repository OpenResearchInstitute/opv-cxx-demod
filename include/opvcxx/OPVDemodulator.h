// Copyright 2020-2021 Rob Riggs <rob@mobilinkd.com>
// Copyright 2022-2026 Open Research Institute, Inc.
// All rights reserved.
//
// Modified for HDL-aligned pipeline:
//   - 24-bit sync word (0x02B8DB) = 12 symbols
//   - 2144-bit frame size (67×32 interleaver)

#pragma once

#include "ClockRecovery.h"
#include "Correlator.h"
#include "DataCarrierDetect.h"
#include "FirFilter.h"
#include "FreqDevEstimator.h"
#include "OPVCobsDecoder.h"
#include "OPVFrameDecoder.h"
#include "OPVFramer.h"
#include "Util.h"
#include "Numerology.h"

#include <algorithm>
#include <array>
#include <functional>
#include <optional>
#include <tuple>

extern OPVCobsDecoder cobs_decoder;

namespace mobilinkd {

namespace detail
{

template <typename FloatType>
struct Taps
{};

template <>
struct Taps<double>
{
	static constexpr auto rrc_taps = std::array<double, 150>{
		0.0029364388513841593, 0.0031468394550958484, 0.002699564567597445, 0.001661182944400927,
		0.00023319405581230247, -0.0012851320781224025, -0.0025577136087664687, -0.0032843366522956313,
		-0.0032697038088887226, -0.0024733964729590865, -0.0010285696910973807, 0.0007766690889758685,
		0.002553421969211845, 0.0038920145144327816, 0.004451886520053017, 0.00404219185231544,
		0.002674727068399207, 0.0005756567993179152, -0.0018493784971116507, -0.004092346891623224,
		-0.005648131453822014, -0.006126925416243605, -0.005349511529163396, -0.003403189203405097,
		-0.0006430502751187517, 0.002365929161655135, 0.004957956568090113, 0.006506845894531803,
		0.006569574194782443, 0.0050017573119839134, 0.002017321931508163, -0.0018256054303579805,
		-0.00571615173291049, -0.008746639552588416, -0.010105075751866371, -0.009265784007800534,
		-0.006136551625729697, -0.001125978562075172, 0.004891777252042491, 0.01071805138282269,
		0.01505751553351295, 0.01679337935001369, 0.015256245142156299, 0.01042830577908502,
		0.003031522725559901, -0.0055333532968188165, -0.013403099825723372, -0.018598682349642525,
		-0.01944761739590459, -0.015005271935951746, -0.0053887880354343935, 0.008056525910253532,
		0.022816244158307273, 0.035513467692208076, 0.04244131815783876, 0.04025481153629372,
		0.02671818654865632, 0.0013810216516704976, -0.03394615682795165, -0.07502635967975885,
		-0.11540977897637611, -0.14703962203941534, -0.16119995609538576, -0.14969512896336504,
		-0.10610329539459686, -0.026921412469634916, 0.08757875030779196, 0.23293327870303457,
		0.4006012210123992, 0.5786324696325503, 0.7528286479934068, 0.908262741447522,
		1.0309661131633199, 1.1095611856548013, 1.1366197723675815, 1.1095611856548013,
		1.0309661131633199, 0.908262741447522, 0.7528286479934068, 0.5786324696325503,
		0.4006012210123992, 0.23293327870303457, 0.08757875030779196, -0.026921412469634916,
		-0.10610329539459686, -0.14969512896336504, -0.16119995609538576, -0.14703962203941534,
		-0.11540977897637611, -0.07502635967975885, -0.03394615682795165, 0.0013810216516704976,
		0.02671818654865632, 0.04025481153629372, 0.04244131815783876, 0.035513467692208076,
		0.022816244158307273, 0.008056525910253532, -0.0053887880354343935, -0.015005271935951746,
		-0.01944761739590459, -0.018598682349642525, -0.013403099825723372, -0.0055333532968188165,
		0.003031522725559901, 0.01042830577908502, 0.015256245142156299, 0.01679337935001369,
		0.01505751553351295, 0.01071805138282269, 0.004891777252042491, -0.001125978562075172,
		-0.006136551625729697, -0.009265784007800534, -0.010105075751866371, -0.008746639552588416,
		-0.00571615173291049, -0.0018256054303579805, 0.002017321931508163, 0.0050017573119839134,
		0.006569574194782443, 0.006506845894531803, 0.004957956568090113, 0.002365929161655135,
		-0.0006430502751187517, -0.003403189203405097, -0.005349511529163396, -0.006126925416243605,
		-0.005648131453822014, -0.004092346891623224, -0.0018493784971116507, 0.0005756567993179152,
		0.002674727068399207, 0.00404219185231544, 0.004451886520053017, 0.0038920145144327816,
		0.002553421969211845, 0.0007766690889758685, -0.0010285696910973807, -0.0024733964729590865,
		-0.0032697038088887226, -0.0032843366522956313, -0.0025577136087664687, -0.0012851320781224025,
		0.00023319405581230247, 0.001661182944400927, 0.002699564567597445, 0.0031468394550958484,
		0.0029364388513841593, 0.0
	};
};

template <>
struct Taps<float>
{
	static constexpr auto rrc_taps = std::array<float, 150>{
		0.0029364388513841593, 0.0031468394550958484, 0.002699564567597445, 0.001661182944400927,
		0.00023319405581230247, -0.0012851320781224025, -0.0025577136087664687, -0.0032843366522956313,
		-0.0032697038088887226, -0.0024733964729590865, -0.0010285696910973807, 0.0007766690889758685,
		0.002553421969211845, 0.0038920145144327816, 0.004451886520053017, 0.00404219185231544,
		0.002674727068399207, 0.0005756567993179152, -0.0018493784971116507, -0.004092346891623224,
		-0.005648131453822014, -0.006126925416243605, -0.005349511529163396, -0.003403189203405097,
		-0.0006430502751187517, 0.002365929161655135, 0.004957956568090113, 0.006506845894531803,
		0.006569574194782443, 0.0050017573119839134, 0.002017321931508163, -0.0018256054303579805,
		-0.00571615173291049, -0.008746639552588416, -0.010105075751866371, -0.009265784007800534,
		-0.006136551625729697, -0.001125978562075172, 0.004891777252042491, 0.01071805138282269,
		0.01505751553351295, 0.01679337935001369, 0.015256245142156299, 0.01042830577908502,
		0.003031522725559901, -0.0055333532968188165, -0.013403099825723372, -0.018598682349642525,
		-0.01944761739590459, -0.015005271935951746, -0.0053887880354343935, 0.008056525910253532,
		0.022816244158307273, 0.035513467692208076, 0.04244131815783876, 0.04025481153629372,
		0.02671818654865632, 0.0013810216516704976, -0.03394615682795165, -0.07502635967975885,
		-0.11540977897637611, -0.14703962203941534, -0.16119995609538576, -0.14969512896336504,
		-0.10610329539459686, -0.026921412469634916, 0.08757875030779196, 0.23293327870303457,
		0.4006012210123992, 0.5786324696325503, 0.7528286479934068, 0.908262741447522,
		1.0309661131633199, 1.1095611856548013, 1.1366197723675815, 1.1095611856548013,
		1.0309661131633199, 0.908262741447522, 0.7528286479934068, 0.5786324696325503,
		0.4006012210123992, 0.23293327870303457, 0.08757875030779196, -0.026921412469634916,
		-0.10610329539459686, -0.14969512896336504, -0.16119995609538576, -0.14703962203941534,
		-0.11540977897637611, -0.07502635967975885, -0.03394615682795165, 0.0013810216516704976,
		0.02671818654865632, 0.04025481153629372, 0.04244131815783876, 0.035513467692208076,
		0.022816244158307273, 0.008056525910253532, -0.0053887880354343935, -0.015005271935951746,
		-0.01944761739590459, -0.018598682349642525, -0.013403099825723372, -0.0055333532968188165,
		0.003031522725559901, 0.01042830577908502, 0.015256245142156299, 0.01679337935001369,
		0.01505751553351295, 0.01071805138282269, 0.004891777252042491, -0.001125978562075172,
		-0.006136551625729697, -0.009265784007800534, -0.010105075751866371, -0.008746639552588416,
		-0.00571615173291049, -0.0018256054303579805, 0.002017321931508163, 0.0050017573119839134,
		0.006569574194782443, 0.006506845894531803, 0.004957956568090113, 0.002365929161655135,
		-0.0006430502751187517, -0.003403189203405097, -0.005349511529163396, -0.006126925416243605,
		-0.005648131453822014, -0.004092346891623224, -0.0018493784971116507, 0.0005756567993179152,
		0.002674727068399207, 0.00404219185231544, 0.004451886520053017, 0.0038920145144327816,
		0.002553421969211845, 0.0007766690889758685, -0.0010285696910973807, -0.0024733964729590865,
		-0.0032697038088887226, -0.0032843366522956313, -0.0025577136087664687, -0.0012851320781224025,
		0.00023319405581230247, 0.001661182944400927, 0.002699564567597445, 0.0031468394550958484,
		0.0029364388513841593, 0.0
	};
};

} // detail

// =============================================================================
// OPV DEMODULATOR CONSTANTS (HDL-aligned)
// =============================================================================

// New sync word: 0x02B8DB (24 bits = 12 symbols)
// Binary: 0000 0010 1011 1000 1101 1011
// Dibits: 00 00 00 10 10 11 10 00 11 01 10 11
// Symbols (00→+1, 01→+3, 10→-1, 11→-3):
//   +1, +1, +1, -1, -1, -3, -1, +1, -3, +3, -1, -3
constexpr size_t OPV_SYNC_SYMBOLS = 12;

// Frame timing: 24-bit sync + 2144-bit payload = 2168 bits = 1084 symbols
constexpr size_t OPV_FRAME_BITS = opv_sync_bits + opv_encoded_bits;  // 24 + 2144 = 2168
constexpr size_t OPV_FRAME_SYMBOLS_TOTAL = OPV_FRAME_BITS / 2;       // 1084 symbols
constexpr size_t OPV_SAMPLES_PER_SYMBOL = 10;
constexpr size_t OPV_SAMPLES_PER_FRAME = OPV_FRAME_SYMBOLS_TOTAL * OPV_SAMPLES_PER_SYMBOL;  // 10840

// Symbol rate for new frame timing
constexpr size_t opv_symbol_rate = OPV_FRAME_SYMBOLS_TOTAL * 25;  // 25 frames/sec = 27100 symbols/sec
constexpr size_t opv_sample_rate = opv_symbol_rate * OPV_SAMPLES_PER_SYMBOL;  // 271000 samples/sec


template <typename FloatType>
struct OPVDemodulator
{

	static constexpr uint8_t MAX_MISSING_SYNC = 8;
	static constexpr FloatType CORRELATION_NEAR_ZERO = 0.1;

	// Correlator now has SYMBOLS=12 built-in for new 24-bit sync word
	using correlator_t = Correlator<FloatType>;
	using sync_word_t = SyncWord<correlator_t>;
	using callback_t = OPVFrameDecoder::callback_t;
	using diagnostic_callback_t = std::function<void(bool, FloatType, FloatType, FloatType, bool, FloatType, int, int, int, int)>;

	enum class DemodState { UNLOCKED, FIRST_SYNC, STREAM_SYNC, FRAME };

	BaseFirFilter<FloatType, detail::Taps<FloatType>::rrc_taps.size()> demod_filter{detail::Taps<FloatType>::rrc_taps};
	DataCarrierDetect<FloatType, sample_rate, 500> dcd{13500, 21500, 1.0, 4.0};
	
	ClockRecovery<FloatType, sample_rate, symbol_rate> clock_recovery;

	correlator_t correlator;
	
	// Preamble: alternating +3/-3 pattern (12 symbols)
	// Energy = 12 * 9 = 108, threshold ~33% of max
	sync_word_t preamble_sync{{+3,-3,+3,-3,+3,-3,+3,-3,+3,-3,+3,-3}, 36.f};
	
	// Stream sync: 0x02B8DB = {+1,+1,+1,-1,-1,-3,-1,+1,-3,+3,-1,-3}
	// Energy = 44, threshold ~45% of max for reliable detection
	sync_word_t stream_sync{{+1,+1,+1,-1,-1,-3,-1,+1,-3,+3,-1,-3}, 20.f};

	FreqDevEstimator<FloatType> dev;
	FloatType idev;
	size_t count_ = 0;

	int8_t polarity = 1;
	OPVFramer<opv_encoded_bits> framer;  // Use new frame size (2144 bits)
	OPVFrameDecoder decoder;
	DemodState demodState = DemodState::UNLOCKED;
	uint8_t sample_index = 0;

	bool dcd_ = false;
	bool need_clock_reset_ = false;
	bool need_clock_update_ = false;

	bool passall_ = false;
	size_t viterbi_cost = 0;
	int sync_count = 0;
	int missing_sync_count = 0;
	uint8_t sync_sample_index = 0;
	diagnostic_callback_t diagnostic_callback;

	OPVDemodulator(callback_t callback)
	: decoder(callback)
	{}

	virtual ~OPVDemodulator() {}

	void dcd_on();
	void dcd_off();
	void initialize(const FloatType input);
	void update_dcd();
	void do_unlocked();
	void do_first_sync();
	void do_stream_sync();
	void do_frame(FloatType filtered_sample);

	bool locked() const
	{
		return dcd_;
	}

	void passall(bool enabled)
	{
	passall_ = enabled;
	}

	void diagnostics(diagnostic_callback_t callback)
	{
		diagnostic_callback = callback;
	}

	void update_values(uint8_t index);

	void operator()(const FloatType input);
};

template <typename FloatType>
void OPVDemodulator<FloatType>::update_values(uint8_t index)
{
	correlator.apply([this,index](FloatType t){dev.sample(t);}, index);
	dev.update();
	sync_sample_index = index;
}

template <typename FloatType>
void OPVDemodulator<FloatType>::dcd_on()
{
	dcd_ = true;
	sync_count = 0;
	missing_sync_count = 0;

	dev.reset();
	framer.reset();
	decoder.reset();
}

template <typename FloatType>
void OPVDemodulator<FloatType>::dcd_off()
{
	dcd_ = false;
	demodState = DemodState::UNLOCKED;
	std::cerr << "DCD lost at sample " << debug_sample_count << " (" << float(debug_sample_count)/OPV_SAMPLES_PER_FRAME << " frames)" << std::endl;
}

template <typename FloatType>
void OPVDemodulator<FloatType>::initialize(const FloatType input)
{
	auto filtered_sample = demod_filter(input);
	correlator.sample(filtered_sample);
}

template <typename FloatType>
void OPVDemodulator<FloatType>::update_dcd()
{
	if (!dcd_ && dcd.dcd())
	{
		dcd_on();
		need_clock_reset_ = true;
	}
	else if (dcd_ && !dcd.dcd())
	{
		dcd_off();
	}
}

template <typename FloatType>
void OPVDemodulator<FloatType>::do_unlocked()
{
	// We expect to find the preamble immediately after DCD.
	if (missing_sync_count < OPV_SAMPLES_PER_FRAME)
	{
		missing_sync_count += 1;
		auto sync_index = preamble_sync(correlator);
		auto sync_updated = preamble_sync.updated();
		if (sync_updated)
		{
			std::cerr << "Detected preamble at sample " << debug_sample_count << " (" << float(debug_sample_count)/OPV_SAMPLES_PER_FRAME << " frames)" << std::endl;
			sync_count = 0;
			missing_sync_count = 0;
			need_clock_reset_ = true;
			dev.reset();
			update_values(sync_index);
			sample_index = sync_index;
			demodState = DemodState::FIRST_SYNC;
		}
		return;
	}

	// We didn't find preamble; check for the STREAM syncword in case we're joining in the middle
	auto sync_index = stream_sync(correlator);
	auto sync_updated = stream_sync.updated();
	if (sync_updated)
	{
		std::cerr << "Stream sync detected while unlocked at sample " << debug_sample_count << " (" << float(debug_sample_count)/OPV_SAMPLES_PER_FRAME << " frames)" << std::endl;

		sync_count = 0;
		missing_sync_count = 0;
		need_clock_reset_ = true;
		dev.reset();
		update_values(sync_index);
		sample_index = sync_index;
		cobs_decoder.reset();
		demodState = DemodState::FRAME;
		return;
	}
}


template <typename FloatType>
void OPVDemodulator<FloatType>::do_first_sync()
{
	FloatType sync_triggered;

	if (correlator.index() != sample_index) return;

	sync_triggered = preamble_sync.triggered(correlator);
	if (sync_triggered > CORRELATION_NEAR_ZERO)
	{
		return;		// Seeing preamble; keep looking.
	}

	// Now check for the STREAM syncword.
	sync_triggered = stream_sync.triggered(correlator);
	if (sync_triggered > CORRELATION_NEAR_ZERO)
	{
		std::cerr << "Detected first STREAM sync word at sample " << debug_sample_count  << " (" << float(debug_sample_count)/OPV_SAMPLES_PER_FRAME << " frames)" << std::endl;
		missing_sync_count = 0;
		need_clock_update_ = true;
		update_values(sample_index);
		cobs_decoder.reset();
		demodState = DemodState::FRAME;
	}
	else
	{
		// Frame symbols for timing window (12 sync symbols + some margin)
		constexpr size_t FRAME_SYMBOL_WINDOW = OPV_FRAME_SYMBOLS_TOTAL + OPV_SYNC_SYMBOLS;
		if (++missing_sync_count > FRAME_SYMBOL_WINDOW)
		{
			std::cerr << "FAILED to find first syncword by sample " << debug_sample_count << " (" << float(debug_sample_count)/OPV_SAMPLES_PER_FRAME << " frames)" << std::endl;
			demodState = DemodState::UNLOCKED;
			missing_sync_count = 0;
		}
		else
		{
			update_values(sample_index);
		}
	}
}


template <typename FloatType>
void OPVDemodulator<FloatType>::do_stream_sync()
{
	uint8_t sync_index = stream_sync(correlator);
	int8_t sync_updated = stream_sync.updated();
	sync_count += 1;
	
	// Timing windows adjusted for 12-symbol sync word (120 samples)
	// Old 8-symbol window: samples 71-87
	// New 12-symbol window: samples 107-127 (approx)
	constexpr int SYNC_WINDOW_START = OPV_SYNC_SYMBOLS * OPV_SAMPLES_PER_SYMBOL - 13;  // ~107
	constexpr int SYNC_WINDOW_END = OPV_SYNC_SYMBOLS * OPV_SAMPLES_PER_SYMBOL + 7;     // ~127
	
	if (sync_updated)
	{
		missing_sync_count = 0;
		if (sync_count > SYNC_WINDOW_START)
		{
			std::cerr << "Detected STREAM sync word at sample " << debug_sample_count  << " (" << float(debug_sample_count)/OPV_SAMPLES_PER_FRAME << " frames)" << std::endl;
			update_values(sync_index);
			demodState = DemodState::FRAME;
		}
		return;
	}
	else if (sync_count > SYNC_WINDOW_END)
	{
		update_values(sync_index);
		missing_sync_count += 1;
		if (missing_sync_count < MAX_MISSING_SYNC)
		{
			std::cerr << "Faking a STREAM sync word " << missing_sync_count << " at sample " << debug_sample_count << " (" << float(debug_sample_count)/OPV_SAMPLES_PER_FRAME << " frames)" << std::endl;
			demodState = DemodState::FRAME;
		}
		else
		{
			std::cerr << "Done faking sync words at sample " << debug_sample_count << " (" << float(debug_sample_count)/OPV_SAMPLES_PER_FRAME << " frames)" << std::endl;
			demodState = DemodState::FIRST_SYNC;
		}
	}
}


template <typename FloatType>
void OPVDemodulator<FloatType>::do_frame(FloatType filtered_sample)
{
	if (correlator.index() != sample_index) return;

	static uint8_t cost_count = 0;

	auto sample = filtered_sample - dev.offset();
	sample *= dev.idev();
	sample *= polarity;

	auto llr_symbol = llr<FloatType, 4>(sample);

	int8_t* framer_buffer_ptr;
	auto len = framer(llr_symbol, &framer_buffer_ptr);
	if (len != 0)
	{
		assert(len == opv_encoded_bits);  // 2144 bits

		need_clock_update_ = true;

		OPVFrameDecoder::frame_type4_buffer_t buffer;
		std::copy(framer_buffer_ptr, framer_buffer_ptr + len, buffer.begin());
		auto frame_decode_result = decoder(buffer, viterbi_cost);

		cost_count = viterbi_cost > 90 ? cost_count + 1 : 0;
		cost_count = viterbi_cost > 100 ? cost_count + 1 : cost_count;
		cost_count = viterbi_cost > 110 ? cost_count + 1 : cost_count;

		if (cost_count > 75)
		{
			std::cerr << "Viterbi cost high too long at sample " << debug_sample_count << " (" << float(debug_sample_count)/OPV_SAMPLES_PER_FRAME << " frames)" << std::endl;
			cost_count = 0;
			demodState = DemodState::UNLOCKED;
			return;
		}

		sync_count = 0;

		switch (frame_decode_result)
		{
		case OPVFrameDecoder::DecodeResult::EOS:
			std::cerr << "EOS at sample " << debug_sample_count << " (" << float(debug_sample_count)/OPV_SAMPLES_PER_FRAME << " frames)" << std::endl;
			demodState = DemodState::FIRST_SYNC;
			break;
		case OPVFrameDecoder::DecodeResult::OK:
			demodState = DemodState::STREAM_SYNC;
			break;
		case OPVFrameDecoder::DecodeResult::FAIL:
			// Handle FAIL case (was missing in original code)
			std::cerr << "Frame decode FAIL at sample " << debug_sample_count << std::endl;
			break;
		}
	}
}

template <typename FloatType>
void OPVDemodulator<FloatType>::operator()(const FloatType input)
{
	static int16_t initializing = OPV_SAMPLES_PER_FRAME;
	static bool initialized = false;

	count_++;

	dcd(input);

	if (initializing)
	{
		--initializing;
		initialize(input);
		count_ = 0;
		return;
	}

	if (! initialized) std::cerr << "Initialize complete at sample " << debug_sample_count << " (" << float(debug_sample_count)/OPV_SAMPLES_PER_FRAME << " frames)" << std::endl;
	initialized = true;

	if (!dcd_)
	{
		if (count_ % (OPV_FRAME_SYMBOLS_TOTAL * 2) == 0)
		{
			update_dcd();
			dcd.update();
			if (diagnostic_callback)
			{
				diagnostic_callback(int(dcd_), dev.error(), dev.deviation(), dev.offset(), (demodState != DemodState::UNLOCKED),
					clock_recovery.clock_estimate(), sample_index, sync_sample_index, clock_recovery.sample_index(), viterbi_cost);
			}
			count_ = 0;
		}
		return;
	}

	auto filtered_sample = demod_filter(input);

	correlator.sample(filtered_sample);

	if (correlator.index() == 0)
	{
		if (need_clock_reset_)
		{
			clock_recovery.reset();
			need_clock_reset_ = false;
		}
		else if (need_clock_update_)
		{
			clock_recovery.update();
			uint8_t clock_index = clock_recovery.sample_index();
			uint8_t clock_diff = std::abs(sample_index - clock_index);
			uint8_t sync_diff = std::abs(sample_index - sync_sample_index);
			bool clock_diff_ok = clock_diff <= 1 || clock_diff == 9;
			bool sync_diff_ok = sync_diff <= 1 || sync_diff == 9;
			if (clock_diff_ok) sample_index = clock_index;
			else if (sync_diff_ok) sample_index = sync_sample_index;
			need_clock_update_ = false;
		}
	}

	clock_recovery(filtered_sample);

	if (demodState != DemodState::UNLOCKED && correlator.index() == sample_index)
	{
		dev.sample(filtered_sample);
	}

	switch (demodState)
	{
	case DemodState::UNLOCKED:
		do_unlocked();
		break;
	case DemodState::FIRST_SYNC:
		do_first_sync();
		break;
	case DemodState::STREAM_SYNC:
		do_stream_sync();
		break;
	case DemodState::FRAME:
		do_frame(filtered_sample);
		break;
	}

	if (count_ % (OPV_FRAME_SYMBOLS_TOTAL * 5) == 0)
	{
		update_dcd();
		count_ = 0;
		if (diagnostic_callback)
		{
			diagnostic_callback(int(dcd_), dev.error(), dev.deviation(), dev.offset(), (demodState != DemodState::UNLOCKED),
				clock_recovery.clock_estimate(), sample_index, sync_sample_index, clock_recovery.sample_index(), viterbi_cost);
		}
		dcd.update();
	}
}

} // mobilinkd
