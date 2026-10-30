/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file Ws2812TimerBurst.hpp
 *
 * WS2812B output on all channels of one io_timer (STM32H7, TIM2..TIM5).
 *
 * The timer runs continuously in PWM mode 1 with one bit period (1.25 us) and preloaded
 * compare registers. During a frame, a DMA burst on TIMx_UP (DCR/DMAR) writes the next
 * compare value of every channel at each update event, so all outputs are sent at the same
 * time, each with its own colour, using one DMA stream and one interrupt per frame.
 * The frame ends with compare values of 0 and the counter is never stopped in the middle of
 * a bit, so between frames the lines are driven low (WS2812 reset/latch).
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/atomic.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <stm32_dma.h>

#include "ws2812_frame.hpp"

class Ws2812TimerBurst
{
public:
	static constexpr unsigned MAX_OUTPUTS = 4;	///< channels of one timer
	static constexpr unsigned MAX_LEDS = 8;		///< LEDs per output, all with the colour of the output

	Ws2812TimerBurst() = default;
	~Ws2812TimerBurst() { deinit(); }

	Ws2812TimerBurst(const Ws2812TimerBurst &) = delete;
	Ws2812TimerBurst &operator=(const Ws2812TimerBurst &) = delete;

	/**
	 * Take over all channels of an io_timer: reserve the channels and the timer (LED mode),
	 * get a DMA stream for TIMx_UP, program the timer and switch the pins to it (driven low).
	 * Everything is released again on failure.
	 * @param timer io_timer index
	 * @param leds LEDs on each output
	 * @return 0 or -errno
	 */
	int init(unsigned timer, unsigned leds);

	/** Stop, drive the pins low as GPIOs and release the timer, the channels and the DMA stream. */
	void deinit();

	/**
	 * Start sending one frame. Does not wait for it (one interrupt at the end).
	 * @param rgb colour of each output, in output channel order
	 * @return 0, -EBUSY (frame in progress), -EAGAIN (reset time after the last frame not over),
	 *         -EIO (timer registers changed by someone else) or -ENODEV (not initialized)
	 */
	int send(const ws2812::Rgb rgb[MAX_OUTPUTS]);

	bool busy() const { return _busy.load(); }
	unsigned outputs() const { return _outputs; }
	unsigned first_channel() const { return _first_channel; }

	void print_status() const;

private:
	static void dma_callback(DMA_HANDLE handle, uint8_t status, void *arg);

	volatile uint32_t &reg(uint32_t offset) const { return *reinterpret_cast<volatile uint32_t *>(_base + offset); }

	void clear_compare();
	bool registers_ok() const;
	void abort_frame();

	DMA_HANDLE _dma{nullptr};
	uint32_t _base{0};			///< timer registers
	unsigned _timer{0};			///< io_timer index
	unsigned _first_channel{0};		///< first output channel (index in timer_io_channels)
	unsigned _outputs{0};
	unsigned _reserved_channels{0};
	bool _timer_reserved{false};
	unsigned _leds{1};
	unsigned _words{0};			///< compare registers written per update event (DMA burst length)
	unsigned _ndata{0};			///< DMA transfers per frame
	uint8_t _word_of[MAX_OUTPUTS] {};	///< position of each output in a burst
	uint32_t _arr{0};
	uint32_t _t0h{0};
	uint32_t _t1h{0};
	uint32_t _dcr{0};
	uint32_t _ccmr1{0};
	uint32_t _ccmr2{0};
	uint32_t _ccer{0};

	px4::atomic<bool> _busy{false};
	volatile hrt_abstime _t_start{0};
	volatile hrt_abstime _t_end{0};

	perf_counter_t _frames_perf{nullptr};
	perf_counter_t _dma_error_perf{nullptr};
	perf_counter_t _timeout_perf{nullptr};
	perf_counter_t _hijack_perf{nullptr};
};
