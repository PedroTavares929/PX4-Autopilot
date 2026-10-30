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
 * @file ws2812_frame.hpp
 *
 * WS2812B frame encoding for a timer DMA burst (no platform dependencies, testable on the host).
 *
 * A frame is a sequence of slots, one per bit period. Each slot holds one compare value
 * (high time in timer ticks) per timer channel, in register order starting at the DMA burst
 * base register. The frame ends with TAIL_SLOTS slots of 0, which leave the lines low.
 */

#pragma once

#include <stdint.h>

namespace ws2812
{

struct Rgb {
	uint8_t r;
	uint8_t g;
	uint8_t b;
};

static constexpr unsigned BITS_PER_LED = 24;
static constexpr unsigned TAIL_SLOTS = 2;

/** Number of slots of a frame with @p leds LEDs on each output, including the trailing 0 slots. */
constexpr unsigned slots(unsigned leds) { return BITS_PER_LED * leds + TAIL_SLOTS; }

/**
 * Write the data words of a frame: GRB, most significant bit first, the same colour on all
 * @p leds LEDs of an output. Only the words of the outputs are written: words of unused
 * channels and the tail slots are left as they are (0).
 *
 * @param buf            frame buffer, slots(leds) * words_per_slot words
 * @param words_per_slot compare registers written per slot (DMA burst length)
 * @param word_of        word index in the slot of each output (timer channel - burst base)
 * @param rgb            colour of each output
 * @param t0h            compare value of a "0" bit
 * @param t1h            compare value of a "1" bit
 */
inline void fill(uint32_t *buf, unsigned words_per_slot, unsigned leds, unsigned outputs,
		 const uint8_t word_of[], const Rgb rgb[], uint32_t t0h, uint32_t t1h)
{
	for (unsigned k = 0; k < outputs; k++) {
		const uint32_t grb = (uint32_t(rgb[k].g) << 16) | (uint32_t(rgb[k].r) << 8) | rgb[k].b;
		uint32_t *word = &buf[word_of[k]];

		for (unsigned led = 0; led < leds; led++) {
			for (unsigned bit = 0; bit < BITS_PER_LED; bit++) {
				*word = (grb & (0x800000u >> bit)) ? t1h : t0h;
				word += words_per_slot;
			}
		}
	}
}

} // namespace ws2812
