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

#include "Ws2812TimerBurst.hpp"

#include <errno.h>
#include <inttypes.h>
#include <string.h>

#include <px4_platform_common/log.h>
#include <px4_platform_common/micro_hal.h>
#include <px4_arch/dshot.h>	// px4_stm32_dmasetup()
#include <px4_arch/io_timer.h>
#include <nuttx/cache.h>
#include <stm32_tim.h>
#include <uORB/uORB.h>
#include <uORB/topics/px4io_status.h>

#if !defined(CONFIG_ARCH_CHIP_STM32H7)
#  error "motor_leds: only STM32H7 is supported"
#endif

// WS2812B timing. Datasheets: T0H 0.25-0.55 us (WS2812B), 0.22-0.38 us (WS2812B-V5);
// T1H 0.65-0.95 us, 0.58-1.0 us; reset > 50 us, > 280 us. The low times follow from the period.
static constexpr uint32_t PERIOD_NS = 1250;			// 800 kHz
static constexpr uint32_t T0H_NS = 317;
static constexpr uint32_t T1H_NS = 750;
static constexpr hrt_abstime RESET_TIME = 300;			// us, low between frames (at least)
static constexpr hrt_abstime FRAME_TIMEOUT = 5000;		// us, a frame takes 33 us (1 LED) to 243 us (8 LEDs)

// as DShot: 32 bit memory and peripheral (DMAR), memory increment, high priority (SPI and px4io use low)
static constexpr uint32_t DMA_SCR = DMA_SCR_PRIHI | DMA_SCR_MSIZE_32BITS | DMA_SCR_PSIZE_32BITS | DMA_SCR_MINC |
				    DMA_SCR_DIR_M2P | DMA_SCR_TCIE | DMA_SCR_TEIE | DMA_SCR_DMEIE;

// Statically allocated: .bss is in AXI SRAM, which DMA1/DMA2 can read (the heap includes the DTCM, which they cannot).
// Cache line aligned, cleaned before each frame.
static uint32_t g_motor_leds_dma[Ws2812TimerBurst::MAX_OUTPUTS * ws2812::slots(Ws2812TimerBurst::MAX_LEDS)]
px4_cache_aligned_data() = {};

static uint32_t ns_to_ticks(uint32_t clock_khz, uint32_t ns)
{
	return (clock_khz * ns + 500000) / 1000000;
}

int Ws2812TimerBurst::init(unsigned timer, unsigned leds)
{
	if (_dma != nullptr) {
		return -EBUSY;
	}

	if ((timer >= MAX_IO_TIMERS) || (leds < 1) || (leds > MAX_LEDS)) {
		return -EINVAL;
	}

	const io_timers_t &tmr = io_timers[timer];
	const io_timers_channel_mapping_element_t &mapping = io_timers_channel_mapping.element[timer];

	// general purpose timer with DMA burst (DCR/DMAR), a DMA controller assigned by the board and channels
	if (((tmr.base != STM32_TIM2_BASE) && (tmr.base != STM32_TIM3_BASE) && (tmr.base != STM32_TIM4_BASE)
	     && (tmr.base != STM32_TIM5_BASE))
	    || (tmr.dshot.dma_map_up == 0) || (mapping.channel_count == 0) || (mapping.channel_count > MAX_OUTPUTS)) {
		return -ENODEV;
	}

	// bit timing in timer ticks, prescaler 1
	const uint32_t clock_khz = tmr.clock_freq / 1000;
	const uint32_t period = ns_to_ticks(clock_khz, PERIOD_NS);
	_t0h = ns_to_ticks(clock_khz, T0H_NS);
	_t1h = ns_to_ticks(clock_khz, T1H_NS);

	if ((period > 0x10000) || (_t0h == 0) || (_t1h >= period)) {
		return -EINVAL;
	}

	_arr = period - 1;

	// Channel layout. The burst covers CCRx of the lowest to the highest channel used. The output
	// order is the board's (e.g. fmu-v6x: AUX1 = TIM5_CH4 ... AUX4 = TIM5_CH1), never assumed.
	unsigned lowest = 3;
	unsigned highest = 0;

	for (unsigned k = 0; k < mapping.channel_count; k++) {
		const unsigned channel = timer_io_channels[mapping.first_channel_index + k].timer_channel;
		lowest = (channel < lowest) ? channel : lowest;
		highest = (channel > highest) ? channel : highest;
	}

	_ccmr1 = 0;
	_ccmr2 = 0;
	_ccer = 0;

	for (unsigned k = 0; k < mapping.channel_count; k++) {
		const unsigned channel = timer_io_channels[mapping.first_channel_index + k].timer_channel;
		const uint32_t output_compare = (GTIM_CCMR_MODE_PWM1 << GTIM_CCMR1_OC1M_SHIFT) | GTIM_CCMR1_OC1PE; // high while CNT < CCR

		if (channel < 2) {
			_ccmr1 |= output_compare << (8 * channel);

		} else {
			_ccmr2 |= output_compare << (8 * (channel - 2));
		}

		_ccer |= GTIM_CCER_CC1E << (4 * channel); // active high
		_word_of[k] = channel - lowest;
	}

	_words = highest - lowest + 1;
	_dcr = ((STM32_GTIM_CCR1_OFFSET / 4) + lowest) | ((_words - 1) << GTIM_DCR_DBL_SHIFT);
	_ndata = _words * ws2812::slots(leds);
	_leds = leds;
	_timer = timer;
	_base = tmr.base;
	_first_channel = mapping.first_channel_index;
	_outputs = mapping.channel_count;

	// Reserve the channels and the timer: pwm_out and dshot started later get -EBUSY and leave them alone
	for (; _reserved_channels < _outputs; _reserved_channels++) {
		if (io_timer_allocate_channel(_first_channel + _reserved_channels, IOTimerChanMode_LED) != 0) {
			deinit();
			return -EBUSY;
		}
	}

	if (io_timer_allocate_timer(timer, IOTimerChanMode_LED) != 0) {
		deinit();
		return -EBUSY;
	}

	_timer_reserved = true;

	DMA_HANDLE dma = stm32_dmachannel(tmr.dshot.dma_map_up);

	if (dma == nullptr) {
		deinit();
		return -ENOSR;
	}

#if defined(PX4IO_SERIAL_TX_DMAMAP) && defined(PX4IO_SERIAL_RX_DMAMAP)

	// px4io starts after this (rc.additional_init) and does not run without its two DMA streams,
	// which would leave the MAIN outputs (motors) dead: make sure they are still available.
	if (orb_exists(ORB_ID(px4io_status), 0) != PX4_OK) {
		DMA_HANDLE tx = stm32_dmachannel(PX4IO_SERIAL_TX_DMAMAP);
		DMA_HANDLE rx = stm32_dmachannel(PX4IO_SERIAL_RX_DMAMAP);

		if (tx != nullptr) {
			stm32_dmafree(tx);
		}

		if (rx != nullptr) {
			stm32_dmafree(rx);
		}

		if ((tx == nullptr) || (rx == nullptr)) {
			stm32_dmafree(dma);
			deinit();
			return -ENOSR;
		}
	}

#endif

	memset(g_motor_leds_dma, 0, sizeof(g_motor_leds_dma));
	up_clean_dcache((uintptr_t)g_motor_leds_dma, (uintptr_t)g_motor_leds_dma + sizeof(g_motor_leds_dma));

	// Program the timer (io_timer_init_timer() is not used: it is reserved for PWM and attaches the IRQ)
	modifyreg32(tmr.clock_register, 0, tmr.clock_bit);
	reg(STM32_GTIM_CR1_OFFSET) = 0;
	reg(STM32_GTIM_CR2_OFFSET) = 0;
	reg(STM32_GTIM_SMCR_OFFSET) = 0;
	reg(STM32_GTIM_DIER_OFFSET) = 0;
	reg(STM32_GTIM_CCER_OFFSET) = 0;
	reg(STM32_GTIM_CCMR1_OFFSET) = _ccmr1;
	reg(STM32_GTIM_CCMR2_OFFSET) = _ccmr2;
	clear_compare();
	reg(STM32_GTIM_PSC_OFFSET) = 0;
	reg(STM32_GTIM_ARR_OFFSET) = _arr;
	reg(STM32_GTIM_DCR_OFFSET) = _dcr;
	reg(STM32_GTIM_CR1_OFFSET) = GTIM_CR1_ARPE | GTIM_CR1_URS;	// URS: only counter overflows request DMA, not UG
	reg(STM32_GTIM_EGR_OFFSET) = GTIM_EGR_UG;			// load prescaler, period and compare values (0)
	reg(STM32_GTIM_SR_OFFSET) = 0;
	reg(STM32_GTIM_CCER_OFFSET) = _ccer;
	reg(STM32_GTIM_CR1_OFFSET) = GTIM_CR1_ARPE | GTIM_CR1_URS | GTIM_CR1_CEN; // always running: compare 0, lines low

	// The pins are low GPIOs since board init: handing them to the timer (low) gives no glitch
	for (unsigned k = 0; k < _outputs; k++) {
		px4_arch_configgpio(timer_io_channels[_first_channel + k].gpio_out);
	}

	_frames_perf = perf_alloc(PC_COUNT, MODULE_NAME": frames");
	_dma_error_perf = perf_alloc(PC_COUNT, MODULE_NAME": dma error");
	_timeout_perf = perf_alloc(PC_COUNT, MODULE_NAME": timeout");
	_hijack_perf = perf_alloc(PC_COUNT, MODULE_NAME": timer changed");

	_t_end = 0;
	_dma = dma;
	return PX4_OK;
}

void Ws2812TimerBurst::deinit()
{
	if (_dma != nullptr) {
		stm32_dmastop(_dma);
		reg(STM32_GTIM_DIER_OFFSET) = 0;
		clear_compare();
		reg(STM32_GTIM_EGR_OFFSET) = GTIM_EGR_UG; // lines low now

		// back to low GPIOs, as set at boot, then stop the timer
		for (unsigned k = 0; k < _outputs; k++) {
			px4_arch_configgpio(io_timer_channel_get_gpio_output(_first_channel + k));
		}

		reg(STM32_GTIM_CR1_OFFSET) = 0;
		reg(STM32_GTIM_CCER_OFFSET) = 0;

		stm32_dmafree(_dma);
		_dma = nullptr;
		_busy.store(false);
	}

	for (; _reserved_channels > 0; _reserved_channels--) {
		io_timer_unallocate_channel(_first_channel + _reserved_channels - 1);
	}

	if (_timer_reserved) {
		io_timer_unallocate_timer(_timer);
		_timer_reserved = false;
	}

	perf_free(_frames_perf);
	perf_free(_dma_error_perf);
	perf_free(_timeout_perf);
	perf_free(_hijack_perf);
	_frames_perf = nullptr;
	_dma_error_perf = nullptr;
	_timeout_perf = nullptr;
	_hijack_perf = nullptr;
}

int Ws2812TimerBurst::send(const ws2812::Rgb rgb[MAX_OUTPUTS])
{
	if (_dma == nullptr) {
		return -ENODEV;
	}

	const hrt_abstime now = hrt_absolute_time();

	if (_busy.load()) {
		if (now - _t_start > FRAME_TIMEOUT) {
			abort_frame();
		}

		return -EBUSY;
	}

	if (now - _t_end < RESET_TIME) {
		return -EAGAIN;
	}

	if (!registers_ok()) {
		perf_count(_hijack_perf);
		return -EIO;
	}

	ws2812::fill(g_motor_leds_dma, _words, _leds, _outputs, _word_of, rgb, _t0h, _t1h);
	up_clean_dcache((uintptr_t)g_motor_leds_dma, (uintptr_t)&g_motor_leds_dma[_ndata]);

	// The active compare values are 0 (tail of the previous frame), so stopping the counter keeps the
	// lines low. UG restarts the bit period with compare 0 and, with URS set, requests no DMA.
	// The CPU never writes data to CCRx: the first bit cannot be sent twice.
	reg(STM32_GTIM_CR1_OFFSET) &= ~GTIM_CR1_CEN;
	reg(STM32_GTIM_DIER_OFFSET) &= ~GTIM_DIER_UDE;
	clear_compare();
	reg(STM32_GTIM_EGR_OFFSET) = GTIM_EGR_UG;

	px4_stm32_dmasetup(_dma, _base + STM32_GTIM_DMAR_OFFSET, (uint32_t)(uintptr_t)g_motor_leds_dma, _ndata, DMA_SCR);

	_t_start = now;
	_busy.store(true);
	stm32_dmastart(_dma, dma_callback, this, false);

	// From now on each update event loads the next slot into the preload registers; it becomes active
	// at the following update event, so every slot lasts exactly one bit period. A request left pending
	// by the previous frame (if any) is served here, while the counter is stopped: the frame then starts
	// one period earlier, still after at least one low period.
	reg(STM32_GTIM_DIER_OFFSET) |= GTIM_DIER_UDE;
	reg(STM32_GTIM_CR1_OFFSET) |= GTIM_CR1_CEN;

	perf_count(_frames_perf);
	return PX4_OK;
}

void Ws2812TimerBurst::dma_callback(DMA_HANDLE handle, uint8_t status, void *arg)
{
	Ws2812TimerBurst *self = static_cast<Ws2812TimerBurst *>(arg);

	// The two tail slots (0) are now in the shadow and preload registers: the lines stay low.
	// The counter is not stopped, so the last bit is never cut short.
	self->reg(STM32_GTIM_DIER_OFFSET) &= ~GTIM_DIER_UDE;

	if (status & DMA_STATUS_ERROR) {
		// stopped somewhere in the frame: low from the next update event on
		self->clear_compare();
		perf_count(self->_dma_error_perf);
	}

	self->_t_end = hrt_absolute_time();
	self->_busy.store(false);
}

void Ws2812TimerBurst::abort_frame()
{
	const irqstate_t flags = px4_enter_critical_section();

	if (_busy.load()) {
		stm32_dmastop(_dma);
		reg(STM32_GTIM_DIER_OFFSET) &= ~GTIM_DIER_UDE;
		reg(STM32_GTIM_CR1_OFFSET) &= ~GTIM_CR1_CEN;
		clear_compare();
		reg(STM32_GTIM_EGR_OFFSET) = GTIM_EGR_UG; // lines low now
		reg(STM32_GTIM_CR1_OFFSET) |= GTIM_CR1_CEN;
		_t_end = hrt_absolute_time();
		_busy.store(false);
		perf_count(_timeout_perf);
	}

	px4_leave_critical_section(flags);
}

void Ws2812TimerBurst::clear_compare()
{
	for (unsigned k = 0; k < _outputs; k++) {
		reg(timer_io_channels[_first_channel + k].ccr_offset) = 0;
	}
}

bool Ws2812TimerBurst::registers_ok() const
{
	return (reg(STM32_GTIM_CR1_OFFSET) == (GTIM_CR1_ARPE | GTIM_CR1_URS | GTIM_CR1_CEN))
	       && (reg(STM32_GTIM_PSC_OFFSET) == 0)
	       && (reg(STM32_GTIM_ARR_OFFSET) == _arr)
	       && (reg(STM32_GTIM_DCR_OFFSET) == _dcr)
	       && (reg(STM32_GTIM_CCMR1_OFFSET) == _ccmr1)
	       && (reg(STM32_GTIM_CCMR2_OFFSET) == _ccmr2)
	       && (reg(STM32_GTIM_CCER_OFFSET) == _ccer);
}

void Ws2812TimerBurst::print_status() const
{
	if (_dma == nullptr) {
		PX4_INFO("not running");
		return;
	}

	// stream serving TIMx_UP: DMAMUX1 channels 0-7 are DMA1 streams 0-7, 8-15 DMA2 streams 0-7 (DMA0: not found)
	const uint32_t request = DMAMAP_REQUEST(io_timers[_timer].dshot.dma_map_up);
	unsigned c = 0;

	while ((c < 16) && ((getreg32(STM32_DMAMUX1_BASE + STM32_DMAMUX_CXCR_OFFSET(c)) & DMAMUX_CCR_DMAREQID_MASK) != request)) {
		c++;
	}

	PX4_INFO("TIM%u (io_timer %u), outputs %u-%u, %u LED(s) each, DMA%u stream %u (request %" PRIu32 ")",
		 2 + (unsigned)((_base - STM32_TIM2_BASE) / 0x400), _timer, _first_channel + 1, _first_channel + _outputs, _leds,
		 (c < 16) ? c / 8 + 1 : 0, c % 8, request);
	PX4_INFO("clock %" PRIu32 " Hz, bit %" PRIu32 " ticks, T0H %" PRIu32 ", T1H %" PRIu32 ", DCR 0x%04" PRIx32
		 ", %u transfers/frame",
		 io_timers[_timer].clock_freq, _arr + 1, _t0h, _t1h, _dcr, _ndata);
	perf_print_counter(_frames_perf);
	perf_print_counter(_dma_error_perf);
	perf_print_counter(_timeout_perf);
	perf_print_counter(_hijack_perf);
}
