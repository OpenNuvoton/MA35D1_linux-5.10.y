// SPDX-License-Identifier: GPL-2.0+
/*
 * linux/drivers/gpu/drm/gpu/drm/ma35d1/ma35d1_regs.h
 *
 * Copyright (c) 2024 Nuvoton technology corporation.
 * Copyright (c) 2024 Nuvoton (Shanghai) corporation.
 *
 * Author: Jiang Tianwen <twjiang@nuvoton.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#define REG_AQHiClockControl					0x0000
#define REG_AQHIIdle						0x0004
#define REG_AQxiConfig						0x00A4
#define REG_GCProductId						0x00A8
#define REG_AQxiStatus						0x000C
#define REG_AQIntrAcknowledge					0x0010
#define REG_AQIntrEnbl						0x0014
#define REG_GCChipId						0x0020
#define REG_GCChipRev						0x0024
#define REG_GCChipDate						0x0028
#define REG_GCChipTime						0x002C
#define REG_GCCustomerId					0x0030
#define REG_gcTotalReads					0x0040
#define REG_gcTotalCycles					0x0078
#define REG_gcregHICHIPatchRev					0x0098
#define REG_GCECOId						0x00E8
#define REG_dcregFrameBufferAddress0				0x1400
#define REG_dcregFrameBufferStride0				0x1408
#define REG_dcregDisplayDitherConfig0				0x1410
#define REG_dcregPanelConfig0					0x1418
#define REG_dcregDisplayDitherTableLow0				0x1420
#define REG_dcregDisplayDitherTableHigh0			0x1428
#define REG_dcregHDisplay0					0x1430
#define REG_dcregHSync0						0x1438
#define REG_dcregVDisplay0					0x1440
#define REG_dcregVSync0						0x1448
#define REG_dcregDisplayCurrentLocation0			0x1450
#define REG_dcregGammaIndex0					0x1458
#define REG_dcregGammaData0					0x1460
#define REG_dcregCursorConfig					0x1468
#define REG_dcregCursorAddress					0x146C
#define REG_dcregCursorLocation					0x1470
#define REG_dcregCursorBackground				0x1474
#define REG_dcregCursorForeground				0x1478
#define REG_dcregDisplayIntr					0x147C
#define REG_dcregDisplayIntrEnable				0x1480
#define REG_dcregCursorModuleClockGatingControl			0x1484
#define REG_dcregGeneralConfig0					0x148C
#define REG_dcregDpiConfig0					0x14B8
#define REG_dcregDebugCounterSelect0				0x14D0
#define REG_dcregDebugCounterValue0				0x14D8
#define REG_dcregFrameBufferColorKey0				0x1508
#define REG_dcregFrameBufferColorKeyHigh0			0x1510
#define REG_dcregFrameBufferConfig0				0x1518
#define REG_dcregFrameBufferScaleConfig0			0x1520
#define REG_dcregFrameBufferBGColor0				0x1528
#define REG_dcregFrameBufferUPlanarAddress0			0x1530
#define REG_dcregFrameBufferVPlanarAddress0			0x1538
#define REG_dcregOverlayConfig0					0x1540
#define REG_dcregOverlayAlphaBlendConfig0			0x1580
#define REG_dcregOverlayAddress0				0x15C0
#define REG_dcregOverlayStride0					0x1600
#define REG_dcregOverlayTL0					0x1640
#define REG_dcregOverlayBR0					0x1680
#define REG_dcregOverlaySrcGlobalColor0				0x16C0
#define REG_dcregOverlayDstGlobalColor0				0x1700
#define REG_dcregOverlayColorKey0				0x1740
#define REG_dcregOverlayColorKeyHigh0				0x1780
#define REG_dcregOverlaySize0					0x17C0
#define REG_dcregFrameBufferUStride0				0x1800
#define REG_dcregFrameBufferVStride0				0x1808
#define REG_dcregFrameBufferSize0				0x1810
#define REG_dcregIndexColorTableIndex0				0x1818
#define REG_dcregIndexColorTableData0				0x1820
#define REG_dcregFrameBufferScaleFactorX0			0x1828
#define REG_dcregFrameBufferScaleFactorY0			0x1830
#define REG_dcregHoriFilterKernelIndex0				0x1838
#define REG_dcregOverlayUPlanarAddress0				0x1840
#define REG_dcregOverlayVPlanarAddress0				0x1880
#define REG_dcregOverlayUStride0				0x18C0
#define REG_dcregOverlayVStride0				0x1900
#define REG_dcregOverlayClearValue0				0x1940
#define REG_dcregOverlayIndexColorTableIndex0			0x1980
#define REG_dcregOverlayIndexColorTableData0			0x19C0
#define REG_dcregHoriFilterKernel0				0x1A00
#define REG_dcregVeriFilterKernelIndex0				0x1A08
#define REG_dcregVeriFilterKernel0				0x1A10
#define REG_dcregFrameBufferClearValue0				0x1A18
#define REG_dcregFrameBufferInitialOffset0			0x1A20
#define REG_dcregModuleClockGatingControl0			0x1A28
#define REG_dcregLatencyCounter0				0x1A30
#define REG_dcregQos0						0x1A38
#define REG_dcregMpuIntfCmd0					0x1C40
#define REG_dcregMpuIntfReadPara00				0x1C48
#define REG_dcregMpuIntfReadPara10				0x1C50
#define REG_dcregMpuIntfReadPara20				0x1C58
#define REG_dcregMpuIntfReadPara30				0x1C60
#define REG_dcregMpuIntfReadPara40				0x1C68
#define REG_dcregMpuIntfReadStatus0				0x1C70
#define REG_dcregMpuIntfConfig0					0x1C78
#define REG_dcregMpuIntfFrame0					0x1C80
#define REG_dcregMpuIntfACWrI800				0x1C88
#define REG_dcregMpuIntfACRdI800				0x1C90
#define REG_dcregMpuIntfACWrM680				0x1C98
#define REG_dcregMpuIntfACRdM680				0x1CA0
#define REG_dcregMpuIntfACVsyncCSX0				0x1CA8
#define REG_dcregFrameBufferROIOrigin0				0x1CB0
#define REG_dcregFrameBufferROISize0				0x1CB8
#define REG_dcregFrameBufferConfigEx0				0x1CC0
#define REG_dcregOverlayROIOrigin0				0x1D04
#define REG_dcregOverlayROISize0				0x1D40

#define OUTPUT_DISABLE						(0x0 << 0)
#define OUTPUT_ENABLE						(0x1 << 0)
#define GAMMA_DISABLE						(0x0 << 2)
#define GAMMA_ENABLE						(0x1 << 2)
#define RESET_DISABLE						(0x0 << 4)
#define RESET_ENABLE						(0x1 << 4)
#define PAGE_FLIP						(0x1 << 6)
#define SHADOW_LOAD						(0x1 << 8)
#define YUV_709_BT709						(0x1 << 14)
#define YUV_2020_BT2020						(0x3 << 14)
#define FRAMEBUFFER_FORMAT_POS					(26)

#define HSYNC_TOTAL_POS						16
#define VSYNC_TOTAL_POS						16
#define HSYNC_PULSE_END_POS					15
#define VSYNC_PULSE_END_POS					15
#define FRAMEBUFFER_HEIGHT_POS					15

#define HSYNC_PULSE_POLARITY_POS				(0x0 << 31)
#define HSYNC_PULSE_ENABLE					(0x1 << 30)
#define VSYNC_PULSE_POLARITY_POS				(0x0 << 31)
#define VSYNC_PULSE_ENABLE					(0x1 << 30)

#define PANELCONFIG_CLOCK_POLARITY_POS				(0x0 << 9)
#define PANELCONFIG_CLOCK_POLARITY_NEG				(0x1 << 9)
#define PANELCONFIG_CLOCK_DISABLE				(0x0 << 8)
#define PANELCONFIG_CLOCK_ENABLE				(0x1 << 8)
#define PANELCONFIG_DATA_POLARITY_POS				(0x0 << 5)
#define PANELCONFIG_DATA_POLARITY_NEG				(0x1 << 5)
#define PANELCONFIG_DATA_DISABLE				(0x0 << 4)
#define PANELCONFIG_DATA_ENABLE					(0x1 << 4)
#define PANELCONFIG_DE_POLARITY_POS				(0x0 << 1)
#define PANELCONFIG_DE_POLARITY_NEG				(0x1 << 1)
#define PANELCONFIG_DE_DATA_DISABLE				(0x0 << 0)
#define PANELCONFIG_DE_DATA_ENABLE				(0x1 << 0)
