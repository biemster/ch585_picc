#include "ch32fun.h"
#include "CH58x_NFCA_LIB.h"
#include <stdio.h>

#define PIN_LED              PA8
#define PICC_DATA_BUF_LEN    20
#define PICC_SIGNAL_BUF_LEN  512
#define NFCA_PICC_RSP_POLAR  0

#define RB_TMR_FREQ_13_56    0x20
#define R8_NFC_CMD           (*(vu8*)0x4000E000)
#define R32_NFC_DRV          (*(vu32*)0x4000E014)


static uint32_t gs_picc_signal_buf[PICC_SIGNAL_BUF_LEN];
__attribute__((aligned(4))) uint8_t g_picc_data_buf[PICC_DATA_BUF_LEN];
__attribute__((aligned(4))) uint8_t g_picc_parity_buf[PICC_DATA_BUF_LEN];


typedef enum {
	High_Level = 0,
	Low_Level,
} PWMX_PolarTypeDef;

typedef enum {
	PWM_Times_1 = 0,
	PWM_Times_4,
	PWM_Times_8,
	PWM_Times_16,
} PWM_RepeatTsTypeDef;

typedef enum {
	CAP_NULL = 0,
	Edge_To_Edge,
	FallEdge_To_FallEdge,
	RiseEdge_To_RiseEdge,
} CapModeTypeDef;

const uint8_t byteParityBitsTable[256] = {
	1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
	0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
	0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
	1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
	0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
	1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
	1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
	0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
	0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
	1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
	1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
	0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
	1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
	0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
	0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
	1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1
};


__INTERRUPT
__HIGH_CODE
void TMR0_IRQHandler(void) {
    nfca_picc_rx_irq_handler();
}

__INTERRUPT
__HIGH_CODE
void TMR3_IRQHandler(void) {
	R8_TMR3_INT_FLAG = R8_TMR3_INT_FLAG;
	R8_TMR3_CTRL_DMA = 0;
	R32_TMR3_DMA_END = R32_TMR3_DMA_NOW + 0x100;
	
	R32_TMR0_CNT_END = 0x120;
	R8_TMR0_CTRL_DMA = RB_TMR_DMA_LOOP | RB_TMR_DMA_ENABLE;
	R8_TMR0_CTRL_MOD = 0xe5;

	NVIC_ClearPendingIRQ(TMR3_IRQn);
	return;
}

void nfca_picc_init() {
	funPinMode( PA7, GPIO_CFGLR_IN_FLOAT ); // NFC CTR
	R32_PIN_IN_DIS |= PA7; // disable PA7 digital input

	funPinMode( (PB8 | PB9 | PB16 | PB17), GPIO_CFGLR_IN_FLOAT );
	R32_PIN_IN_DIS |= (((PB8 | PB9) & ~PB) << 16); // disable PB8 PB9 digital input
	R16_PIN_CONFIG |= (((PB16 | PB17) & ~PB) >> 8); // pin alt func

	nfca_picc_config_t cfg;
	cfg.signal_buf = gs_picc_signal_buf;
	cfg.signal_buf_len = PICC_SIGNAL_BUF_LEN;

	cfg.parity_buf = g_picc_parity_buf;
	cfg.data_buf = g_picc_data_buf;
	cfg.data_buf_len = PICC_DATA_BUF_LEN;
	
	uint8_t res = nfca_picc_lib_init(&cfg);
	if(res) {
		printf("nfca picc lib init error\n");
		while(1);
	}
}

void nfca_picc_start() {
	R8_NFC_CMD = 0x48;
	R32_NFC_DRV = (R32_NFC_DRV & 0xf9cf) | 0x620;

	R8_TMR3_CTRL_MOD = RB_TMR_ALL_CLEAR;

#if NFCA_PICC_RSP_POLAR != 0
	R8_TMR3_CTRL_MOD = (High_Level << 4) | (PWM_Times_4 << 6) | RB_TMR_FREQ_13_56;
#else
	R8_TMR3_CTRL_MOD = (Low_Level << 4) | (PWM_Times_4 << 6) | RB_TMR_FREQ_13_56;
#endif

	R32_TMR3_CNT_END = TMR3_NFCA_PICC_CNT_END;
	R32_TMR3_FIFO = 0;
	R32_TMR3_DMA_END = R32_TMR3_DMA_NOW + 0x100;
	R8_TMR3_INT_FLAG = RB_TMR_IF_DMA_END;
	R8_TMR3_INTER_EN = RB_TMR_IE_DMA_END;

#if NFCA_PICC_RSP_POLAR != 0
	R8_TMR3_CTRL_MOD = ((High_Level << 4) | (PWM_Times_4 << 6) | RB_TMR_FREQ_13_56 | RB_TMR_COUNT_EN);
#else
	R8_TMR3_CTRL_MOD = ((Low_Level << 4) | (PWM_Times_4 << 6) | RB_TMR_FREQ_13_56 | RB_TMR_COUNT_EN);
#endif

	R8_TMR0_CTRL_MOD = RB_TMR_ALL_CLEAR;
	R8_TMR0_CTRL_MOD = RB_TMR_MODE_IN | (RiseEdge_To_RiseEdge << 6) | RB_TMR_FREQ_13_56;
	R32_TMR0_CNT_END = TMR0_NFCA_PICC_CNT_END;

	R32_TMR0_DMA_END = R32_TMR0_DMA_NOW + 0x100;
	R8_TMR0_INT_FLAG = RB_TMR_IF_DMA_END;

	R32_TMR0_DMA_BEG = (uint32_t)gs_picc_signal_buf;
	R32_TMR0_DMA_END = (uint32_t)&(gs_picc_signal_buf[PICC_SIGNAL_BUF_LEN]);
	R8_TMR0_CTRL_DMA = RB_TMR_DMA_LOOP | RB_TMR_DMA_ENABLE;

	R8_TMR0_INT_FLAG = RB_TMR_IF_DATA_ACT;
	R8_TMR0_INTER_EN = RB_TMR_IE_DATA_ACT;

	R8_TMR0_CTRL_MOD = (RB_TMR_MODE_IN | (RiseEdge_To_RiseEdge << 6) | RB_TMR_FREQ_13_56 | RB_TMR_COUNT_EN);

	NVIC_EnableIRQ(TMR3_IRQn);
	NVIC_EnableIRQ(TMR0_IRQn);
}

__HIGH_CODE
void ISO14443ACalOddParityBit(uint8_t *data, uint8_t *out_parity, uint16_t len) {
    for (int i = 0; i < len; i++) {
        out_parity[i] = byteParityBitsTable[data[i]];
    }
}

__HIGH_CODE
static uint16_t nfca_picc_data_handler(uint16_t bits_num) {
	uint16_t send_bits = 0;

	if((bits_num == 7) && (((g_picc_data_buf[0] == 0x26)) || (g_picc_data_buf[0] == 0x52))) {
		g_picc_data_buf[0] = 0x44;
		g_picc_data_buf[1] = 0x00;
		send_bits = 16;
		ISO14443ACalOddParityBit(g_picc_data_buf, g_picc_parity_buf, 2);
	}

	return send_bits;
}

static void nfca_picc_online(void) {}
static void nfca_picc_offline(void) {}

static nfca_picc_cb_t gs_nfca_picc_cb = {
	.online = nfca_picc_online,
	.data_handler = nfca_picc_data_handler,
	.offline = nfca_picc_offline,
};

int main()
{
	SystemInit();

	funGpioInitAll(); // no-op on ch5xx

	funPinMode( PIN_LED, GPIO_CFGLR_OUT_2Mhz_PP ); // Set PIN_LED to output

	nfca_picc_init();
	nfca_picc_register_callback(&gs_nfca_picc_cb);
	nfca_picc_start();

	while(1) {
		funDigitalWrite( PIN_LED, FUN_LOW );  // Turn on LED
		Delay_Ms( 250 );
		funDigitalWrite( PIN_LED, FUN_HIGH ); // Turn off LED
		Delay_Ms( 250 );
	}
}
