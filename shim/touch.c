/*
 * renoir touch bring-up probe (milestone T1), shim side, MMU off.
 *
 * Xiaomi Mi 11 Lite 5G (SM7350 "shima"): Goodix GT9897T ("Berlin" family,
 * Xiaomi driver gtx8_spi) on SPI at QUP wrapper0 SE2 = 0x988000, chip select
 * 0, mode 0, <= 6 MHz. Reset = TLMM GPIO30, IRQ = GPIO31 (not used: we poll).
 * Supplies: iovdd = PM8350C L2 (1.8 V), avdd = PM8350C L7 (3.3 V), voted via
 * RPMh (apps RSC DRV2). Everything below comes from renoir's live FDT and
 * Xiaomi's lisa/renoir kernel (drivers/input/touchscreen/gt9897t,
 * drivers/clk/qcom/gcc-shima.c, drivers/pinctrl/qcom/pinctrl-shima.c,
 * drivers/spi/spi-geni-qcom.c). The RPMh client follows ios-sweet's kernel
 * patch 0062.
 *
 * Writes only: two RPMh LDO votes, GCC clock votes for QUP0 SE2, TLMM config
 * of GPIO20-23 (SPI) and GPIO30 (reset), the SE's own registers, and the
 * touch IC. Nothing persistent.
 */
#include <stdint.h>
#include <stdbool.h>
#include "console.h"
#include "touch.h"

#define rsr(r) ({ uint64_t _v; __asm__ volatile("mrs %0, " r : "=r"(_v)); _v; })
static inline uint32_t rd(uintptr_t a) { uint32_t v = *(volatile uint32_t *)a; __asm__ volatile("dmb oshld" ::: "memory"); return v; }
static inline void wr(uintptr_t a, uint32_t v) { __asm__ volatile("dmb oshst" ::: "memory"); *(volatile uint32_t *)a = v; __asm__ volatile("dsb osh" ::: "memory"); }

static uint64_t ticks(void) { __asm__ volatile("isb" ::: "memory"); return rsr("cntpct_el0"); }
static void udelay(unsigned us) { uint64_t end = ticks() + rsr("cntfrq_el0") / 1000000 * us; while (ticks() < end) ; }
static void mdelay(unsigned ms) { udelay(ms * 1000); }

/* ---------------------------------------------------------------- RPMh */
#define RH_RSC		0x18220000UL	/* apps RSC, DRV2 (renoir FDT rsc@18200000, drv-id 2) */
#define RH_TCS		0x18220d00UL	/* qcom,tcs-offset 0xd00 */
#define RH_DB_FALLBACK	0x80860000UL	/* renoir reserved-memory qcom,cmd-db (live FDT) */
#define RH_DB_FALLBACK_SZ 0x20000U
#define RH_ACTIVE_N	2u		/* tcs-config: ACTIVE 2 first */
#define RH_TCS_STRIDE	0x2a0u
#define RH_CMD_STRIDE	0x14u
#define RH_IRQ_EN	0x00u
#define RH_IRQ_STATUS	0x04u
#define RH_IRQ_CLEAR	0x08u
#define RH_WAIT		0x10u
#define RH_CONTROL	0x14u
#define RH_STATUS	0x18u
#define RH_ENABLE	0x1cu
#define RH_MSGID	0x30u
#define RH_ADDR		0x34u
#define RH_DATA		0x38u
#define RH_CMD_STATUS	0x3cu
#define RH_AMC_EN	(1u << 16)
#define RH_TRIGGER	(1u << 24)
#define RH_ISSUED	(1u << 8)
#define RH_COMPLETE	(1u << 16)
#define RH_WRITE_ACK	(8u | (1u << 8) | (1u << 16))
#define RH_DB_MAGIC	0x0c0330dbu
#define RH_DB_HEADER	144u
#define RH_DB_ENTRY	24u
#define RH_VRM		4u

static uintptr_t rh_db;
static uint32_t rh_db_size;
static unsigned rh_ncpt;

static uint16_t db16(uint32_t off) { return (uint16_t)(rd(rh_db + (off & ~3u)) >> (8u * (off & 3u))); }

static bool rh_sync(uint32_t off, uint32_t v)
{
	wr(RH_TCS + off, v);
	for (unsigned i = 0; i < 1000; i++) {
		if (rd(RH_TCS + off) == v)
			return true;
		udelay(1);
	}
	return false;
}

static int rh_init(void)
{
	uint32_t id = rd(RH_RSC), config = rd(RH_RSC + 0x0c);
	unsigned major = (id >> 16) & 0xff, count = (config >> 12) & 0x3f;
	rh_ncpt = (config >> 27) & 0x1f;
	con_hex2("RPMH", "id=", id, "config=", config);
	if ((major != 1 && major != 2) || count < 8 || rh_ncpt < 3 || rh_ncpt > 16) {
		con_str("RPMH", "unsupported RSC revision/geometry");
		return -1;
	}
	/* SM8350-era DTs describe cmd-db directly as reserved memory; the old
	 * SDM845 locator at 0x0c3f000c is not part of renoir's FDT, so don't
	 * touch it (an unmapped secure read would be a synchronous abort). */
	uint32_t pa = RH_DB_FALLBACK, size = RH_DB_FALLBACK_SZ;
	rh_db = pa;
	rh_db_size = size;
	if (rd(rh_db + 4) != RH_DB_MAGIC) {
		con_hex2("RPMH", "cmd-db at=", pa, "bad magic=", rd(rh_db + 4));
		return -1;
	}
	con_hex2("RPMH", "cmd-db at=", pa, "size=", size);
	return 0;
}

static int rh_lookup(const char *res, uint32_t *address)
{
	uint32_t key[2] = {0, 0};
	for (unsigned i = 0; i < 8 && res[i]; i++)
		key[i / 4] |= (uint32_t)(uint8_t)res[i] << (8u * (i % 4));
	unsigned matches = 0;
	for (unsigned i = 0; i < 8; i++) {
		uint32_t h = 8u + 16u * i;
		unsigned slave = db16(h);
		if (!slave)
			break;
		uint32_t entries = RH_DB_HEADER + db16(h + 2);
		unsigned n = db16(h + 6);
		if (entries + n * RH_DB_ENTRY > rh_db_size)
			return -1;
		for (unsigned j = 0; j < n; j++) {
			uint32_t e = entries + RH_DB_ENTRY * j;
			if (rd(rh_db + e) != key[0] || rd(rh_db + e + 4) != key[1])
				continue;
			uint32_t a = rd(rh_db + e + 16);
			if (slave != RH_VRM || (a >> 16) != RH_VRM || (a & 15))
				return -1;
			*address = a;
			matches++;
		}
	}
	return matches == 1 ? 0 : -1;
}

static bool rh_free(unsigned t)
{
	uint32_t b = t * RH_TCS_STRIDE;
	return rd(RH_TCS + b + RH_STATUS) != 0 && rd(RH_TCS + b + RH_STATUS) != 0xffffffffu &&
	    rd(RH_TCS + b + RH_CONTROL) == 0 && rd(RH_TCS + b + RH_ENABLE) == 0 &&
	    rd(RH_TCS + b + RH_WAIT) == 0 && !(rd(RH_TCS + RH_IRQ_STATUS) & (1u << t));
}

/* One ACTIVE-set vote: voltage, mode HPM (7), enable. */
static int rh_vreg_on(const char *res, unsigned mv)
{
	uint32_t address = 0;
	if (rh_lookup(res, &address)) {
		con_str("RPMH", "cmd-db lookup failed");
		return -1;
	}
	int chosen = -1;
	for (unsigned t = 0; t < RH_ACTIVE_N && chosen < 0; t++)
		if (rh_free(t))
			chosen = (int)t;
	if (chosen < 0) {
		con_str("RPMH", "no idle ACTIVE TCS");
		return -1;
	}
	unsigned t = (unsigned)chosen;
	uint32_t b = t * RH_TCS_STRIDE, bit = 1u << t;
	uint32_t irq_en = rd(RH_TCS + RH_IRQ_EN);
	if (!rh_sync(RH_IRQ_EN, irq_en & ~bit))
		return -1;
	const uint32_t offs[3] = {0, 8, 4}, vals[3] = {mv, 7, 1};
	for (unsigned c = 0; c < 3; c++) {
		uint32_t cmd = b + c * RH_CMD_STRIDE;
		if (!rh_sync(cmd + RH_MSGID, RH_WRITE_ACK) || !rh_sync(cmd + RH_ADDR, address + offs[c]) ||
		    !rh_sync(cmd + RH_DATA, vals[c]))
			return -1;
	}
	if (!rh_sync(b + RH_WAIT, 7) || !rh_sync(b + RH_ENABLE, 7) ||
	    !rh_sync(b + RH_CONTROL, 0) || !rh_sync(b + RH_CONTROL, RH_AMC_EN))
		return -1;
	wr(RH_TCS + b + RH_CONTROL, RH_AMC_EN | RH_TRIGGER);
	/* ios-sweet 0062: with our IRQ source masked the status bit may never
	 * latch, so also accept an observed busy->idle transition with every
	 * command ISSUED|COMPLETE. */
	bool done = false, progress = false;
	uint32_t irq = 0, idle = 0, cs0 = 0;
	for (unsigned us = 0; us < 100000 && !done; us++) {
		irq = rd(RH_TCS + RH_IRQ_STATUS);
		idle = rd(RH_TCS + b + RH_STATUS);
		bool complete = true;
		for (unsigned c = 0; c < 3; c++) {
			uint32_t st = rd(RH_TCS + b + c * RH_CMD_STRIDE + RH_CMD_STATUS);
			if (c == 0)
				cs0 = st;
			complete &= st != 0xffffffffu && (st & (RH_ISSUED | RH_COMPLETE)) == (RH_ISSUED | RH_COMPLETE);
		}
		progress |= idle == 0 || !complete;
		done = idle && idle != 0xffffffffu && ((irq & bit) || progress) && complete;
		if (!done)
			udelay(1);
	}
	if (!done)
		con_hex2("RPMH", "timeout idle=", idle, "cmd0=", cs0);
	/* Release the TCS the way rpmh-rsc does: clear, disable, then re-arm IRQ. */
	rh_sync(b + RH_CONTROL, 0);
	rh_sync(b + RH_ENABLE, 0);
	rh_sync(b + RH_WAIT, 0);
	wr(RH_TCS + RH_IRQ_CLEAR, bit);
	rh_sync(RH_IRQ_EN, irq_en);
	con_hex2("RPMH", res[4] == '2' ? "ldoc2 addr=" : "ldoc7 addr=", address, done ? "OK irq=" : "FAIL irq=", irq);
	return done ? 0 : -1;
}

/* ------------------------------------------------------- clocks + pins */
#define GCC		0x00100000UL
#define GCC_VOTE	0x52008u	/* APCS clock branch enable vote */
#define GCC_SE_RCG	0x17270u	/* gcc_qupv3_wrap0_s2_clk_src */
#define TLMM		0x0f100000UL	/* TLMM tile base + REG_BASE 0x100000 */
#define GPIO_CFG(n)	(TLMM + 0x1000u * (n))
#define GPIO_IO(n)	(GPIO_CFG(n) + 4u)
#define TS_RESET	30u
#define TS_IRQ		31u

static bool wait_clear(uintptr_t a, uint32_t mask)
{
	for (unsigned i = 0; i < 10000; i++) {
		if (!(rd(a) & mask))
			return true;
		udelay(10);
	}
	return false;
}

static bool clocks(void)
{
	/* wrap0 m_ahb(6) s_ahb(7) core(8) core_2x(9), SE2(12). */
	wr(GCC + GCC_VOTE, rd(GCC + GCC_VOTE) | (0xfu << 6) | (1u << 12));
	uint32_t dfs = rd(GCC + GCC_SE_RCG + 0x14), cfg = rd(GCC + GCC_SE_RCG + 4);
	con_hex2("GCC", "vote=", rd(GCC + GCC_VOTE), "rcg cfg=", cfg);
	if (dfs & 1) {
		/* DFS owned: SE CLK_SEL 0 selects perf level 0; require XO/1. */
		uint32_t l0 = rd(GCC + GCC_SE_RCG + 0x1c);
		con_hex2("GCC", "dfs=", dfs, "level0=", l0);
		if ((l0 & 0x371f) != 1)
			return false;
	} else {
		wr(GCC + GCC_SE_RCG + 4, (cfg & ~(0x371fu | (1u << 20))) | 1u); /* XO, div 1 */
		wr(GCC + GCC_SE_RCG, rd(GCC + GCC_SE_RCG) | 1u);
		if (!wait_clear(GCC + GCC_SE_RCG, 1u))
			return false;
	}
	const uint32_t br[] = {0x17004, 0x17008, 0x23000, 0x23008, 0x1726c};
	for (unsigned i = 0; i < 5; i++) {
		/* AHB branches with hardware clock gating on (bit 1) legitimately
		 * read CLK_OFF while idle; Linux skips their halt check too. */
		if (rd(GCC + br[i]) & 2u)
			continue;
		if (!wait_clear(GCC + br[i], 1u << 31)) {
			con_hex2("GCC", "branch off=", br[i], "val=", rd(GCC + br[i]));
			return false;
		}
	}
	return true;
}

static void gpio_cfg(unsigned pin, unsigned func, unsigned pull, unsigned drive_ma, bool out, bool high)
{
	if (out)
		wr(GPIO_IO(pin), high ? 2u : 0u);
	uint32_t c = rd(GPIO_CFG(pin)) & ~0x3ffu;
	c |= pull | (func << 2) | (((drive_ma / 2) - 1) << 6) | (out ? 0x200u : 0u);
	wr(GPIO_CFG(pin), c);
}

/* ---------------------------------------------------------- GENI SPI */
#define SE		0x00988000UL	/* QUP wrapper0 SE2 */
#define SE_FORCE_DEF	0x020u
#define SE_OUT_CTRL	0x024u
#define SE_CGC_CTRL	0x028u
#define SE_STATUS	0x040u
#define SE_MCLK		0x048u
#define SE_IF_DIS	0x064u
#define SE_FW		0x068u
#define SE_CLK_SEL	0x07cu
#define SPI_CPHA	0x224u
#define SPI_LOOPBACK	0x22cu
#define SPI_CPOL	0x230u
#define SPI_DEMUX_INV	0x24cu
#define SPI_DEMUX_SEL	0x250u
#define SE_BYTE_GRAN	0x254u
#define SE_DMA		0x258u
#define SPI_TRANS_CFG	0x25cu
#define SE_TX_PACK0	0x260u
#define SE_TX_PACK1	0x264u
#define SPI_WORD_LEN	0x268u
#define SPI_TX_LEN	0x26cu
#define SPI_RX_LEN	0x270u
#define SPI_PRE_POST	0x274u
#define SPI_DELAYS	0x278u
#define SE_RX_PACK0	0x284u
#define SE_RX_PACK1	0x288u
#define SE_CMD		0x600u
#define SE_CTRL		0x604u
#define SE_IRQ		0x610u
#define SE_IRQ_EN	0x614u
#define SE_IRQ_CLR	0x618u
#define SE_S_IRQ_EN	0x644u
#define SE_S_IRQ_CLR	0x648u
#define SE_TX		0x700u
#define SE_RX		0x780u
#define SE_RX_STAT	0x804u
#define SE_TX_WM	0x80cu
#define SE_RX_WM	0x810u
#define SE_RX_RFR	0x814u
#define SE_GSI_EV	0xe18u
#define SE_IRQ_ROUTE	0xe1cu
#define SE_HW_TX	0xe24u
#define SE_HW_RX	0xe28u
#define SE_DMA_CFG	0xe30u
#define M_DONE		(1u << 0)
#define M_ERRORS	(0x0eu | (3u << 24) | (3u << 28))	/* overrun/illegal/failure, FIFO rd/wr errors */

static unsigned tx_depth, rx_depth;
static bool spi_dead;

/* Linux geni_se_config_packing(): 8-bit words, 4 per FIFO word, MSB first. */
static void packing(uint32_t *c0, uint32_t *c1)
{
	uint32_t v[4];
	for (unsigned i = 0; i < 4; i++)
		v[i] = ((8u * i + 7u) << 5) | (1u << 4) | (7u << 1);
	v[3] |= 1u;
	*c0 = v[0] | (v[1] << 10);
	*c1 = v[2] | (v[3] << 10);
}

static bool spi_init(void)
{
	uint32_t fw = rd(SE + SE_FW), tx = rd(SE + SE_HW_TX), rx = rd(SE + SE_HW_RX);
	con_hex2("SPI SE2", "fw=", fw, "status=", rd(SE + SE_STATUS));
	tx_depth = (tx >> 16) & 0x3f;
	rx_depth = (rx >> 16) & 0x3f;
	if (((fw >> 8) & 0xff) != 1 || (rd(SE + SE_IF_DIS) & 1) || ((tx >> 24) & 0x3f) != 32 ||
	    ((rx >> 24) & 0x3f) != 32 || tx_depth < 2 || rx_depth < 2) {
		con_hex2("SPI SE2", "no FIFO: if_disable=", rd(SE + SE_IF_DIS), "hw rx=", rx);
		return false;
	}
	if (rd(SE + SE_STATUS) & 0x1001u) {
		con_str("SPI SE2", "busy, skipped");
		return false;
	}
	wr(SE + SE_GSI_EV, 0);
	wr(SE + SE_IRQ_ROUTE, 0);
	wr(SE + SE_S_IRQ_EN, 0);
	wr(SE + SE_IRQ_CLR, 0xffffffffu);
	wr(SE + SE_S_IRQ_CLR, 0xffffffffu);
	wr(SE + SE_DMA, 0);
	wr(SE + SE_CGC_CTRL, rd(SE + SE_CGC_CTRL) | 0x7fu);
	wr(SE + SE_DMA_CFG, rd(SE + SE_DMA_CFG) | 0x0fu);
	wr(SE + SE_OUT_CTRL, 0x7f);
	wr(SE + SE_FORCE_DEF, 1);
	wr(SE + SE_CLK_SEL, 0);
	wr(SE + SE_MCLK, (4u << 4) | 1u);	/* 19.2 MHz / 4 = 4.8 MHz (IC max 6 MHz) */
	wr(SE + SPI_LOOPBACK, 0);
	wr(SE + SPI_CPHA, 0);			/* SPI mode 0 */
	wr(SE + SPI_CPOL, 0);
	wr(SE + SPI_DEMUX_INV, 0);
	wr(SE + SPI_DEMUX_SEL, 0);		/* CS0 */
	wr(SE + SPI_TRANS_CFG, 0);		/* no CS toggle between words */
	wr(SE + SPI_PRE_POST, 0);
	wr(SE + SPI_DELAYS, 0);
	wr(SE + SPI_WORD_LEN, (rd(SE + SPI_WORD_LEN) & ~0x3ffu) | (8u - 4u));
	uint32_t p0, p1;
	packing(&p0, &p1);
	wr(SE + SE_BYTE_GRAN, 0);
	wr(SE + SE_TX_PACK0, p0);
	wr(SE + SE_TX_PACK1, p1);
	wr(SE + SE_RX_PACK0, p0);
	wr(SE + SE_RX_PACK1, p1);
	wr(SE + SE_TX_WM, 0);
	wr(SE + SE_RX_WM, 0);
	wr(SE + SE_RX_RFR, rx_depth - 1);
	wr(SE + SE_IRQ_EN, M_DONE | M_ERRORS | (1u << 5) | (1u << 26) | (1u << 27) | (1u << 30));
	if ((rd(SE + 0x800) & 0xfffffff) || (rd(SE + SE_RX_STAT) & 0x1ffffff)) {
		con_str("SPI SE2", "inherited FIFO data, skipped");
		return false;
	}
	return true;
}

static void spi_abort(void)
{
	wr(SE + SE_CTRL, 2);
	for (unsigned i = 0; i < 5000 && !(rd(SE + SE_IRQ) & (1u << 5)); i++)
		udelay(1);
	for (unsigned i = 0; i < 64 && (rd(SE + SE_RX_STAT) & 0x1ffffff); i++)
		(void)rd(SE + SE_RX);
	wr(SE + SE_IRQ_CLR, 0xffffffffu);
	if (rd(SE + SE_STATUS) & 1)
		spi_dead = true;
}

/* One full-duplex transfer with CS held for all n bytes. */
static int spi_xfer(const uint8_t *tx, uint8_t *rx, unsigned n)
{
	if (spi_dead || !n || n > 0xffffff)
		return -1;
	if (!wait_clear(SE + SE_STATUS, 1u)) {
		spi_dead = true;
		return -1;
	}
	wr(SE + SE_IRQ_CLR, 0xffffffffu);
	wr(SE + SPI_TX_LEN, n);
	wr(SE + SPI_RX_LEN, n);
	wr(SE + SE_CMD, 3u << 27);		/* FULL_DUPLEX, CS released at the end */
	unsigned sent = 0, got = 0;
	uint64_t deadline = ticks() + rsr("cntfrq_el0") / 10;	/* 100 ms */
	while (ticks() < deadline) {
		uint32_t irq = rd(SE + SE_IRQ);
		if (irq & M_ERRORS) {
			con_hex2("SPI SE2", "error irq=", irq, "status=", rd(SE + SE_STATUS));
			break;
		}
		while (sent < n && (rd(SE + 0x800) & 0xfffffff) < tx_depth) {
			uint32_t w = 0;
			for (unsigned j = 0; j < 4 && sent + j < n; j++)
				w |= (uint32_t)(tx ? tx[sent + j] : 0) << (8u * j);
			wr(SE + SE_TX, w);
			sent += 4;
		}
		uint32_t cnt = rd(SE + SE_RX_STAT) & 0x1ffffff;
		while (cnt--) {
			uint32_t w = rd(SE + SE_RX);
			for (unsigned j = 0; j < 4 && got < n; j++, got++)
				if (rx)
					rx[got] = (uint8_t)(w >> (8u * j));
		}
		if (irq & M_DONE) {
			if (got < n) {
				cnt = rd(SE + SE_RX_STAT) & 0x1ffffff;
				while (cnt--) {
					uint32_t w = rd(SE + SE_RX);
					for (unsigned j = 0; j < 4 && got < n; j++, got++)
						if (rx)
							rx[got] = (uint8_t)(w >> (8u * j));
				}
			}
			wr(SE + SE_IRQ_CLR, irq);
			return got == n ? 0 : -1;
		}
		wr(SE + SE_IRQ_CLR, irq & ((1u << 26) | (1u << 27) | (1u << 30)));
	}
	spi_abort();
	return -1;
}

/* ------------------------------------------------- bit-banged SPI */
/*
 * QUP0 SE2 comes up with FIFO_IF_DISABLE: it is a GSI-only engine (DMA via
 * the GPI block behind the SMMU, shared with the trusted-touch VM). Instead
 * of all that, drive the four pins as plain GPIOs. SE pin order on QUP SPI:
 * l0 MISO = GPIO20, l1 MOSI = GPIO21, l2 SCLK = GPIO22, l3 CS0 = GPIO23.
 * SPI mode 0, MSB first, CS active low. MMIO speed keeps SCLK far below the
 * IC's 6 MHz limit.
 */
#define BB_MISO 20u
#define BB_MOSI 21u
#define BB_CLK  22u
#define BB_CS   23u
static inline void pin(unsigned n, unsigned v) { *(volatile uint32_t *)GPIO_IO(n) = v ? 2u : 0u; }
static inline unsigned pin_in(unsigned n) { return *(volatile uint32_t *)GPIO_IO(n) & 1u; }

static void bb_init(void)
{
	gpio_cfg(BB_CS, 0, 0, 6, true, true);
	gpio_cfg(BB_CLK, 0, 0, 6, true, false);
	gpio_cfg(BB_MOSI, 0, 0, 6, true, false);
	gpio_cfg(BB_MISO, 0, 1, 2, false, false);	/* input, pull-down */
}

static int bb_xfer(const uint8_t *tx, uint8_t *rx, unsigned n)
{
	pin(BB_CLK, 0);
	pin(BB_CS, 0);
	udelay(1);
	for (unsigned i = 0; i < n; i++) {
		unsigned o = tx ? tx[i] : 0xff, in = 0;
		for (int b = 7; b >= 0; b--) {
			pin(BB_MOSI, (o >> b) & 1u);
			pin(BB_CLK, 1);
			in = (in << 1) | pin_in(BB_MISO);
			pin(BB_CLK, 0);
		}
		if (rx)
			rx[i] = (uint8_t)in;
	}
	udelay(1);
	pin(BB_CS, 1);
	udelay(2);
	return 0;
}

/* ------------------------------------------------- Goodix Berlin (SPI) */
static uint8_t xbuf[9 + 512], rbuf[9 + 512];

static int gdx_read(uint32_t addr, uint8_t *data, unsigned len)
{
	if (len > 512)
		return -1;
	xbuf[0] = 0xF1;
	xbuf[1] = addr >> 24; xbuf[2] = addr >> 16; xbuf[3] = addr >> 8; xbuf[4] = addr;
	for (unsigned i = 5; i < 9 + len; i++)
		xbuf[i] = 0xFF;
	if (bb_xfer(xbuf, rbuf, 9 + len))
		return -1;
	for (unsigned i = 0; i < len; i++)
		data[i] = rbuf[9 + i];
	return 0;
}

static int gdx_write(uint32_t addr, const uint8_t *data, unsigned len)
{
	if (len > 512)
		return -1;
	xbuf[0] = 0xF0;
	xbuf[1] = addr >> 24; xbuf[2] = addr >> 16; xbuf[3] = addr >> 8; xbuf[4] = addr;
	for (unsigned i = 0; i < len; i++)
		xbuf[5 + i] = data[i];
	return bb_xfer(xbuf, rbuf, 5 + len);
}

static bool gdx_write_check(uint32_t addr, uint8_t v)
{
	for (unsigned i = 0; i < 5; i++) {
		uint8_t r = 0;
		if (!gdx_write(addr, &v, 1) && !gdx_read(addr, &r, 1) && r == v)
			return true;
	}
	return false;
}

/* Xiaomi brl_reset_after(): SPI mode, hold CPU, clocks, trim, soft reset. */
static bool gdx_reset_after(void)
{
	mdelay(5);
	if (!gdx_write_check(0xC900, 0x01)) {
		con_str("GOODIX", "SPI mode select: no answer");
		return false;
	}
	con_str("GOODIX", "SPI mode 0xC900 = 01 read back OK");
	bool held = false;
	for (unsigned i = 0; i < 10 && !held; i++) {
		uint8_t w[2] = {0x01, 0x00}, a[4], b[4], c[4];
		if (gdx_write(0x0002, w, 2) || gdx_read(0x2000, a, 4) || gdx_read(0x2000, b, 4) ||
		    gdx_read(0x2000, c, 4))
			continue;
		held = true;
		for (unsigned j = 0; j < 4; j++)
			if (a[j] != b[j] || b[j] != c[j])
				held = false;
	}
	if (!held || !gdx_write_check(0xD807, 0xFF) || !gdx_write_check(0xD806, 0x77) ||
	    !gdx_write_check(0xD006, 0x3C)) {
		con_str("GOODIX", "hold-cpu/clock/trim sequence failed");
		return false;
	}
	mdelay(5);
	uint8_t rst = 0xFA;
	gdx_write(0xD808, &rst, 1);
	return gdx_write_check(0xC900, 0x01);
}

static void ascii(char *out, const uint8_t *in, unsigned n)
{
	unsigned k = 0;
	for (unsigned i = 0; i < n; i++)
		out[k++] = (in[i] >= 0x20 && in[i] < 0x7f) ? (char)in[i] : '.';
	out[k] = 0;
}

static void square(unsigned cx, unsigned cy, uint32_t color)
{
	for (int dy = -24; dy <= 24; dy++)
		for (int dx = -24; dx <= 24; dx++) {
			int x = (int)cx + dx, y = (int)cy + dy;
			if (x >= 0 && x < FB_W && y >= 0 && y < FB_H)
				con_fb[y * (FB_STRIDE / 4) + x] = color;
		}
}

void touch_probe(void)
{
	con_skip();
	con_str("TOUCH", "renoir T1b: Goodix GT9897T, GPIO SPI");
	/* Reset low before any supply comes up (Xiaomi power-on order). */
	gpio_cfg(TS_RESET, 0, 0, 2, true, false);
	if (rh_init() || rh_vreg_on("ldoc2", 1800))
		return;
	mdelay(3);
	if (rh_vreg_on("ldoc7", 3300))
		return;
	mdelay(15);
	if (!clocks()) {
		con_str("GCC", "QUP0 SE2 clocks did not start");
		return;
	}
	(void)spi_init();	/* report the SE's state; transfers are bit-banged */
	bb_init();
	gpio_cfg(TS_RESET, 0, 0, 2, true, true);	/* release reset */
	if (!gdx_reset_after())
		return;
	mdelay(100);

	uint8_t v[28];
	char s1[16], s2[16];
	if (gdx_read(0x1000C, v, sizeof(v))) {
		con_str("GOODIX", "FW version read failed");
		return;
	}
	ascii(s1, v, 6);
	ascii(s2, v + 10, 8);
	con_str("GOODIX rom", s1);
	con_str("GOODIX patch", s2);
	con_hex2("GOODIX", "vid=", (uint32_t)v[18] << 24 | v[19] << 16 | v[20] << 8 | v[21], "sensor=", v[23]);

	/* IC info: length, then parse to misc.touch_data_addr. */
	uint8_t info[512];
	if (gdx_read(0x10068, info, 2)) {
		con_str("GOODIX", "IC info read failed");
		return;
	}
	unsigned len = info[0] | (unsigned)info[1] << 8;
	if (len < 64 || len > sizeof(info) || gdx_read(0x10068, info, len)) {
		con_hex("GOODIX info len", len);
		return;
	}
	unsigned o = 2 + 16 + 10;		/* length, version, feature */
	o += 4;					/* drv, sen, button, force */
	unsigned n = info[o++];			/* active scan rates */
	o += 2 * n;
	for (unsigned k = 0; k < 4; k++) {	/* mutual, self tx, self rx, stylus */
		n = info[o++];
		o += 2 * n;
	}
	if (o + 48 > len) {
		con_hex("GOODIX misc off", o);
		return;
	}
	uint32_t tda = info[o + 44] | (uint32_t)info[o + 45] << 8 | (uint32_t)info[o + 46] << 16 |
	    (uint32_t)info[o + 47] << 24;
	con_hex2("GOODIX", "drv/sen=", (uint32_t)info[28] << 8 | info[29], "touch_data=", tda);
	if (tda < 0x10000 || tda > 0x20000)
		return;

	con_skip();
	con_str("TOUCH", "touch the screen! (60 s, white = finger)");
	uint64_t end = ticks() + rsr("cntfrq_el0") * 60;
	unsigned shown = 0;
	while (ticks() < end) {
		uint8_t e[8 + 8 * 10 + 2];
		if (gdx_read(tda, e, 8 + 8 + 2) == 0 && (e[0] & 0x80)) {
			unsigned num = e[2] & 0x0f;
			if (num > 1 && num <= 10)
				gdx_read(tda + 18, e + 18, (num - 1) * 8);
			for (unsigned i = 0; i < num && i < 10; i++) {
				const uint8_t *pt = e + 8 + 8 * i;
				unsigned x = pt[2] | pt[3] << 8, y = pt[4] | pt[5] << 8;
				square(x, y, WHITE);
				if (shown < 8) {
					con_hex2("FINGER", "x=", x, "y=", y);
					shown++;
				}
			}
			uint8_t z = 0;
			gdx_write(tda, &z, 1);		/* sync clean, as the Xiaomi driver */
		}
		mdelay(8);
	}
	con_str("TOUCH", "done");
}
