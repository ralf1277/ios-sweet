#pragma once
#include <stdint.h>

#ifdef BOARD_RENOIR
#define FB_PHYS	0xe1000000UL
#else
#define FB_PHYS	0x9c000000UL
#endif

struct mmu_info {
	uint64_t img_lo, img_hi, ttbr;
	uint64_t sctlr_before, sctlr_after;
	int n_l3;
};

int mmu_enable(struct mmu_info *info);
uint64_t mmu_probe(uintptr_t va);
