/*-
 * Copyright (c) 2014-2019 Mindaugas Rasiukevicius <rmind at netbsd org>
 * Copyright (c) 2010-2013 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This material is based upon work partially supported by The
 * NetBSD Foundation under a contract with Mindaugas Rasiukevicius.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifdef _KERNEL
#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: npf_nat.c,v 1.45 2019/01/19 21:19:32 rmind Exp $");

#include <sys/param.h>
#include <sys/types.h>

#include <sys/atomic.h>
#include <sys/bitops.h>
#include <sys/kmem.h>
#include <sys/mutex.h>
#include <sys/cprng.h>
#endif

#include "npf_impl.h"
#include "npf_conn.h"

/*
 * NPF portmap structure.
 */
struct npf_portmap {
	unsigned		p_refcnt;
	uint32_t		p_bitmap[0];
};

/* Portmap range: [ 1024 .. 65535 ] */
#define	PORTMAP_FIRST		(1024)
#define	PORTMAP_SIZE		((65536 - PORTMAP_FIRST) / 32)
#define	PORTMAP_FILLED		((uint32_t)~0U)
#define	PORTMAP_MASK		(31)
#define	PORTMAP_SHIFT		(5)

#define	PORTMAP_MEM_SIZE	\
    (sizeof(npf_portmap_t) + (PORTMAP_SIZE * sizeof(uint32_t)))

npf_portmap_t *
npf_portmap_create(void)
{
	npf_portmap_t *pm;

	pm = kmem_zalloc(PORTMAP_MEM_SIZE, KM_SLEEP);
	pm->p_refcnt = 1;
	KASSERT((uintptr_t)pm->p_bitmap == (uintptr_t)pm + sizeof(*pm));
	return pm;
}

void
npf_portmap_destroy(npf_portmap_t *pm)
{
	KASSERT(pm->p_refcnt == 0);
	kmem_free(pm, PORTMAP_MEM_SIZE);
}

void
npf_portmap_getref(npf_portmap_t *pm)
{
	atomic_inc_uint(&pm->p_refcnt);
}

void
npf_portmap_putref(npf_portmap_t *pm)
{
	KASSERT(pm->p_refcnt > 0);

	/* Destroy the port map on last reference. */
	if (atomic_dec_uint_nv(&pm->p_refcnt) != 0) {
		return;
	}
	npf_portmap_destroy(pm);
}

/*
 * npf_portmap_get: allocate and return a port from the given portmap.
 *
 * => Returns the port value in network byte-order.
 * => Zero indicates a failure.
 */
in_port_t
npf_portmap_get(npf_portmap_t *pm)
{
	unsigned n = PORTMAP_SIZE, idx, bit;
	uint32_t map, nmap;

	KASSERT(pm->p_refcnt > 0);

	idx = cprng_fast32() % PORTMAP_SIZE;
	for (;;) {
		KASSERT(idx < PORTMAP_SIZE);
		map = pm->p_bitmap[idx];
		if (__predict_false(map == PORTMAP_FILLED)) {
			if (n-- == 0) {
				/* No space. */
				return 0;
			}
			/* This bitmap is filled, next. */
			idx = (idx ? idx : PORTMAP_SIZE) - 1;
			continue;
		}
		bit = ffs32(~map) - 1;
		nmap = map | (1U << bit);
		if (atomic_cas_32(&pm->p_bitmap[idx], map, nmap) == map) {
			/* Success. */
			break;
		}
	}
	return htons(PORTMAP_FIRST + (idx << PORTMAP_SHIFT) + bit);
}

/*
 * npf_portmap_take: allocate a specific port in the portmap.
 */
bool
npf_portmap_take(npf_portmap_t *pm, in_port_t port)
{
	uint32_t map, nmap;
	unsigned idx, bit;

	KASSERT(pm->p_refcnt > 0);

	port = ntohs(port) - PORTMAP_FIRST;
	idx = port >> PORTMAP_SHIFT;
	bit = port & PORTMAP_MASK;
	map = pm->p_bitmap[idx];
	nmap = map | (1U << bit);
	if (map == nmap) {
		/* Already taken. */
		return false;
	}
	return atomic_cas_32(&pm->p_bitmap[idx], map, nmap) == map;
}

/*
 * npf_portmap_put: release the port, making it available in the portmap.
 *
 * => The port value should be in network byte-order.
 */
void
npf_portmap_put(npf_portmap_t *pm, in_port_t port)
{
	uint32_t map, nmap;
	unsigned idx, bit;

	KASSERT(pm->p_refcnt > 0);

	port = ntohs(port) - PORTMAP_FIRST;
	idx = port >> PORTMAP_SHIFT;
	bit = port & PORTMAP_MASK;
	do {
		map = pm->p_bitmap[idx];
		KASSERT(map | (1U << bit));
		nmap = map & ~(1U << bit);
	} while (atomic_cas_32(&pm->p_bitmap[idx], map, nmap) != map);
}
