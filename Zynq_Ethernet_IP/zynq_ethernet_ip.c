/*
 * =========================================================================================
 * Project     : Zynq-7000 (ZC702) Gigabit Ethernet Counter -- HARDWARE DATAPATH VERSION
 * File        : zynq_ethernet_ip.c
 * Description : Same lwIP TCP server as before (PC <-> Ethernet <-> Zynq, port 7,
 *               32-byte payload), but the "+1 per byte" computation is now done by the
 *               axis_counter_plus1 custom IP in the PL, NOT by a software for-loop.
 *               Software's only job for the data path is: copy the received TCP payload
 *               into a DDR buffer, kick off the AXI DMA (which streams it through
 *               axis_counter_plus1 and back), wait for completion, then tcp_write() the
 *               result straight out of the destination DDR buffer.
 * =========================================================================================
 *
 * =========================================================================================
 *       FUNCTION OWNERSHIP & CODE DISTRIBUTION BETWEEN THE TWO TEAMS
 * =========================================================================================
 *
 * -----------------------------------------------------------------------------------------
 * TEAM 1 FUNCTIONS: SYSTEM, DRIVER & HARDWARE INTERFACE LAYER (5 Functions)
 * -----------------------------------------------------------------------------------------
 *  1. init_platform() / cleanup_platform() -> Hardware cache, MMU, and timer initialization.
 *  2. IP4_ADDR() & MAC Setup               -> Physical network addressing & subnet config.
 *  3. xemac_add() & netif_set_up()         -> Low-level GEM0 driver binding to Marvell PHY.
 *  4. xemacif_input()                      -> DMA receive ring polling to fetch Ethernet frames.
 *  5. init_axi_dma()                       -> AXI DMA driver bring-up (NEW: drives the
 *                                             axis_counter_plus1 hardware datapath).
 *
 * -----------------------------------------------------------------------------------------
 * TEAM 2 FUNCTIONS: TCP PROTOCOL, ORCHESTRATION & APPLICATION LAYER (4 Functions)
 * -----------------------------------------------------------------------------------------
 *  6. lwip_init()                          -> lwIP protocol stack & memory pool allocation.
 *  7. tcp_new() / tcp_bind() / tcp_listen() -> TCP socket creation and port 7 listener.
 *  8. accept_callback()                    -> TCP 3-way handshake handler & callback linking.
 *  9. recv_callback()                      -> Payload copy into DDR, triggers the AXI DMA
 *                                             transfer through axis_counter_plus1, waits
 *                                             for completion, tcp_write()/tcp_output(),
 *                                             pbuf_free(). The "+1" itself now happens in
 *                                             the PL, not in this function.
 * =========================================================================================
 */

#include <stdio.h>
#include <string.h>

#include "xparameters.h"
#include "netif/xadapter.h"
#include "xil_printf.h"
#include "xil_cache.h"
#include "xaxidma.h"
#include "lwip/tcp.h"
#include "lwip/err.h"

/*
 * [SOLUTION 1 IMPLEMENTATION]: Local definition of platform initialization routines
 * This replaces the missing "platform.h" header file directly.
 */
static void init_platform(void) {
    Xil_ICacheEnable();
    Xil_DCacheEnable();
}

static void cleanup_platform(void) {
    Xil_DCacheDisable();
    Xil_ICacheDisable();
}

/*
 * [SHARED SPECIFICATIONS: TEAM 1 & TEAM 2]
 * BUFFER_SIZE : 32 bytes (matches in_Size and out_Size from tutorial t_1)
 * SERVER_PORT : 7 (Standard TCP Echo / Counter port)
 */
#define SERVER_PORT 7
#define BUFFER_SIZE 32

/*
 * [TEAM 1] DMA source/destination scratch buffers in DDR.
 * Fixed addresses in a region well above the program's own .text/.data/heap/stack
 * (matches the Xilinx AXI DMA polling-mode example convention). Both axi_dma_0's
 * MM2S and S2MM channels are mapped through S_AXI_ACP -> ACP_DDR_LOWOCM in this
 * project (see Address Editor), which is cache-coherent -- the explicit
 * Xil_DCache* calls below are kept anyway as defensive/portable practice in case
 * the port mapping is ever changed to S_AXI_HP0 (non-coherent).
 */
#define DMA_SRC_ADDR   0x01000000
#define DMA_DST_ADDR   0x01010000
#define DMA_DEV_ID     XPAR_AXIDMA_0_DEVICE_ID

static struct netif server_netif;
static XAxiDma       AxiDma;

/*
 * =========================================================================================
 * [FUNCTION 5 - TEAM 1]: init_axi_dma()
 * Layer       : Hardware Interface Layer
 * Description : Brings up the axi_dma_0 IP driver instance. Must succeed before any
 *               TCP connection is accepted, since recv_callback() depends on AxiDma
 *               being ready.
 * =========================================================================================
 */
static int init_axi_dma(void) {
    XAxiDma_Config *dma_cfg;

    dma_cfg = XAxiDma_LookupConfig(DMA_DEV_ID);
    if (!dma_cfg) {
        xil_printf("Error: No AXI DMA config found for device ID %d.\n\r", DMA_DEV_ID);
        return XST_FAILURE;
    }

    if (XAxiDma_CfgInitialize(&AxiDma, dma_cfg) != XST_SUCCESS) {
        xil_printf("Error: AXI DMA driver initialization failed.\n\r");
        return XST_FAILURE;
    }

    if (XAxiDma_HasSg(&AxiDma)) {
        xil_printf("Error: This code expects axi_dma_0 in Simple (non-SG) mode.\n\r");
        return XST_FAILURE;
    }

    /* Disable channel interrupts -- we poll for completion in recv_callback() */
    XAxiDma_IntrDisable(&AxiDma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DMA_TO_DEVICE);
    XAxiDma_IntrDisable(&AxiDma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);

    xil_printf("AXI DMA (axi_dma_0) ready -- axis_counter_plus1 datapath armed.\n\r");
    return XST_SUCCESS;
}

/*
 * =========================================================================================
 * [FUNCTION 9 - TEAM 2]: recv_callback()
 * Layer       : Application & Orchestration Layer
 * Triggered by: Incoming TCP packet carrying payload from PC
 * Replaces    : the software "for (i...) out_data[i] = in_data[i] + 1;" loop with a
 *               hardware transfer through axis_counter_plus1.
 * =========================================================================================
 */
err_t recv_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (!p) {
        xil_printf("Remote client disconnected.\n\r");
        tcp_close(tpcb);
        return ERR_OK;
    }

    tcp_recved(tpcb, p->len);

    u16_t len = p->len;
    if (len > BUFFER_SIZE) {
        len = BUFFER_SIZE; /* clamp -- same 32-byte contract as before */
    }

    xil_printf("Received %d bytes from PC via Ethernet.\n\r", len);

    /* 1) stage the input bytes into the DMA source buffer in DDR */
    memcpy((void *)DMA_SRC_ADDR, p->payload, len);
    Xil_DCacheFlushRange((UINTPTR)DMA_SRC_ADDR, len);

    /* 2) arm S2MM (RX) FIRST so the destination is ready before data starts moving,
     *    then kick off MM2S (TX) -- same ordering the DMA needs whether a human,
     *    software, or a hardware sequencer is driving it. */
    if (XAxiDma_SimpleTransfer(&AxiDma, DMA_DST_ADDR, len, XAXIDMA_DEVICE_TO_DMA) != XST_SUCCESS) {
        xil_printf("Error: S2MM SimpleTransfer failed.\n\r");
        pbuf_free(p);
        return ERR_OK;
    }
    if (XAxiDma_SimpleTransfer(&AxiDma, DMA_SRC_ADDR, len, XAXIDMA_DMA_TO_DEVICE) != XST_SUCCESS) {
        xil_printf("Error: MM2S SimpleTransfer failed.\n\r");
        pbuf_free(p);
        return ERR_OK;
    }

    /* 3) poll for completion -- once S2MM is no longer busy, every byte has been
     *    through axis_counter_plus1 (+1 per byte) and landed in DMA_DST_ADDR. */
    while (XAxiDma_Busy(&AxiDma, XAXIDMA_DEVICE_TO_DMA)) {
        /* busy-wait; fine for a 32-byte transfer, negligible latency */
    }

    Xil_DCacheInvalidateRange((UINTPTR)DMA_DST_ADDR, len);

    /* 4) TRANSMISSION: send the hardware-incremented bytes straight out of DDR */
    err_t write_status = tcp_write(tpcb, (void *)DMA_DST_ADDR, len, TCP_WRITE_FLAG_COPY);
    if (write_status == ERR_OK) {
        tcp_output(tpcb);
        xil_printf("Transmitted %d hardware-incremented bytes to PC.\n\r", len);
    } else {
        xil_printf("tcp_write failed with error code: %d\n\r", write_status);
    }

    pbuf_free(p);
    return ERR_OK;
}

/*
 * =========================================================================================
 * [FUNCTION 8 - TEAM 2]: accept_callback()
 * Layer       : TCP Session Management Layer
 * Triggered by: Completion of TCP 3-Way Handshake (SYN -> SYN-ACK -> ACK) from PC
 * =========================================================================================
 */
err_t accept_callback(void *arg, struct tcp_pcb *newpcb, err_t err) {
    xil_printf("TCP Handshake complete! PC connected.\n\r");
    tcp_recv(newpcb, recv_callback);
    return ERR_OK;
}

/*
 * =========================================================================================
 * MAIN PROGRAM
 * =========================================================================================
 */
int main() {
    ip_addr_t ipaddr, netmask, gw;

    /* [FUNCTION 1 - TEAM 1]: init_platform() - Enables ARM caches and MMU */
    init_platform();

    xil_printf("\n\r=========================================================\n\r");
    xil_printf("  Zynq-7000 Gigabit Ethernet Counter (HW datapath, lwIP RAW) \n\r");
    xil_printf("=========================================================\n\r");

    /* [FUNCTION 5 - TEAM 1]: init_axi_dma() - Bring up axi_dma_0 before accepting clients */
    if (init_axi_dma() != XST_SUCCESS) {
        xil_printf("Fatal: AXI DMA bring-up failed, halting.\n\r");
        return -1;
    }

    /* [FUNCTION 2 - TEAM 1]: IP4_ADDR() & MAC Setup - Static network addressing */
    unsigned char mac_ethernet_address[] = { 0x00, 0x0a, 0x35, 0x00, 0x01, 0x02 };
    IP4_ADDR(&ipaddr,  192, 168,   1, 10);
    IP4_ADDR(&netmask, 255, 255, 255,   0);
    IP4_ADDR(&gw,      192, 168,   1,   1);

    /* [FUNCTION 6 - TEAM 2]: lwip_init() - Protocol stack & memory pool allocation */
    lwip_init();

    /* [FUNCTION 3 - TEAM 1]: xemac_add() - Bind lwIP to Zynq GEM0 hardware */
#if defined (XPAR_XEMACPS_0_BASEADDR)
    if (!xemac_add(&server_netif, &ipaddr, &netmask, &gw, mac_ethernet_address, XPAR_XEMACPS_0_BASEADDR)) {
#else
    if (!xemac_add(&server_netif, &ipaddr, &netmask, &gw, mac_ethernet_address, PLATFORM_EMAC_BASEADDR)) {
#endif
        xil_printf("Error: Failed to bind GEM0 network interface!\n\r");
        return -1;
    }
    netif_set_default(&server_netif);
    netif_set_up(&server_netif);
    xil_printf("GEM0 bound to 192.168.1.10 successfully.\n\r");

    /* [FUNCTION 7 - TEAM 2]: tcp_new/bind/listen/accept - TCP server on port 7 */
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb) { xil_printf("Error: Failed to allocate TCP PCB.\n\r"); return -1; }
    if (tcp_bind(pcb, IP_ADDR_ANY, SERVER_PORT) != ERR_OK) { xil_printf("Error: Bind failed.\n\r"); return -1; }
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, accept_callback);
    xil_printf("TCP Server listening on port %d. Ready.\n\r", SERVER_PORT);

    /* [FUNCTION 4 - TEAM 1]: xemacif_input() - DMA polling loop */
    while (1) {
        xemacif_input(&server_netif);
    }

    cleanup_platform();
    return 0;
}
