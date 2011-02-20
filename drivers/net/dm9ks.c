/*

  dm9ks.c: Version 1.33 06/08/2005 
  
        A Davicom DM9000A/DM9010 ISA NIC fast Ethernet driver for Linux.
	Copyright (C) 1997 2005  Sten Wang, Jackal Huang

	This program is free software; you can redistribute it and/or
	modify it under the terms of the GNU General Public License
	as published by the Free Software Foundation; either version 2
	of the License, or (at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.


  (C)Copyright 1997-2005 DAVICOM Semiconductor,Inc. All Rights Reserved.

	
V1.00	10/13/2004	Add new function Early transmit & IP/TCP/UDP Checksum
			offload enable & flow control is default
V1.1	12/29/2004	Add Two packet mode & modify RX function
V1.2	01/14/2005	Add Early transmit mode 
V1.3	03/02/2005	Support kernel 2.6
v1.33   06/08/2005	#define DM9KS_MDRAL		0xf4
			#define DM9KS_MDRAH		0xf5 
*/

#if 0
#define	TD	printk
#else
#define	TD(x)	do {} while (0)
#endif

#if defined(MODVERSIONS)
#include <linux/modversions.h>
#endif
				
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/ioport.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/skbuff.h>
#include <linux/version.h>
#ifdef	CONFIG_DEBUG_SPINLOCK
#undef	CONFIG_DEBUG_SPINLOCK
#endif
#include <linux/spinlock.h>
#include <linux/crc32.h>

#include <linux/delay.h>
#include <asm/dma.h>
#include <asm/io.h>
// #include <asm/arch/regs-mem.h>

#define CARDNAME	"dm9000"
#define DRV_VERSION	"1.33"

/* Board/System/Debug information/definition ---------------- */
#define DM9KS_DEF_MAC		{ 0x00, 0x80, 0x00, 0x80, 0x00, 0x3f } 

#define DM9KS_ID		0x90000A46
/*-------register name-----------------------*/
#define DM9KS_NCR		0x00	/* Network control Reg.*/
#define DM9KS_NSR		0x01	/* Network Status Reg.*/
#define DM9KS_TCR		0x02	/* TX control Reg.*/
#define DM9KS_RXCR		0x05	/* RX control Reg.*/
#define DM9KS_BPTR		0x08
#define DM9KS_EPCR		0x0b
#define DM9KS_EPAR		0x0c
#define DM9KS_EPDRL		0x0d
#define DM9KS_EPDRH		0x0e
#define DM9KS_GPR		0x1f	/* General purpose register */
#define DM9KS_TCR2		0x2d
#define DM9KS_SMCR		0x2f 	/* Special Mode Control Reg.*/
#define DM9KS_ETXCSR		0x30	/* Early Transmit control/status Reg.*/
#define	DM9KS_TCCR		0x31	/* Checksum cntrol Reg. */
#define DM9KS_RCSR		0x32	/* Receive Checksum status Reg.*/
#define DM9KS_MRCMDX		0xf0
#define DM9KS_MRCMD		0xf2
#define DM9KS_MDRAL		0xf4
#define DM9KS_MDRAH		0xf5
#define DM9KS_MWCMD		0xf8
#define DM9KS_TXPLL		0xfc
#define DM9KS_TXPLH		0xfd
#define DM9KS_ISR		0xfe
#define DM9KS_IMR		0xff
/*---------------------------------------------*/
#define DM9KS_REG05		0x30	/* SKIP_CRC/SKIP_LONG */ 
#define DM9KS_REGFF		0x83	/* IMR */
#define DM9KS_DISINTR		0x80

#define DM9KS_PHY		0x40	/* PHY address 0x01 */
#define DM9KS_PKT_RDY		0x01	/* Packet ready to receive */
#define DM9KS_MIN_IO		0x300
#define DM9KS_MAX_IO		0x370

#define DM9KS_VID_L		0x28
#define DM9KS_VID_H		0x29
#define DM9KS_PID_L		0x2A
#define DM9KS_PID_H		0x2B

#define DM9KS_RX_INTR		0x01
#define DM9KS_TX_INTR		0x02

#define DM9KS_DWORD_MODE	1
#define DM9KS_BYTE_MODE		2
#define DM9KS_WORD_MODE		0

#define MAX_FRAME_SIZE		1522
#define TRUE			1
#define FALSE			0

#define DMFE_TIMER_WUT  jiffies+(HZ*2)	/* timer wakeup time : 2 second */
#define DMFE_TX_TIMEOUT (HZ*2)		/* tx packet time-out time 1.5 s" */

// #define	DM9KS_DEBUG
#if defined(DM9KS_DEBUG)
static int dmfe_debug = 1;
#define DMFE_DBUG(dbug_now, msg, vaule)\
if (dmfe_debug||dbug_now) printk(KERN_INFO "dmfe: %s %x\n", msg, vaule)
#else
#define DMFE_DBUG(dbug_now, msg, vaule)\
if (dbug_now) printk(KERN_INFO "dmfe: %s %x\n", msg, vaule)
#endif

typedef struct _RX_DESC {
	u8 rxbyte;
	u8 status;
	u16 length;
} RX_DESC;

typedef union {
	u8 buf[4];
	RX_DESC desc;
} rx_t;

enum DM9KS_PHY_mode {
	DM9KS_10MHD   = 0, 
	DM9KS_100MHD  = 1, 
	DM9KS_10MFD   = 4,
	DM9KS_100MFD  = 5, 
	DM9KS_AUTO    = 8, 
} ;

/* Structure/enum declaration ------------------------------- */
typedef struct board_info {
 
	u32 reset_counter;		/* counter: RESET */ 
	u32 reset_tx_timeout;		/* RESET caused by TX Timeout */ 

	u32 io_addr;			/* Register I/O base address */
	u32 io_data;			/* Data I/O address */
	u16 tx_pkt_cnt;

	u8 op_mode;			/* PHY operation mode */
	u8 io_mode;			/* 0:word, 2:byte */
	u8 device_wait_reset;		/* device state */

	struct device	*dev;	     /* parent device */
	struct resource *irq_res;

	struct timer_list timer;
	struct net_device_stats stats;
	unsigned char srom[128];
	spinlock_t lock;

} board_info_t;

/* Protect register accessing */
static spinlock_t reg_lock;
/* Global variable declaration ----------------------------- */
static struct net_device * dmfe_dev = NULL;

/* For module input parameter */
static int mode       = DM9KS_10MFD;
static int media_mode = DM9KS_10MFD;
static int  irq       = 3;
static u32 iobase     = DM9KS_MIN_IO;
static u32 iodata     = DM9KS_MIN_IO | 8;

/* function declaration ------------------------------------- */
static int dmfe_probe(struct net_device *);
static int dmfe_open(struct net_device *);
static int dmfe_start_xmit(struct sk_buff *, struct net_device *);
static void dmfe_tx_done(unsigned long);
static void dmfe_packet_receive(struct net_device *);
static int dmfe_stop(struct net_device *);
static struct net_device_stats * dmfe_get_stats(struct net_device *); 
static int dmfe_do_ioctl(struct net_device *, struct ifreq *, int);
static irqreturn_t dmfe_interrupt(int , void *);
static void dmfe_timer(unsigned long);
static void dmfe_init_dm9000(struct net_device *);
static unsigned long cal_CRC(unsigned char *, unsigned int, u8);
static u8 ior(board_info_t *, int);
static void iow(board_info_t *, int, u8);
static u16 phy_read(board_info_t *, int);
static void phy_write(board_info_t *, int, u16);
#ifdef	USE_E2ROM
static u16 read_srom_word(board_info_t *, int);
#endif
static void dmfe_hash_table(struct net_device *);
#if defined(CHECKSUM)
static u8 check_rx_ready(u8);
#endif

static u32 dm9ks_get_link(struct net_device *dev)
{
	board_info_t *db = netdev_priv(dev);
	return phy_read(db, 1) & (1<<2);
}

static const struct net_device_ops dm9ks_ops = {
	.ndo_open		= &dmfe_open,
	.ndo_stop		= &dmfe_stop,
	.ndo_start_xmit		= &dmfe_start_xmit,
	.ndo_get_stats		= &dmfe_get_stats,
	.ndo_set_multicast_list	= &dmfe_hash_table,
	.ndo_do_ioctl		= &dmfe_do_ioctl,
	.ndo_set_mac_address	= eth_mac_addr,
};

static const struct ethtool_ops dm9ks_ethtool_ops = {
	.get_link	= dm9ks_get_link,
};

DECLARE_TASKLET(dmfe_tx_tasklet,dmfe_tx_done,0);

/* DM9000 network baord routine ---------------------------- */

/*
  Search DM9000 board, allocate space and register it
*/
static struct net_device * __init dmfe_probe1(struct platform_device * pdev)
{
	struct net_device *dev;
	int err;

	dev= alloc_etherdev(sizeof(struct board_info));

	if (!dev)
		return ERR_PTR(-ENOMEM);

	SET_NETDEV_DEV(dev, &pdev->dev);
	err = dmfe_probe(dev);
	if (err)
		goto out;

	err = register_netdev(dev);
	if (err)
		goto out1;
	
	return dev;
out1:
	release_region(dev->base_addr,2);
out:
	free_netdev(dev);
	return ERR_PTR(err);
} 

static int __init dmfe_probe(struct net_device *dev)
{
	struct board_info *db;    /* Point a board information structure */
	u32 id_val;
	u16 i, dm9000_found = FALSE;
#ifdef	USE_E2ROM
#else
	u8 def_mac[] = DM9KS_DEF_MAC;
#endif

	DMFE_DBUG(0, "dmfe_probe()",0);

	/* Search All DM9000 serial NIC */
	do {
		outb(DM9KS_VID_L, iobase);
		mdelay(1);
		id_val = inb(iodata);
		mdelay(1);
		outb(DM9KS_VID_H, iobase);
		mdelay(1);
		id_val |= inb(iodata) << 8;
		mdelay(1);
		outb(DM9KS_PID_L, iobase);
		mdelay(1);
		id_val |= inb(iodata) << 16;
		mdelay(1);
		outb(DM9KS_PID_H, iobase);
		mdelay(1);
		id_val |= inb(iodata) << 24;

		printk("dm9000: ID = %08x\n", id_val);
		if (id_val == DM9KS_ID) {
	
			/* Request IO from system */
			if (!request_region(iobase, 2, dev->name))
				return -ENODEV;

			printk("DM9000: found at I/O = %x, VID = %x \n",iobase, id_val);
			dm9000_found = TRUE;

			/* Allocated board information structure */
			db = netdev_priv(dev);
			memset(db, 0, sizeof(struct board_info));
			spin_lock_init(&db->lock);
			spin_lock_init(&reg_lock);
			dmfe_dev    = dev;
			db->io_addr = iobase;
			db->io_data = iodata;
			/* driver system function */
				
			dev->base_addr 		= iobase;
			dev->irq 		= irq;

			dev->netdev_ops		= &dm9ks_ops;
			dev->ethtool_ops	= &dm9ks_ethtool_ops;
#ifdef CONFIG_NET_POLL_CONTROLLER
			dev->netdev_ops->ndo_poll_controller	= &dmfe_poll_controller;
#endif

#if 0
			dev->open 		= &dmfe_open;
			dev->hard_start_xmit 	= &dmfe_start_xmit;
			dev->stop 		= &dmfe_stop;
			dev->get_stats 		= &dmfe_get_stats;
			dev->set_multicast_list = &dmfe_hash_table;
			dev->do_ioctl 		= &dmfe_do_ioctl;
			dev->ethtool_ops	= &dm9ks_ethtool_ops;
#endif

#if defined(CHECKSUM)
			dev->features = dev->features | NETIF_F_NO_CSUM;
#endif
#ifdef	USE_E2ROM
			/* Read SROM content */
			for (i=0; i<64; i++)
				((u16 *)db->srom)[i] = read_srom_word(db, i);
			for (i=0; i<6; i++)
				dev->dev_addr[i] = db->srom[i];

			printk("MAC: %02x-%02x-%02x-%02x-%02x-%02x\n", 
					dev->dev_addr[0], dev->dev_addr[1], dev->dev_addr[2],
					dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5]);

#else
			/* We dont hv srom, just set the default mac address */
			/* Set Node Address */
			for (i=0; i<6; i++)
				dev->dev_addr[i] = def_mac[i];
#endif


		} //end of if ()
		iobase += 0x10;
	} while(!dm9000_found && (iobase & 0xfff) <= DM9KS_MAX_IO);

	return dm9000_found ? 0:-ENODEV;
} 

/*
  Open the interface.
  The interface is opened whenever "ifconfig" actives it.
*/
static int dmfe_open(struct net_device *dev)
{
	board_info_t *db = netdev_priv(dev);
	unsigned long irqflags = db->irq_res->flags & IRQF_TRIGGER_MASK;

	DMFE_DBUG(0, "dmfe_open", 0);
	
	if (irqflags == IRQF_TRIGGER_NONE) {
		dev_warn(db->dev, "WARNING: no IRQ resource flags set.\n");
		irqflags = 0;
	}
	
	irqflags |= IRQF_SHARED;

	if (request_irq(dev->irq, &dmfe_interrupt, irqflags, dev->name,dev)) 
		return -EAGAIN;

	/* Initilize DM910X board */
//	spin_lock_init(&db->lock);
//	spin_lock_init(&reg_lock);
	dmfe_init_dm9000(dev);
 
 	mdelay(200);
	/* Init driver variable */
	db->reset_counter 	= 0;

	/* set and active a timer process */
	init_timer(&db->timer);
	db->timer.expires 	= DMFE_TIMER_WUT * 2;
	db->timer.data 		= (unsigned long)dev;
	db->timer.function 	= &dmfe_timer;
	add_timer(&db->timer);	//Move to DM9000 initiallization was finished.
	
	netif_start_queue(dev);

	return 0;
} 

/* Set PHY operationg mode
*/
static void set_PHY_mode(board_info_t *db)
{
	u16 phy_reg0 = 0x1200;		/* Auto-negotiation & Restart Auto-negotiation */
	u16 phy_reg4 = 0x01e1;		/* Default flow control disable*/

	if ( !(db->op_mode & DM9KS_AUTO) ) // op_mode didn't auto sense */
	{ 
		switch(db->op_mode) {
			case DM9KS_10MHD:  phy_reg4 = 0x21; 
                        	           phy_reg0 = 0x1000;
					   break;
			case DM9KS_10MFD:  phy_reg4 = 0x41; 
					   phy_reg0 = 0x1100;
                                	   break;
			case DM9KS_100MHD: phy_reg4 = 0x81; 
					   phy_reg0 = 0x3000;
				    	   break;
			case DM9KS_100MFD: phy_reg4 = 0x101; 
					   phy_reg0 = 0x3100;
				   	   break;
			default: 
					   break;
		} // end of switch
	
	} // end of if
	phy_write(db, 0, phy_reg0);
	phy_write(db, 4, phy_reg4);
} 

/* 
	Initilize dm9000 board
*/
static void dmfe_init_dm9000(struct net_device *dev)
{
	board_info_t *db = netdev_priv(dev);
	DMFE_DBUG(0, "dmfe_init_dm9000()", 0);

	/* set the internal PHY power-on, GPIOs normal, and wait 2ms */
	iow(db, DM9KS_GPR, 0);	/* GPR (reg_1Fh)bit GPIO0=0 pre-activate PHY */
	udelay(20);		/* wait 2ms for PHY power-on ready */

	/* do a software reset and wait 20us */
	iow(db, DM9KS_NCR, 3);
	udelay(20);		/* wait 20us at least for software reset ok */
	iow(db, DM9KS_NCR, 3);	/* NCR (reg_00h) bit[0] RST=1 & Loopback=1, reset on. Added by SPenser */
	udelay(20);		/* wait 20us at least for software reset ok */

	/* I/O mode */
	db->io_mode = ior(db, DM9KS_ISR) >> 6; /* ISR bit7:6 keeps I/O mode */

	/* Set PHY */
	db->op_mode = media_mode;
	set_PHY_mode(db);

	/* Program operating register */
	iow(db, DM9KS_NCR, 0);
	iow(db, DM9KS_TCR, 0);		/* TX Polling clear */
	iow(db, DM9KS_BPTR, 0x3f);	/* Less 3kb, 600us */
	iow(db, DM9KS_SMCR, 0);		/* Special Mode */
	iow(db, DM9KS_NSR, 0x2c);	/* clear TX status */
	iow(db, DM9KS_ISR, 0x0f); 	/* Clear interrupt status */

	/* Added by jackal at 03/29/2004 */
#if defined(CHECKSUM)
	iow(db, DM9KS_TCCR, 0x07);	/* TX UDP/TCP/IP checksum enable */
	iow(db, DM9KS_RCSR, 0x02);	/*Receive checksum enable */
#endif

#if defined(ETRANS)
	iow(db, DM9KS_TCR2, 0x10);
	iow(db, DM9KS_ETXCSR, 0x83);
#endif
 
	/* Set address filter table */
	dmfe_hash_table(dev);

	/* Activate DM9000A/DM9010 */
	iow(db, DM9KS_RXCR, DM9KS_REG05 | 1);	/* RX enable */
	iow(db, DM9KS_IMR, DM9KS_REGFF); 	// Enable TX/RX interrupt mask
 
	/* Init Driver variable */
	db->tx_pkt_cnt 		= 0;
	dev->trans_start 	= 0;

	
	netif_carrier_on(dev);
} 

/*
  Hardware start transmission.
  Send a packet to media from the upper layer.
*/
static int dmfe_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	board_info_t *db = netdev_priv(dev);
	char * data_ptr;
	int i, tmplen;

	TD("T");
	if (db->tx_pkt_cnt >= 1) {
//		printk(KERN_ERR "%s: Device busy\n", __func__);
		TD("t");
		return 1;
	}
	
	if (skb->len > MAX_FRAME_SIZE) {
		printk(KERN_ERR "%s: packet to send too large, len = %d\n", __func__, skb->len);
		TD("t");
		return 1;
	}
		
	spin_lock(&db->lock);
	/* packet counting */
	db->tx_pkt_cnt++;

	db->stats.tx_packets++;
	db->stats.tx_bytes+=skb->len;

	if (db->tx_pkt_cnt >= 1)
		netif_stop_queue(dev);

	/* Disable all interrupt */
	iow(db, DM9KS_IMR, DM9KS_DISINTR);
	
	/* Set TX length to reg. 0xfc & 0xfd */
	iow(db, DM9KS_TXPLL, (skb->len & 0xff));
	iow(db, DM9KS_TXPLH, (skb->len >> 8) & 0xff);

	/* Move data to TX SRAM */
	data_ptr = (char *)skb->data;

	outb(DM9KS_MWCMD, db->io_addr); // Write data into SRAM trigger
	//db->sent_pkt_len = skb->len;

	switch(db->io_mode) {
		case DM9KS_BYTE_MODE:
			for (i = 0; i < skb->len; i++)
				outb((data_ptr[i] & 0xff), db->io_data);
			break;
		case DM9KS_WORD_MODE:
			tmplen = (skb->len + 1) / 2;
			for (i = 0; i < tmplen; i++)
         			outw(((u16 *)data_ptr)[i], db->io_data);
         		break;
         	case DM9KS_DWORD_MODE:
         		tmplen = (skb->len + 3) / 4;			
			for (i = 0; i< tmplen; i++)
				outl(((u32 *)data_ptr)[i], db->io_data);
			break;
	} 
	
#if !defined(ETRANS)
	/* Issue TX polling command */
	iow(db, DM9KS_TCR, 0x1); /* Cleared after TX complete*/
#endif

	/* Saved the time stamp */
	dev->trans_start = jiffies;

	/* Free this SKB */
	dev_kfree_skb(skb);

	/* Re-enable interrupt */
	iow(db, DM9KS_IMR, DM9KS_REGFF);

	spin_unlock(&db->lock);
	TD("t");
	return 0;
} 

/*
  Stop the interface.
  The interface is stopped when it is brought.
*/
static int dmfe_stop(struct net_device *dev)
{
	board_info_t *db = netdev_priv(dev);
	DMFE_DBUG(0, "dmfe_stop", 0);

	/* deleted timer */
	del_timer(&db->timer);

	netif_stop_queue(dev); 

	/* free interrupt */
	free_irq(dev->irq, dev);

	/* RESET devie */
	phy_write(db, 0x00, 0x8000);	/* PHY RESET */
	iow(db, DM9KS_GPR, 0x01); 	/* Power-Down PHY */
	iow(db, DM9KS_IMR, DM9KS_DISINTR);	/* Disable all interrupt */
	iow(db, DM9KS_RXCR, 0x00);	/* Disable RX */

	/* Dump Statistic counter */
#ifdef	DM9KS_DEBUG
	printk("\nRX FIFO OVERFLOW %ld\n", db->stats.rx_fifo_errors);
	printk("RX CRC %ld\n", db->stats.rx_crc_errors);
	printk("RX LEN Err %ld\n", db->stats.rx_length_errors);
	printk("RESET %d\n", db->reset_counter);
	printk("RESET: TX Timeout %d\n", db->reset_tx_timeout);
#endif

	return 0;
} 

static void dmfe_tx_done(unsigned long unused)
{
	struct net_device *dev = dmfe_dev;
	board_info_t *db = netdev_priv(dev);
	u8 nsr, tsr1 = 0, tsr2 = 0;
	int tx_done = 0;

	DMFE_DBUG(0, "dmfe_tx_done()", 0);
	
	nsr = ior(db, DM9KS_NSR);

	if (nsr & 0x04) {
		tx_done ++;
		tsr1 = ior(db, 0x03);		
// #ifdef	DM9KS_DEBUG
		if (tsr1)
			printk("%s: Tx error, TSR1 = %02x, NSR = %02x\n", __func__, tsr1, nsr);
// #endif
	}
	
	if (nsr & 0x08) {
		tx_done ++;
		tsr2 = ior(db, 0x04);		
// #ifdef	DM9KS_DEBUG
		if (tsr2)
			printk("%s: Tx error, TSR2 = %02x, NSR = %02x\n", __func__, tsr2, nsr);
// #endif
	}
	
	if (tx_done) {
#ifdef	DM9KS_DEBUG
		if (tx_done > 1)
			printk("2 Tx done in one time\n");
#endif
		if (!tsr1 && !tsr2)
			dev->trans_start = 0;
		db->tx_pkt_cnt--;
		netif_wake_queue(dev);
	} else {
		tsr1 = ior(db, 0x03);
		tsr2 = ior(db, 0x04);

// #ifdef	DM9KS_DEBUG
		printk("%s: Invalid NSR (%02x) while ISR indicate Tx complete, TSR1 = %02x, TSR2 = %02x, %d packets in queue\n",
			 __func__, nsr, tsr1, tsr2, db->tx_pkt_cnt);
// #endif
		
		if (db->tx_pkt_cnt) {
			db->tx_pkt_cnt = 0;	
			netif_wake_queue(dev);
			dev->trans_start = 0;
		}
	}

	return;
} 

/*
  DM9000 insterrupt handler
  receive the packet to upper layer, free the transmitted packet
*/
static irqreturn_t dmfe_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	board_info_t *db;
	int int_status;
	u8 reg_save;
//	unsigned long flags;

	TD("I");
	DMFE_DBUG(0, "dmfe_interrupt()", 0);

	/* A real interrupt coming */
	db = netdev_priv(dev);
//	spin_lock_irqsave(&db->lock, flags);
	spin_lock_irq(&db->lock);

	/* Save previous register address */
	reg_save = inb(db->io_addr);

	/* Disable all interrupt */
	iow(db, DM9KS_IMR, DM9KS_DISINTR); 

	/* Got DM9000A/DM9010 interrupt status */
	int_status = ior(db, DM9KS_ISR);		/* Got ISR */
	iow(db, DM9KS_ISR, int_status);		/* Clear ISR status */ 

	/* Received the coming packet */
	if (int_status & DM9KS_RX_INTR)
		dmfe_packet_receive(dev);

	/* Trnasmit Interrupt check */
	if (int_status & DM9KS_TX_INTR)
		tasklet_schedule(&dmfe_tx_tasklet);

	/* Re-enable interrupt mask */ 
	iow(db, DM9KS_IMR, DM9KS_REGFF);

	
	/* Restore previous register address */
	outb(reg_save, db->io_addr); 

//	spin_unlock_irqrestore(&db->lock, flags);
	spin_unlock_irq(&db->lock);

	TD("i");
	return IRQ_HANDLED;
} 

/*
  Get statistics from driver.
*/
static struct net_device_stats * dmfe_get_stats(struct net_device *dev)
{
	board_info_t *db = netdev_priv(dev);
	DMFE_DBUG(0, "dmfe_get_stats", 0);
	return &db->stats;
} 

/*
  Process the upper socket ioctl command
*/
static int dmfe_do_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	DMFE_DBUG(0, "dmfe_do_ioctl()", 0);
	return 0;
} 

/*
  A periodic timer routine
  Dynamic media sense, allocated Rx buffer...
*/
static void dmfe_timer(unsigned long data)
{
	struct net_device *dev = (struct net_device *)data;
	board_info_t *db = netdev_priv(dev);
	u8 reg_save;
	
	DMFE_DBUG(0, "dmfe_timer()", 0);

	/* Save previous register address */
	reg_save = inb(db->io_addr);

	/* TX timeout check */
	if (dev->trans_start && ((jiffies-dev->trans_start)>DMFE_TX_TIMEOUT)) {
#ifdef DM9KS_DEBUG
		printk("DM9K: %s() reset caused by TX Timeout, trans_start = %lu, %lu jiffies passed\n", 
			__func__, dev->trans_start, jiffies - dev->trans_start);
#endif
		db->device_wait_reset = 1;
		db->reset_tx_timeout++;
	} 

	/* DM9000A/DM9010 dynamic RESET check and do */
	if (db->device_wait_reset) {
#ifdef DM9KS_DEBUG
		printk("DM9K: %s() reset...", __func__);
#endif
		spin_lock(&db->lock);
		netif_stop_queue(dev); 
		db->reset_counter++;
		db->device_wait_reset = 0;
		dmfe_init_dm9000(dev);
		netif_wake_queue(dev);
		spin_unlock(&db->lock);
#ifdef DM9KS_DEBUG
		printk("Done\n");
#endif
	} 

#ifdef DM9KS_DEBUG
	{
		u16 bmsr = phy_read(db, 1);
		if (!(bmsr & 1<<2))
			printk("link failed, BMSR = %04x\n", bmsr);

		if (bmsr & 1<<1)
			printk("jabber detected, BMSR = %04x\n", bmsr);
	}
#endif
		
	/* Restore previous register address */
	outb(reg_save, db->io_addr);

	/* Set timer again */
	db->timer.expires = DMFE_TIMER_WUT;
	add_timer(&db->timer);
} 

#if !defined(CHECKSUM)
#define check_rx_ready(a)	((a) == 0x01)
#else
inline u8 check_rx_ready(u8 rxbyte)
{
	if (!(rxbyte & 0x01))
		return 0;
	return ((rxbyte >> 4) | 0x01);
} 
#endif

/*
  Received a packet and pass to upper layer
*/
static void dmfe_packet_receive(struct net_device *dev)
{
	board_info_t *db = netdev_priv(dev);
	struct sk_buff *skb;
	u8 rxbyte, val;
	u16 i, GoodPacket, tmplen = 0, MDRAH, MDRAL;
	u32 tmpdata;
	int pkt_received = 0;

	rx_t rx;

	u16 * ptr = (u16*)&rx;
	u8* rdptr;

	TD("R");
	DMFE_DBUG(0, "dmfe_packet_receive()", 0);

	do {
		/*store the value of Memory Data Read address register*/
		MDRAH=ior(db, DM9KS_MDRAH);
		MDRAL=ior(db, DM9KS_MDRAL);
		
		ior(db, DM9KS_MRCMDX);		/* Dummy read */
		rxbyte = inb(db->io_data);	/* Got most updated data */

		/* packet ready to receive check */
		if (!(val = check_rx_ready(rxbyte))) break;

		pkt_received++;
		/* A packet ready now  & Get status/length */
		GoodPacket = TRUE;
		outb(DM9KS_MRCMD, db->io_addr);

		/* Read packet status & length */
		switch (db->io_mode) 
			{
			  case DM9KS_BYTE_MODE: 
 				    *ptr	= inb(db->io_data) + (inb(db->io_data) << 8);
				    *(ptr+1)	= inb(db->io_data) + (inb(db->io_data) << 8);
				    break;
			  case DM9KS_WORD_MODE:
				    *ptr	= inw(db->io_data);
				    *(ptr+1)    = inw(db->io_data);
				    break;
			  case DM9KS_DWORD_MODE:
				    tmpdata	= inl(db->io_data);
				    *ptr	= tmpdata;
				    *(ptr+1)    = tmpdata >> 16;
				    break;
			  default:
				    break;
			} 

		/* Packet status check */
		if (rx.desc.status & 0xbf) {
			GoodPacket = FALSE;
			if (rx.desc.status & 0x01)
				db->stats.rx_fifo_errors++;
			if (rx.desc.status & 0x02)
				db->stats.rx_crc_errors++;
			if (rx.desc.status & 0x80)
				db->stats.rx_length_errors++;
#ifdef DM9KS_DEBUG
			{ 
				char *msg = "";
				if (rx.desc.status & 0x01)	msg = "<RX FIFO error>";
				if (rx.desc.status & 0x02)	msg = "<RX CRC error>";
				if (rx.desc.status & 0x80)	msg = "<RX Length error>";
				if (rx.desc.status & 0x08)	msg = "<Physical Layer error>";
				printk(KERN_DEBUG "%s: Rx error, status = %02x, %s ", __func__,  rx.desc.status, msg);
			}
#endif
		} 

		if (rx.desc.length > MAX_FRAME_SIZE) {
			db->stats.rx_length_errors++;
			printk(KERN_DEBUG "%s: Warning: large packet received, size = %d\n", __FILE__, rx.desc.length);
		}

		if (!GoodPacket) {
			// drop this packet!!!
			switch (db->io_mode) {
				case DM9KS_BYTE_MODE:
			 		for (i=0; i<rx.desc.length; i++)
						inb(db->io_data);
					break;
				case DM9KS_WORD_MODE:
					tmplen = (rx.desc.length + 1) / 2;
					for (i = 0; i < tmplen; i++)
						inw(db->io_data);
					break;
				case DM9KS_DWORD_MODE:
					tmplen = (rx.desc.length + 3) / 4;
					for (i = 0; i < tmplen; i++)
						inl(db->io_data);
					break;
			} 
			continue;/*next the packet*/
		} 
		
		skb = dev_alloc_skb(rx.desc.length+4);
		if (skb == NULL ) {	
			printk(KERN_INFO "%s: Memory squeeze.\n", dev->name);
			/*re-load the value into Memory data read address register*/
			iow(db,DM9KS_MDRAH,MDRAH);
			iow(db,DM9KS_MDRAL,MDRAL);
			return;
		} else {
			/* Move data from DM9000 */
			skb->dev = dev;
			skb_reserve(skb, 2);
			rdptr = (u8*)skb_put(skb, rx.desc.length - 4);
			
			/* Read received packet from RX SARM */
			switch (db->io_mode) {
				case DM9KS_BYTE_MODE:
			 		for (i=0; i<rx.desc.length; i++)
						rdptr[i]=inb(db->io_data);
					break;
				case DM9KS_WORD_MODE:
					tmplen = (rx.desc.length + 1) / 2;
					for (i = 0; i < tmplen; i++)
						((u16 *)rdptr)[i] = inw(db->io_data);
					break;
				case DM9KS_DWORD_MODE:
					tmplen = (rx.desc.length + 3) / 4;
					for (i = 0; i < tmplen; i++)
						((u32 *)rdptr)[i] = inl(db->io_data);
					break;
			} 
		
			/* Pass to upper layer */
			skb->protocol = eth_type_trans(skb,dev);
#if defined(CHECKSUM)
			if (val == 0x01)
				skb->ip_summed = CHECKSUM_UNNECESSARY;
#endif
			netif_rx(skb);
			db->stats.rx_packets++;
			db->stats.rx_bytes += rx.desc.length;
		} 
		if (pkt_received > 5)
			printk("%s: %d packets received\n", __func__, pkt_received);
	} while((rxbyte & 0x01) == DM9KS_PKT_RDY);
	
	TD("r");
} 

/*
  Read a word data from SROM
*/
#ifdef	USE_E2ROM
static u16 read_srom_word(board_info_t *db, int offset)
{
	iow(db, DM9KS_EPAR, offset);
	iow(db, DM9KS_EPCR, 0x4);
//	udelay(200);
	mdelay(2);
	iow(db, DM9KS_EPCR, 0x0);
	return (ior(db, DM9KS_EPDRL) + (ior(db, DM9KS_EPDRH) << 8) );
}
#endif

/*
  Set DM9000A/DM9010 multicast address
*/
static void dmfe_hash_table(struct net_device *dev)
{
	board_info_t *db = netdev_priv(dev);
	struct dev_mc_list *mcptr = dev->mc_list;
	int mc_cnt = dev->mc_count;
	u32 hash_val;
	u16 i, oft, hash_table[4];

	DMFE_DBUG(0, "dmfe_hash_table()", 0);

	/* Set Node address */
	for (i = 0, oft = 0x10; i < 6; i++, oft++)
		iow(db, oft, dev->dev_addr[i]);

	/* Clear Hash Table */
	for (i = 0; i < 4; i++)
		hash_table[i] = 0x0;

	/* broadcast address */
	hash_table[3] = 0x8000;

	/* the multicast address in Hash Table : 64 bits */
	for (i = 0; i < mc_cnt; i++, mcptr = mcptr->next) {
		hash_val = cal_CRC((char *)mcptr->dmi_addr, 6, 0) & 0x3f; 
		hash_table[hash_val / 16] |= (u16) 1 << (hash_val % 16);
	} 

	/* Write the hash table to MAC MD table */
	for (i = 0, oft = 0x16; i < 4; i++) {
		iow(db, oft++, hash_table[i] & 0xff);
		iow(db, oft++, (hash_table[i] >> 8) & 0xff);
	} 
} 

/*
  Calculate the CRC valude of the Rx packet
  flag = 1 : return the reverse CRC (for the received packet CRC)
         0 : return the normal CRC (for Hash Table index)
*/
static unsigned long cal_CRC(unsigned char * Data, unsigned int Len, u8 flag)
{
	
	u32 crc = ether_crc_le(Len, Data);

	if (flag) 
		return ~crc;
		
	return crc;
	 
} 

/*
   Read a byte from I/O port
*/
static u8 ior(board_info_t *db, int reg)
{
	u8 val;
	unsigned long flags;
	spin_lock_irqsave(&reg_lock, flags);
	outb(reg, db->io_addr);
	udelay(1);
	val = inb(db->io_data);
	udelay(1);
	spin_unlock_irqrestore(&reg_lock, flags);
	return val;
} 

/*
   Write a byte to I/O port
*/
static void iow(board_info_t *db, int reg, u8 value)
{
	unsigned long flags;
	spin_lock_irqsave(&reg_lock, flags);
	outb(reg, db->io_addr);
	udelay(1);
	outb(value, db->io_data);
	udelay(1);
	spin_unlock_irqrestore(&reg_lock, flags);
} 

/*
   Read a word from phyxcer
*/
static u16 phy_read(board_info_t *db, int reg)
{
	/* Fill the phyxcer register into REG_0C */
	iow(db, DM9KS_EPAR, DM9KS_PHY | reg);

	iow(db, DM9KS_EPCR, 0xc); 	/* Issue phyxcer read command */
	udelay(100);			/* Wait read complete */
	iow(db, DM9KS_EPCR, 0x0); 	/* Clear phyxcer read command */

	/* The read data keeps on REG_0D & REG_0E */
	return ( ior(db, DM9KS_EPDRH) << 8 ) | ior(db, DM9KS_EPDRL);
} 

/*
   Write a word to phyxcer
*/
static void phy_write(board_info_t *db, int reg, u16 value)
{
	/* Fill the phyxcer register into REG_0C */
	iow(db, DM9KS_EPAR, DM9KS_PHY | reg);

	/* Fill the written data into REG_0D & REG_0E */
	iow(db, DM9KS_EPDRL, (value & 0xff));
	iow(db, DM9KS_EPDRH, ( (value >> 8) & 0xff));

	iow(db, DM9KS_EPCR, 0xa);	/* Issue phyxcer write command */
	udelay(500);			/* Wait write complete */
	iow(db, DM9KS_EPCR, 0x0);	/* Clear phyxcer write command */
} 

static int dm9000_drv_probe(struct platform_device *dev)
{
	struct resource	*addr_res = platform_get_resource(dev, IORESOURCE_MEM, 0);
	struct resource *data_res = platform_get_resource(dev, IORESOURCE_MEM, 1);
	struct resource *irq_res  = platform_get_resource(dev, IORESOURCE_IRQ, 0);
	board_info_t *db;

// nickmit
	
	printk(KERN_INFO "%s Ethernet Driver, V%s\n", CARDNAME, DRV_VERSION);
	if (!addr_res || !data_res || !irq_res)
		return -ENODEV;

	
#if 0
	ret = dm9000_mach_init();
	if (ret)
		return ret;
#endif
				
	iobase	= (u32) ioremap(addr_res->start, 1);
	iodata	= (u32) ioremap(data_res->start, 1);
	irq	= irq_res->start;
	printk(KERN_INFO "\tIO = %08x/%03x, IRQ = %d\n",
		addr_res->start, data_res->start & 0xfff, irq);
//	set_irq_type(irq, IRQT_RISING);
//	set_irq_type(irq, IRQT_HIGH);

	dmfe_dev = dmfe_probe1(dev);
	if (IS_ERR(dmfe_dev)) {
		iounmap((void __iomem *)iobase);
		iounmap((void __iomem *)iodata);
		return PTR_ERR(dmfe_dev);
	} 

	db = (board_info_t *) netdev_priv(dmfe_dev);
	db->irq_res = irq_res;
	dev_set_drvdata(&dev->dev, dmfe_dev);

	return 0;
} 

static int dm9000_drv_suspend(struct platform_device *dev, pm_message_t state)
{
	struct net_device *ndev = dev_get_drvdata(&dev->dev);
	board_info_t *db = netdev_priv(ndev);

	if (ndev) {
		if (netif_running(ndev)) {
			netif_device_detach(ndev);
			/* RESET devie */
			phy_write(db, 0x00, 0x8000);	/* PHY RESET */
			iow(db, DM9KS_GPR, 0x01); 	/* Power-Down PHY */
			iow(db, DM9KS_IMR, DM9KS_DISINTR);	/* Disable all interrupt */
			iow(db, DM9KS_RXCR, 0x00);	/* Disable RX */
		} 
	} 
	return 0;
} 

static int dm9000_drv_resume(struct platform_device *dev)
{
	struct net_device *ndev = dev_get_drvdata(&dev->dev);

	if (ndev) {
		if (netif_running(ndev)) {
			dmfe_init_dm9000(ndev);
			netif_device_attach(ndev);
		} 
	} 
	return 0;
} 

static int dm9000_drv_remove(struct platform_device *dev)
{
	struct net_device *ndev = dev_get_drvdata(&dev->dev);
	board_info_t *db = netdev_priv(ndev);

	dev_set_drvdata(&dev->dev, NULL);

	unregister_netdev(ndev);

	iounmap((void __iomem *)db->io_addr);
	iounmap((void __iomem *)db->io_data);

	kfree(ndev);		/* free device structure */

	printk(KERN_DEBUG "clean_module() exit\n");

	return 0;
} 
static struct platform_driver dm9000_driver = {
	.driver = {
		.name	= "dm9000",
		.owner	= THIS_MODULE,
	},
	.probe   = dm9000_drv_probe,
	.remove  = dm9000_drv_remove,
	.suspend = dm9000_drv_suspend,
	.resume  = dm9000_drv_resume,
} ;


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bill, bill_wu@davicom.com.tw");
MODULE_DESCRIPTION("Davicom DM9000A/DM9010 ISA/uP Fast Ethernet Driver");
module_param(mode, int, 0400);
module_param(irq, int, 0400);
module_param(iobase, int, 0400);
MODULE_PARM_DESC(mode,"Media Speed, 0:10MHD, 1:10MFD, 4:100MHD, 5:100MFD");
MODULE_PARM_DESC(irq,"EtherLink IRQ number");
MODULE_PARM_DESC(iobase, "EtherLink I/O base address");

/* Description: 
   when user used insmod to add module, system invoked init_module()
   to initilize and register.
*/
static int __init dm9000_init(void)
{
	media_mode = mode;
	return platform_driver_register(&dm9000_driver);	/* search board and register */
} 

static void __exit dm9000_cleanup(void)
{
	platform_driver_unregister(&dm9000_driver);
} 

module_init(dm9000_init);
module_exit(dm9000_cleanup);
