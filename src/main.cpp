/*
BSD 2-Clause License

Copyright (c) 2018, Paul Topley
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <iostream>
#include <string>
#include <rte_eal.h>
#include <rte_ethdev.h>
#include <rte_lcore.h>

int main( int argc, char *argv[] )
{
	// DPDK Init

	char* params[32] ;
	int   params_count = 0 ;

	params[params_count++] = (char*) "pktpmp" ;
	params[params_count++] = (char*) "-c" ;

	// Hardcoded to Quad Core for now
	uint32_t coreMask = 0;

	for (unsigned coreId = 0; coreId < 4; coreId++)
	{
		coreMask |= (1 << coreId);
	}

	char coreMaskHex[5];
	snprintf( coreMaskHex, 5, "%x", coreMask ) ;

	params[params_count++] = coreMaskHex ;

	params[params_count++] = (char*) "-n" ;
	params[params_count++] = (char*) "2" ;

	// Init DPDK
	if ( rte_eal_init( params_count, params ) )
	{
		std::cout << "Failed DPDK init" << std::endl ;

		return -1 ;
	}

	// Init mempool

	struct rte_mempool * mempool = rte_pktmbuf_pool_create( "pktpmp_pool", 65000, 256, 0, 1500, rte_socket_id() ) ;

	if ( mempool == NULL )
	{
		std::cout << "Failed to create mempool" << std::endl ;

		return -1 ;
	}

	// Init ports, hard coded to 2 to begin with

	int nPortCount = 2 ; //rte_eth_dev_count() ;

	for ( int port = 0; port < nPortCount; port++ ){
	{
		if ( rte_eth_dev_is_valid_port(port) != 1 )
		{
			std::cout << "Invalid physical port " << port << std::endl ;
			return -1 ;
		}

		int ret = 0 ;
		rte_eth_dev_info devInfo;
		rte_eth_conf     port_conf;

		memset(&devInfo, 0, sizeof(devInfo));
		rte_eth_dev_info_get(port, &devInfo);

		if (devInfo.pci_dev->id.vendor_id == 0 || devInfo.pci_dev->id.device_id == 0)
		{
			std::cout << "Failed to get dev info for port " << port << std::endl ;

			return -1 ;
		}

		memset( &port_conf, 0, sizeof(struct rte_eth_conf) ) ;

		ret = rte_eth_dev_configure( port, 1, 1, &port_conf ) ;

		if ( ret < 0 )
		{
			std::cout << "Cannot configure device: err=" << ret << "(" << rte_strerror(ret) << ") port " << port << std::endl ;

			return -1 ;
		}

		rte_eth_rxconf rxConf ;
		rte_eth_txconf txConf ;

		// Get Defaults for rx and tx confs
		memcpy(&rxConf, &devInfo.default_rxconf, sizeof(rxConf));
		memcpy(&txConf, &devInfo.default_txconf, sizeof(txConf));

		rxConf.rx_drop_en = 0;
		txConf.txq_flags = ETH_TXQ_FLAGS_NOOFFLOADS | ETH_TXQ_FLAGS_NOMULTSEGS | ETH_TXQ_FLAGS_NOMULTMEMP | ETH_TXQ_FLAGS_NOREFCOUNT ;

		uint16_t rx_descriptors = 2048;
		uint16_t tx_descriptors = 4096;

		// Init RX queue
		ret = rte_eth_rx_queue_setup( ( uint8_t ) port, 0, rx_descriptors, rte_eth_dev_socket_id(port), &rxConf, mempool);

		if ( ret < 0 )
		{
			std::cout << "RX hardware queue setup failed : err=" << ret << " (" << rte_strerror(ret) << ") port " << port << std::endl ;

			return -1 ;
		}

		// Init TX queue

		ret = rte_eth_tx_queue_setup( ( uint8_t ) port, 0, tx_descriptors, rte_eth_dev_socket_id(port), &txConf);

		if ( ret < 0 )
		{
			std::cout << "TX hardware queue setup failed : err=" << ret << " (" << rte_strerror(ret) << ") port " << port << std::endl ;

			return -1 ;
		}

		rte_eth_promiscuous_enable( ( uint8_t ) port );
		rte_eth_allmulticast_enable( ( uint8_t ) port );

		rte_eth_fc_conf fc_conf;

		memset(&fc_conf, 0, sizeof(fc_conf));

		fc_conf.mode       = RTE_FC_NONE ;
		fc_conf.high_water = 0 ;
		fc_conf.low_water  = 0 ;
		fc_conf.pause_time = 1 ;
		fc_conf.send_xon   = 0 ;

		rte_eth_fc_conf fc;

		if (rte_eth_dev_flow_ctrl_get(port, &fc) != 0 || fc.mode != RTE_FC_NONE)
		{
			uint8_t retries;

			for (retries = 0; retries < 4; retries++)
			{
				ret = rte_eth_dev_flow_ctrl_set( ( uint8_t ) port, &fc_conf );

				if (ret == 0 || ret == -EOPNOTSUPP)
				{
					break;
				}
				else if (ret == -ENOSYS)
				{
					std::cout << "Failed to set flow control for port " << port << " (ret " << ret << ")" << std::endl ;
				}
				else if (ret != 0)
				{
					std::cout << "Failed to set flow control for port " << port << " (ret " << ret << ")" << std::endl ;
				}

				sleep(500);
			}

			switch(ret)
			{
				// success
				case 0:
				// not supported
				case -EOPNOTSUPP:
				break;

				// the i40e (x710 driver) often returns ENOSYS (-38) which is due to an undefined error returned from the card firmware
				case -ENOSYS:
				default:
					std::cout << "Failed to set flow control for port " << port << " (ret " << ret << ") Ignoring" << std::endl ;
					return -1;
			}
		}

		if (rte_eth_dev_get_vlan_offload(port) != 0)
		{
			/* Disable all VLAN functionality (strip, offload, etc.) */
			ret = rte_eth_dev_set_vlan_offload ( ( uint8_t ) port, 0x0 ) ;

			if (ret != 0)
			{
				std::cout << "Set VLAN offload failed: err " << ret << " (" << rte_strerror(ret) << ") port " << port << std::endl ;

				return -1 ;
			}

			std::cout << "VLAN offload successfully disabled for port " << port << std::endl ;
		}

		/* Start device */
		ret = rte_eth_dev_start( ( uint8_t ) port );

		if ( ret < 0 )
		{
			std::cout << "Port start failed: err " << ret << "(" << rte_strerror(ret) << ") port " << port << std::endl ;

			return -1 ;
		}
	}

	PortConfig portConfigs[2] ;

	portConfigs[0].Init( 0 ) ;
	portConfigs[1].Init( 1 ) ;


	// Init Threads

	// check state of each core
	int lcore_id;

	RTE_LCORE_FOREACH_SLAVE(lcore_id)
	{
		if (lcore_config[lcore_id].state != WAIT)
			return -1 ;
	}


	// Core Config, assume 4 core initially
	// Core 0: Non essential, debug output command process etc
	// Core 0 is the master core, so we will run non essential on it.

	for ( int i = 1; i < 4; i++ )
	{
		switch( i )
		{
			case 1: // RX core
			{
				int ret = rte_eal_remote_launch( rx_thread, (void*) &portConfigs, 1 ) ;

				if ( ret != 0 )
				{
					return -1 ;
				}

				break ;
			}

			case 2: // TX core
			{
				int ret = rte_eal_remote_launch( tx_thread, (void*) &portConfigs, 1 ) ;

				if ( ret != 0 )
				{
					return -1 ;
				}

			}
				break ;
			}

			case 3: // Worker core
			{
				int ret = rte_eal_remote_launch( worker_thread, (void*) &portConfigs, 1 ) ;

				if ( ret != 0 )
				{
					return -1 ;
				}

				break ;
			}

			default:
			{
				break ;
			}
		}
	}

	gp_thread(NULL) ;

	// Master Core loop returned, wait on slaves to cleanly shutdown.

	RTE_LCORE_FOREACH_SLAVE(lcore_id) {
		if (rte_eal_wait_lcore(lcore_id) < 0)
			return -1;
	}
	return 0 ;

}
