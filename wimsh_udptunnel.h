/* 
 *  Copyright (C) 2007 Dip. Ing. dell'Informazione, University of Pisa, Italy
 *  http://info.iet.unipi.it/~cng/ns2mesh80216/
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA, USA
 */

#ifndef __NS2_WIMSH_UDPTUNNEL_H
#define __NS2_WIMSH_UDPTUNNEL_H

#include "agent.h"
#include "trafgen.h"
#include "packet.h"

class WimshUdpTunnel : public Agent {
protected:
	// Odd message types are reserved for the sponsor node, even for the BS.
	enum MsgType {
			AUTH_REQ = 0,
			AUTH_RSP,
			REG_REQ,
			REQ_RSP,
			DHCP_DISCOVER,
			DHCP_OFFER,
			DHCP_REQ,
			DHCP_ACK,
			TIME_REQ,
			TIME_RSP,
			TFTP_RRQ,
			TFTP_DATA,
			TFTP_CPLT,
			TFTP_RSP
	};
	enum TunnelType {
			SPONSOR,
			BS
	};
public:
	WimshUdpTunnel();
	WimshUdpTunnel(packet_t);
	virtual void sendmsg(int nbytes, const char *flags = 0)
	{
		sendmsg(nbytes, NULL, flags);
	}
	virtual void sendmsg(int nbytes, AppData* data, const char *flags = 0);
	virtual void recv(Packet* pkt, Handler*);
	virtual int command(int argc, const char*const* argv);
protected:
	TunnelType tunnelType_;
	int seqno_;
	static unsigned int nagents_;

	unsigned int size (MsgType type);
};

#endif
