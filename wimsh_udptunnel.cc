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

#include "wimsh_udptunnel.h"
#include "rtp.h"
#include "random.h"
#include "address.h"
#include "ip.h"
#include "stat.h"

unsigned int WimshUdpTunnel::nagents_ = 0;

static class WimshUdpTunnelClass : public TclClass {
public:
	WimshUdpTunnelClass() : TclClass("Agent/WimshUdpTunnel") {}
	TclObject* create(int, const char*const*) {
		return (new WimshUdpTunnel());
	}
} class_wimsh_udp_tunnel_agent;

WimshUdpTunnel::WimshUdpTunnel() : Agent(PT_UDP), seqno_(-1)
{
	bind("packetSize_", &size_);
	tunnelType_ = SPONSOR;
}

WimshUdpTunnel::WimshUdpTunnel(packet_t type) : Agent(type)
{
	bind("packetSize_", &size_);
	tunnelType_ = SPONSOR;
}

// put in timestamp and sequence number, even though UDP doesn't usually 
// have one.
void WimshUdpTunnel::sendmsg(int nbytes, AppData* data, const char* flags)
{
	if ( debug_ ) {
		fprintf (stderr, "%.9f sendmsg flow id %d\n", NOW, fid_);
	}

	Packet *p;

	p = allocpkt();
	hdr_rtp* rh = hdr_rtp::access(p);

	rh->flags() = (u_int16_t) AUTH_REQ;
	hdr_cmn::access(p)->size() = size ((MsgType)rh->flags());
	rh->seqno() = ++seqno_;
	hdr_cmn::access(p)->timestamp() = NOW;
	p->setdata(data);
	target_->recv(p);
	idle();
}

unsigned int WimshUdpTunnel::size (MsgType msgType)
{
	unsigned int s;
	switch (msgType) {
		case 0: s = 30; break;
		case 1: s = 30; break;
		case 2: s = 30; break;
		case 3: s = 30; break;
		case 4: s = 30; break;
		case 5: s = 30; break;
		case 6: s = 30; break;
		case 7: s = 30; break;
		case 8: s = 30; break;
		case 9: s = 30; break;
		case 10: s = 30; break;
		case 11: s = 30; break;
		case 12: s = 30; break;
		case 13: s = 30; break;
		default: abort();
	};
	return s;
}

void WimshUdpTunnel::recv(Packet* pkt, Handler*)
{
	if ( debug_ ) {
		fprintf (stderr, "%.9f Received packet type %d flow id %d "
				"timestamp %.9f\n",
				NOW, hdr_rtp::access(pkt)->flags(), fid_,
				hdr_cmn::access(pkt)->timestamp());
	}

	if ( hdr_rtp::access(pkt)->flags() == TFTP_RSP ) {
		Stat::put ("udp_tunnel_delay", fid_,
				NOW - hdr_cmn::access(pkt)->timestamp());
		nagents_--;
		if ( nagents_ > 0 ) return;
		char** argv = new char*[1];
	  	argv[0] = "print";
		Stat::command (1, argv);
		exit (0);
	}

	Packet *p;

	p = allocpkt();
	hdr_rtp* rh = hdr_rtp::access(p);

	rh->flags() = (u_int16_t) hdr_rtp::access(pkt)->flags() + 1;
	hdr_cmn::access(p)->size() = size ((MsgType)rh->flags());
	hdr_cmn::access(p)->timestamp() = hdr_cmn::access(pkt)->timestamp();
	rh->seqno() = ++seqno_;
	p->setdata(0);
	target_->recv(p);
	Packet::free (pkt);
}


int WimshUdpTunnel::command(int argc, const char*const* argv)
{
	if ( argc == 3 && strcmp (argv[1], "set-node") == 0 ) {
		if ( strcmp (argv[2], "bs") == 0 ) {
			tunnelType_ = BS;
		} else if ( strcmp (argv[2], "sponsor") == 0 ) {
			tunnelType_ = SPONSOR;
		} else {
			fprintf (stderr, "Unknown node type '%s'\n", argv[1]);
			return TCL_ERROR;
		}
		return TCL_OK;
	} else if ( argc == 2 && strcmp (argv[1], "start") == 0 ) {
		nagents_++;
		sendmsg (0, 0, 0);
		return TCL_OK;
	}
	return (Agent::command(argc, argv));
}
