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

#ifndef __NS2_WIMSH_FORWARDING_H
#define __NS2_WIMSH_FORWARDING_H

#include <wimax_common.h>
#include <wimsh_packet.h>

class WimaxSdu;
class WimshMac;
class WimshTopology;

/*
 *
 * class WimshForwarding
 *
 */

//! Abstract class for the forwarding module in 802.16.
class WimshForwarding {
protected:
	//! Pointer to the MAC layer.
	WimshMac* mac_;
	//! Pointer to the topology.
	WimshTopology* topology_;

public:
	//! Create the Forwarding object.
	WimshForwarding (WimshMac* m, WimshTopology* t) :
		mac_ (m), topology_ (t) { }
	//! Do nothing.
	virtual ~WimshForwarding () { }
	
	//! Get the next-hop of an SDU.
	virtual WimaxNodeId nextHop (WimaxSdu* sdu) = 0;

	//! Tcl interface.
	virtual int command(int argc, const char*const* argv) = 0;

	//! Initialize the data structures
	virtual void initialize () = 0;
	
	//! Receive the control message
	virtual void recvMshDsch (WimshMshDsch* dsch) = 0;

	//! The burst profile of a link has changed.
	virtual void profile () = 0;
};

/*
 *
 * class WimshForwardingSpf
 *
 */

//! Implement the shortest-path-first algorithm.
class WimshForwardingSpf : public WimshForwarding {

public:
	//! Create the object
	WimshForwardingSpf( WimshMac* m, WimshTopology* t ) : 
	  WimshForwarding (m, t) { }
    //! Do nothing
    ~WimshForwardingSpf () { };
	//! Get the next-hop of an SDU, based on the shortest-path policy.
	WimaxNodeId nextHop (WimaxSdu* sdu);
	//! Tcl interface.
	int command (int argc, const char*const* argv) { return TCL_OK; }
	//! Do nothing
	void initialize () { }
	//! Do nothing
	void recvMshDsch (WimshMshDsch* dsch) { }
	//! Do nothing
	void profile() { }
};

#endif // __NS2_WIMSH_FORWARDING_H
