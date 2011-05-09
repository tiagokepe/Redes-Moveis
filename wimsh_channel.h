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

#ifndef __NS2_WIMSH_CHANNEL_H
#define __NS2_WIMSH_CHANNEL_H

#include <map>
#include <deque>

#include <wimax_common.h>
#include <t_timers.h>

#include <scheduler.h>
#include <rng.h>

class WimshPhy;
class WimshBurst;
class WimshTopologySimple;

//! Physical channel.
/*!
  Each burst is delayed of the propagation time before being dispatched
  to all the PHYs that are currently listening to this channel.

  Note that the channel does not enforce that a PHY keeps listening to
  this channel for the whole burst duration.
  */
class WimshChannel : public TclObject {
public:
	//! Map of PHYs connected to this channel (tx, rx, or not connected).
	std::map<WimshPhy*, wimax::ChannelStatus> phymap_;

private:
	//! Pointer to the topology. :XXX: this is not used right now.
	WimshTopologySimple* topology_;

	//! Timer to delay bursts of a propagation time.
	TMultiTimer<WimshChannel, WimshBurst*> timer_;

	//! Fixed propagation delay, in seconds. Initialized via tcl command.
	double propagation_;

	//! Uniform error rate for DATA bursts.
	double errorData_;

	//! Uniform error rate for CONTROL bursts.
	double errorCtrl_;

	//! Random number generator for DATA errors.
	RNG* rngErrorData_;

	//! Random number generator for CONTROL errors.
	RNG* rngErrorCtrl_;

	//! Channel identifier (used for statistics collection).
	unsigned int uid_;

public:
	//! Do nothing.
	WimshChannel ();
	//! Do nothing.
	virtual ~WimshChannel () { }

	//! Set the specified mode for a given PHY.
	void setMode (WimshPhy* phy, wimax::ChannelStatus s) { phymap_[phy] = s; }
	//! Get the specified mode for a given PHY.
	wimax::ChannelStatus getMode (WimshPhy* phy) { return phymap_[phy]; }
	//! Receive a PDU burst from a PHY. Start the propagation timer.
	void recvBurst (WimshBurst* burst);
	//! Handle the propagation timer: dispatch burst.
	void handle (WimshBurst* burst);

	//! Return a channel numerical identifier.
	unsigned int uid () { return uid_; }

protected:
	//! Tcl interface.
	virtual int command(int argc, const char*const* argv);

private:
	WimshChannel (const WimshChannel&);
	void operator= (const WimshChannel&);
};

#endif // __NS2_WIMSH_CHANNEL_H
