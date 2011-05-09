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

#ifndef __NS2_WIMSH_PHY_H
#define __NS2_WIMSH_PHY_H

#include <wimax_common.h>
#include <t_timers.h>

#include <list>

class WimshChannel;
class WimshBurst;
class WimshMac;
class WimshTopologySimple;

//! Physical layer management information base.
class WimshPhyMib : public TclObject {
public:
	//! Uncoded block size (in bytes) for each burst profile.
	static const unsigned int alpha[6];

private:
	//! OFDM symbol duration (in seconds). Set via Tcl command.
	double symDuration_;
	//! Frame duration (in seconds). Set via Tcl command.
	double frameDuration_;
	//! Number of OFDM symbols per frame.
	unsigned int symPerFrame_;
	//! Number of control slots per frame.
	unsigned int controlSlots_;
	//! Number of minislots per frame.
	unsigned int slotPerFrame_;
	//! Number of OFDM symbols per minislot.
	unsigned int symPerSlot_;
	//! Number of scheduling frames between two consecutive configuration ones.
	unsigned int cfgInterval_;

public:
	//! Build an empty PHY.
	WimshPhyMib ();
	//! Do nothing.
	virtual ~WimshPhyMib () { }

	//! Tcl interface.
	virtual int command(int argc, const char*const* argv);

	//! Return the frame duration (in seconds).
	double frameDuration () { return frameDuration_; }
	//! Return the OFDM symbol duration (in seconds).
	double symDuration () { return symDuration_; }
	//! Return the data slot duration.
	double slotDuration () { return symPerSlot_ * symDuration_; }
	//! Return the control slot duration.
	double controlSlotDuration () { return 7 * symDuration_; }

	//! Return the start time of the next frame (in seconds).
	double nextFrame ();

	//! Return the number of OFDM symbols per frame.
	unsigned int symPerFrame () { return symPerFrame_; }
	//! Return the number of OFDM symbols per minislot.
	unsigned int symPerSlot () { return symPerSlot_; }
	//! Return the number of minislots per frame.
	unsigned int slotPerFrame () { return slotPerFrame_; }
	//! Return the number of control slots per frame.
	unsigned int controlSlots () { return controlSlots_; }
	//! Return the configuration frames interval.
	unsigned int cfgInterval () { return cfgInterval_; }

	//! Return the number of OFDM symbols of a short preamble, for data PDUs.
	unsigned int symShortPreamble () { return 1; }
	//! Return the number of OFDM symbols of a long preamble, for control PDUs.
	unsigned int symLongPreamble () { return 2; }

	//! Return the duration of the control subframe (in seconds).
	double controlDuration () { return controlSlotDuration() * controlSlots(); }
	//! Return the duration of the data subframe (in seconds).
	/*!
	  Note that this is not product of the minislot duration times the
	  number of minislots. It is the difference between the frame duration
	  and the control subframe duration.
	  */
	double dataDuration () { return frameDuration_ - controlDuration(); }

protected:
	//! Recompute derived parameters.
	bool recompute ();

	//! Dump the parameters to the specified output stream.
	void dump (FILE* os);
};

//! Physical layer.
class WimshPhy : public TclObject {
protected:
	//! Descriptor for each received burst, to detect inteference.
	struct BurstDesc {
		//! Source node.
		WimaxNodeId src_;
		//! Destination node.
		WimaxNodeId dst_;
		//! Tx start time.
		double start_;
		//! Tx finish time.
		double finish_;
		//! Pointer to the channel over which this burst was received.
		WimshChannel* channel_;
		//! Burst dispatched (neighbor only, otherwise null).;
		WimshBurst* burst_;
	};

	//! List of descriptors of received bursts to detect interference.
	std::list<BurstDesc> rxBursts_;

	//! Multi-timer for transmission finish times.
	TMultiTimer<WimshPhy, WimshBurst*> rxFinishTimes_;

	//! Pointer to the channel to which the PHY is connected (either tx/rx).
	WimshChannel* channel_;

	//! Pointer to the PHY MIB.
	WimshPhyMib* phyMib_;

	//! Pointer to the MAC layer.
	WimshMac* mac_;

	//! Pointer to the topology.
	WimshTopologySimple* topology_;

	//! Small amount of time that is used to check for transmission collision.
	/*!
	  This value should definitely be smaller than the duration of an
	  OFDM symbol.
	  */
	double epsilon_;

public:
	//! Initialize an empty PHY.
	WimshPhy ();
	//! Do nothing.
	virtual ~WimshPhy () { }

	//! Switch the channel/mode used by this PHY.
	/*!
	  If no channel is specified, then only the mode is changed.
	  If there is a channel switch or a tx/rx switch, then
	  we drop any burst which has not been dispatched yet.

	  :TODO: This function should be optimized.
	  */
	void setMode (wimax::ChannelStatus s, WimshChannel* channel = 0);
	//! Send a burst to the current channel.
	/*!
	  Compute the burst duration (including physical preamble, if needed)
	  and send the latter to the channel.
	  */
	void sendBurst (WimshBurst* burst);
	//! Receive a burst from the channel (perhaps corrupted).
	/*!
	  Drop any corrupted PDU and then pass the PDUs to the MAC layer.
	  */
	void recvBurst (WimshBurst* burst);
	//! Handle the timer_ event.
	/*!
	  Get the burst with the earliest finish time. If this is from a
	  neighbor of ours, then check whether there are ongoing communications
	  which interfere with this one. If so, then set the error flag
	  into the burst. In any case, pass the burst to the MAC layer.
	  If this is not from a neighbor of ours, then just ignore the burst.
	  The burst is also ignore if there was a channel or tx/rx switch
	  of the PHY during this burst's transmission.
	  
	  :TODO: This function should be optimized.
	  */
	void handle (WimshBurst*);

protected:
	//! Tcl interface.
	virtual int command(int argc, const char*const* argv);

private:
	//! Utility function to determine whether transmissions overlap.
	/*!
	  Return true if x is greater than y of at most one epsilon_.
	  */
	bool gt (double x, double y) {
		return ( x > ( y + epsilon_ ) ) ? true : false; }

private:
	WimshPhy (const WimshPhy&);
	void operator= (const WimshPhy&);
};

#endif // __NS2_WIMSH_PHY_H
