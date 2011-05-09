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

#ifndef __NS2_WIMSH_SCHEDULER_H
#define __NS2_WIMSH_SCHEDULER_H

#include <wimax_common.h>
#include <wimsh_buffers.h>
#include <t_timers.h>

class WimshMac;

/*
 *
 * class WimshScheduler
 *
 */

//! Abstract class for an 802.16 packet scheduler.
/*!
  Each MAC has one WimshScheduler object, which receives and stores
  MAC PDUs from the MAC itself. We assume that the MAC already filled
  the following MAC header fields: length, src/dst NodeID, crc, priority,
  precedence. The fragmentation subheader, on the other hand, will be
  added if needed by the WimshFragmentationBuffer object.
  */
class WimshScheduler {
protected:
	//! Pointer to the MAC layer.
	WimshMac* mac_;

	//! Cumulative occupancy (in bytes) of the FIFO queues.
	/*!
	  Must be updated by the functions of derived classes.
	  
	  For instance, it may be used to drop packets.
	  */
	unsigned int bufSize_;

	//! Maximum buffer size (in bytes). Set by the MAC via Tcl command.
	unsigned int maxBufSize_;

public:
	//! Create an empty bandwidth manager.
	WimshScheduler (WimshMac* m);
	//! Do nothing.
	virtual ~WimshScheduler () { }

	//! Initialize internal data structures. Called by the MAC layer.
	virtual void initialize () = 0;

	//! Adds a MAC PDU to this object.
	virtual void addPdu (WimaxPdu* pdu) = 0;

	//! Schedule a new data burst to a neighbor.
	virtual void schedule (WimshFragmentationBuffer& frag, WimaxNodeId dst) = 0;

	//! Return the size, in bytes, of the queue to a neighbor (by index).
	virtual unsigned int neighbor (unsigned ndx) = 0;

	//! Tcl interface via MAC.
	virtual int command (int argc, const char*const* argv);

	//! Return the total buffer occupancy, in bytes.
	virtual unsigned int bufSize () { return bufSize_; }
};

/*
 *
 * class WimshSchedulerFifo
 *
 */

//! FIFO packet scheduler.
/*!
  MAC PDUs are stored in several FIFO queues, on a per destination NodeID
  basis.
  */
class WimshSchedulerFifo : public WimshScheduler {

	//! Vector of FIFO queues of MAC PDUs.
	std::vector< std::queue<WimaxPdu*> > buffer_;

	//! Occupancy (in bytes) of each FIFO queue.
	std::vector<unsigned int> size_;

public:
	//! Create an  empty scheduler.
	WimshSchedulerFifo (WimshMac* m);
	//! Do nothing.
	~WimshSchedulerFifo () { }

	//! Resize the number of queues based on the number of neighbors.
	void initialize ();

	//! Adds a MAC PDU to this object.
	void addPdu (WimaxPdu* pdu);

	//! Schedule a new data burst to a neighbor.
	void schedule (WimshFragmentationBuffer& frag, WimaxNodeId dst);

	//! Return the size, in bytes, of the queue to a neighbor.
	unsigned int neighbor (unsigned int ndx) { return size_[ndx]; }

	//! Tcl interface via MAC.
	int command (int argc, const char*const* argv);
};

#endif // __NS2_WIMSH_SCHEDULER_H
