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

#ifndef __NS2_WIMSH_BUFFERS_H
#define __NS2_WIMSH_BUFFERS_H

#include <wimax_common.h>
#include <wimsh_packet.h>

#include <vector>

class WimaxMac;

/*
 *
 * class WimshFragmentationBuffer
 *
 */

//! Create a new burst of PDUs (or fragments thereof).
/*!
  Only data bursts are created using the fragmentation buffer, since control
  PDU burst are never fragmented.

  Data PDU bursts are created by the fragmentation buffer but they are not
  owned by it. Thus, it is necessary to deallocate the memory allocated here
  after use.

  Each fragment created here stores a copy of the ns2 packet into the
  payload of the SDU encapsulated into the PDUs, which have to be freed
  independently.
  */
class WimshFragmentationBuffer {
	//! Current PDU burst. This variable is null only at initialization.
	WimshBurst* burst_;
	//! Remaining size in this burst.
	unsigned int size_;
	//! Last PDU (or fragment thereof) that did not fit into the PDU burst.
	WimaxPdu* lastPdu_;
	//! Next Fragment Sequence Number (FSN) to be used.
	unsigned char fsn_;
public:
	//! Create an empty fragmentation buffer.
	WimshFragmentationBuffer ();
	//! Do nothing.
	~WimshFragmentationBuffer () { }

	//! Create a new PDU burst. True if there is still some room.
	/*!
	  The burst profile to be used is specified, as well as the maximum size
	  of this burst. If an SDU does not fit into the remaining size, then
	  it is fragmented, if possible.

	  Since the last PDU (or fragment thereof) is kept for using it into
	  the subsequent PDU burst, it is possible that a new burst is full
	  immediately after the invocation of newBurst(). In this case, false
	  is returned to indicate that new PDUs cannot be added.
	  */
	bool newBurst (wimax::BurstProfile p, unsigned int size);
	//! Add a PDU, if possible. True if the PDU is entirely added.
	bool addPdu (WimaxPdu* pdu);
	//! Return the current PDU burst.
	WimshBurst* getBurst () { return burst_; }
	//! Return the remaining size.
	unsigned int size () { return size_; }

protected:
	//! Add the backlog of a fragmentation subheader to the bandwidth manager.
	void addBacklogFsh (WimaxPdu* pdu);
	//! Move the Fragment Sequence Number to the next value.
	void fsn () { fsn_ = ( fsn_ + 1 ) % WimaxFsh::fsnSize(); }
};

#endif // __NS2_WIMSH_BUFFERS_H
