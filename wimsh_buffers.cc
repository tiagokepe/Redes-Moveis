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

#include <wimsh_buffers.h>

#include <ip.h>

/*
 *
 * class WimshFragmentationBuffer
 *
 */

WimshFragmentationBuffer::WimshFragmentationBuffer ()
{
	fsn_ = 0;
	burst_ = 0;
	lastPdu_ = 0;
}

bool
WimshFragmentationBuffer::newBurst (wimax::BurstProfile p, unsigned int size)
{
	size_ = size;
	burst_ = new WimshBurst;
	burst_->profile () = p;
	// the type is automatically set when adding PDUs
	// the error flag defaults to zero
	// the size is automatically computed when adding PDUs
	// the txtime is computed by the PHY layer

	//
	// if there is no pending PDU, return with success
	//
	if ( lastPdu_ == 0 ) return true;

	//
	// if there is a pending PDU, add it immediately
	//

	// compute how many bytes are needed to send the pending PDU
	// unfragmented (note that the PDU can be a fragment already)
	unsigned int needed = lastPdu_->size();

	// there is room for the whole PDU (or fragment thereof)
	if ( needed <= size_ ) {

		// if the stored PDU is a fragment, then it is the last one
		if ( lastPdu_->hdr().fragmentation() == true )
			lastPdu_->fsh().state() = WimaxFsh::LAST_FRAG;

		burst_->addData (lastPdu_);
		size_ -= lastPdu_->size();

		lastPdu_ = 0;
		if ( size_ > WimaxPdu::minSize() ) return true;
		return false;

	// there is not enough room for the whole PDU (or fragment thereof)
	} else {

		int fragsize =
			  size_                        // remaining space
			- lastPdu_->hdr().size()       // minus the MAC header
			- WimaxPdu::meshSubhdrSize()   // minus the mesh subheader
			- WimaxFsh::size();            // minus the fragmentation subheader

		// return immediately if there is not enough space to send a fragment
		if ( fragsize < 1 ) return false;
		
		// create a new PDU to be added to the burst
		// note: the IP datagram is copied!
		WimaxPdu* newpdu = new WimaxPdu (*lastPdu_);
		WimaxSdu* newsdu = new WimaxSdu (*lastPdu_->sdu());
		newsdu->copyPayload (lastPdu_->sdu());
		newpdu->sdu() = newsdu;

		if ( lastPdu_->hdr().fragmentation() == false ) {
			// set the header fields of the newly created PDU
			newpdu->hdr().fragmentation() = true;
			newpdu->fsh().state() = WimaxFsh::FIRST_FRAG;
			newpdu->fsh().fsn() = fsn_;
			newpdu->size (fragsize);

			// Increment the next value of FSN.
			fsn ();

			// update the frame sequence number and size of the last PDU
			lastPdu_->hdr().fragmentation() = true;
			lastPdu_->fsh().fsn() = newpdu->fsh().fsn();
			lastPdu_->size (lastPdu_->sdu()->size() - fragsize);
		} else {
			// set the header fields of the newly created PDU
			newpdu->hdr().fragmentation() = true;
			newpdu->fsh().state() = WimaxFsh::CONT_FRAG;
			newpdu->fsh().fsn() = lastPdu_->fsh().fsn();
			newpdu->size (fragsize);

			// update the size of the last PDU
			lastPdu_->hdr().length() = lastPdu_->size () - fragsize;
		}

		size_ = 0;
		burst_->addData (newpdu);
		return false;
	}
}

bool
WimshFragmentationBuffer::addPdu (WimaxPdu* pdu)
{
	// Check the PDU size against the remaining size.
	// If there is enough room, do not fragment the PDU.
	if ( size_ >= pdu->size() ) {

		// Just add the PDU to the burst.
		burst_->addData (pdu);

		// Even though the default MAC header does not include the
		// fragmentation subheader, we still have to increment the FSN
		fsn ();

		// Update the remaining size.
		size_ -= pdu->size();

		// Check whether there is still some room in this burst.
		// Note that this is a necessary but not sufficient condition.
		if ( size_ > WimaxPdu::minSize() ) return true;
		return false;
	}

	// Check whether it is possible to fragment the current PDU.
	// Specifically, if at least one byte (plus MAC header plus
	// the mesh subheader plus the fragmentation subheader) fit in there,
	// then fragment it. Otherwise, just keep the PDU unfragmented for later.

	// In any case, save the last PDU.
	lastPdu_ = pdu;

	// Check the maximum fragment size.
	int fragsize =
		  size_                        // remaining space
		- pdu->hdr().size()            // minus the MAC header
		- WimaxPdu::meshSubhdrSize()   // minus the mesh subheader
		- WimaxFsh::size();            // minus the fragmentation subheader

	if ( fragsize > 0 ) {
		// Create a new PDU to be added to the burst.
		WimaxPdu* newpdu = new WimaxPdu (*pdu);
		WimaxSdu* newsdu = new WimaxSdu (*pdu->sdu());
		newsdu->copyPayload (pdu->sdu());
		newpdu->sdu() = newsdu;

		// Set the header fields of the newly created PDU.
		newpdu->hdr().fragmentation() = true;
		newpdu->fsh().state() = WimaxFsh::FIRST_FRAG;
		newpdu->fsh().fsn() = fsn_;
		newpdu->size (fragsize);

		// Increment the FSN to its next value.
		fsn();

		// Update the size of the last PDU.
		lastPdu_->hdr().fragmentation() = true;
		lastPdu_->fsh().fsn() = newpdu->fsh().fsn();
		lastPdu_->size (lastPdu_->sdu()->size() - fragsize);

		size_ = 0;
		burst_->addData (newpdu);
	}

	// in any case, indicate that no more PDUs can be added to this burst
	return false;
}
