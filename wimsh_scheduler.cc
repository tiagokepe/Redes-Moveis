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

#include <wimsh_scheduler.h>

#include <wimsh_mac.h>
#include <wimsh_packet.h>
#include <wimsh_bwmanager.h>

#include <stat.h>
#include <ip.h>

/*
 *
 * class WimshScheduler
 *
 */

WimshScheduler::WimshScheduler (WimshMac* m) : mac_ (m)
{
	bufSize_ = 0;
	maxBufSize_ = 0;
}

int
WimshScheduler::command (int argc, const char * const* argv)
{
	if ( argc == 2 && strcmp(argv[0], "size") == 0 ) {
		maxBufSize_ = atoi (argv[1]);
		if ( maxBufSize_ < 0 ) {
			fprintf (stderr, "invalid buffer size '%d'\n", maxBufSize_);
			return TCL_ERROR;
		}
		return TCL_OK;
	}

	return TCL_ERROR;
}

/*
 *
 * class WimshSchedulerFifo
 *
 */

WimshSchedulerFifo::WimshSchedulerFifo (WimshMac* m) :
	WimshScheduler (m)
{
	// nihil
}

int
WimshSchedulerFifo::command (int argc, const char * const* argv)
{
	return WimshScheduler::command (argc, argv);
}

void
WimshSchedulerFifo::initialize ()
{
	// resize the vectors of FIFO queues and sizes to the specified size
	buffer_.resize (mac_->nneighs());
	size_.resize (mac_->nneighs());
}

void
WimshSchedulerFifo::addPdu (WimaxPdu* pdu)
{
	if ( WimaxDebug::trace("WSCH::addPdu") ) fprintf (stderr,
			"%.9f WSCH::addPdu     [%d] %s\n",
			NOW, mac_->nodeId(), WimaxDebug::format(pdu));

	// if the size of this PDU overflows the buffer size, drop the PDU/SDU/IP
	if ( bufSize_ + pdu->size() > maxBufSize_ ) {
		pdu->sdu()->freePayload();
		delete pdu->sdu();
		delete pdu;

		Stat::put ("wimsh_drop_overflow", mac_->index(), 1.0);
		return;
	}
	
	// retrieve the index used for this neighbor
	const unsigned int ndx = mac_->neigh2ndx (pdu->hdr().meshCid().dst());

	// update the FIFO queue size, including MAC overhead (header/crc)
	// at the moment we do not know if a fragmentation subheader will
	// be added by the fragmentation buffer
	size_[ndx] += pdu->size();

	// update the cumulative buffer size
	bufSize_ += pdu->size();

	Stat::put ("wimsh_bufsize_a", mac_->index(), bufSize_ );
	Stat::put ("wimsh_bufsize_d", mac_->index(), bufSize_ );

	// buffer the PDU
	buffer_[ndx].push (pdu);

	// indicate the updated backlog to the bandwidth manager
	mac_->bwmanager()->backlog (
			(WimaxNodeId) HDR_IP(pdu->sdu()->ip())->saddr(),  // src node
			(WimaxNodeId) HDR_IP(pdu->sdu()->ip())->daddr(),  // dst node
			pdu->hdr().meshCid().priority(),                  // priority
			pdu->hdr().meshCid().dst(),                       // next hop
		   pdu->size());                                     // bytes
}

void
WimshSchedulerFifo::schedule (WimshFragmentationBuffer& frag, WimaxNodeId dst)
{
	unsigned int dstNdx = mac_->neigh2ndx(dst);
	if ( WimaxDebug::trace("WSCH::schedule") ) fprintf (stderr,
			"%.9f WSCH::schedule   [%d] dst %d backlog %d remaining %d\n",
			NOW, mac_->nodeId(), dst, size_[dstNdx], frag.size());
	
	// schedule PDUs directed to the specified neighbor on a FIFO manner
	// until there is room into the fragmentation buffer
	// and the output queue towards that node is backlogged

	bool spare = true;   // true if there is spare room into the burst

	while ( spare && size_[dstNdx] > 0 ) {
		// get the head-of-line PDU
		WimaxPdu* pdu = buffer_[dstNdx].front();
		buffer_[dstNdx].pop ();

		// update the size of the output queue to dst
		size_[dstNdx] -= pdu->size();

		// update the cumulative buffer size at this node
		bufSize_ -= pdu->size();
		Stat::put ("wimsh_bufsize_a", mac_->index(), bufSize_ );
		Stat::put ("wimsh_bufsize_d", mac_->index(), bufSize_ );

		// add the PDU to the fragmentation buffer
		spare = frag.addPdu (pdu);
	}

}
