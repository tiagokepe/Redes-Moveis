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

#include <wimsh_packet.h>

/*
 *
 * class WimshBurst
 *
 */

void
WimshBurst::addMshDsch (WimshMshDsch* m)
{
	mshDsch_ = m;

	// set the type to MSHDSCH
	type_ = wimax::MSHDSCH;

	// update the burst size
	size_ = mshDsch_->size ();
}

void
WimshBurst::addMshNcfg (WimshMshNcfg* m)
{
	mshNcfg_ = m;
	type_ = wimax::MSHNCFG;
	size_ = mshNcfg_->size ();
}

void
WimshBurst::addMshNent (WimshMshNent* m)
{
	mshNent_ = m;
	type_ = wimax::MSHNENT;
	size_ = mshNent_->size ();
}


WimshBurst::~WimshBurst ()
{
	// Delete all PDUs, including encapsuled SDUs and ns-2 packets.
	std::list<WimaxPdu*>::iterator it;
	for ( it = pdus_.begin() ; it != pdus_.end() ; ++it ) {
		(*it)->sdu()->freePayload();
		delete (*it)->sdu();
		delete (*it);
	}

	// Delete control messages, if any.
	if ( mshDsch_ ) delete mshDsch_;
	if ( mshNcfg_ ) delete mshNcfg_;
	if ( mshNent_ ) delete mshNent_;
}

WimshBurst::WimshBurst (const WimshBurst& obj)
{
	// copy the transmission time
	txtime_ = obj.txtime_;

	// copy the error flag
	error_ = obj.error_;

	// copy the burst profile and type
	profile_ = obj.profile_;
	type_ = obj.type_;

	// copy the burst size
	size_ = obj.size_;

	// copy the source NodeID
	src_ = obj.src_;

	// copy the whole list of PDUs, if any
	// note that the elements + PDUs + SDUs + ns2 packets are copied
	std::list<WimaxPdu*>::const_iterator it;
	for ( it = obj.pdus_.begin() ; it != obj.pdus_.end() ; ++it ) {
		WimaxPdu* pdu = new WimaxPdu (*(*it));
		WimaxSdu* sdu = new WimaxSdu (*(*it)->sdu());
		sdu->copyPayload ((*it)->sdu());
		pdu->sdu() = sdu;
		
		pdus_.push_back (pdu);
	}

	// copy the MSH-DSCH, if any
	if ( obj.mshDsch_ ) mshDsch_ = new WimshMshDsch (*obj.mshDsch_);
	else mshDsch_ = 0;

	if ( obj.mshNcfg_ ) mshNcfg_ = new WimshMshNcfg (*obj.mshNcfg_);
	else mshNcfg_ = 0;
	
	if ( obj.mshNent_ ) mshNent_ = new WimshMshNent (*obj.mshNent_);
	else mshNent_ = 0;
}

/*
 *
 * class WimshMshDsch
 *
 */

WimshMshDsch::AllocationType
WimshMshDsch::allocationType_ = WimshMshDsch::BASIC;

void
WimshMshDsch::slots2level (
		unsigned int N, unsigned int minislots,
		unsigned char& level, Persistence& persistence)
{
   if ( minislots <= N ) {
      level = minislots;
      persistence = WimshMshDsch::FRAME1;
   } else if ( minislots <= 2 * N ) {
      level = 1 + ( minislots - 1 ) / 2;
      persistence = WimshMshDsch::FRAME2;
   } else if ( minislots <= 4 * N ) {
      level = 1 + ( minislots - 1 ) / 4;
      persistence = WimshMshDsch::FRAME4;
   } else if ( minislots <= 8 * N ) {
      level = 1 + ( minislots - 1 ) / 8;
      persistence = WimshMshDsch::FRAME8;
   } else if ( minislots <= 32 * N ) {
      level = 1 + ( minislots - 1 ) / 32;
      persistence = WimshMshDsch::FRAME32;
   } else if ( minislots <= 128 * N ) {
      level = 1 + ( minislots - 1 ) / 128;
      persistence = WimshMshDsch::FRAME128;
   } else {
      level = N;
      persistence = WimshMshDsch::FRAME128;
   }
}

void
WimshMshDsch::addContiguous (GntIE& x)
{
	std::list<GntIE>::iterator it;

	// find a contiguous grant to x in the same set of frames, if any
	for ( it = gnt_.begin() ; it != gnt_.end() ; ++it ) {
		if ( x.nodeId_ == it->nodeId_ &&
				x.frame_ == it->frame_ &&
				x.persistence_ == it->persistence_ &&
				x.fromRequester_ == it->fromRequester_ &&
				x.channel_ == it->channel_ ) {

			// check if (x, *it) are contiguous
			if ( x.start_ + x.range_ == it->start_ ) {
				it->start_ = x.start_;
				it->range_ = x.range_ + it->range_;
				break;

			// check if (*it, x) are contiguous
			} else if ( it->start_ + it->range_ == x.start_ ) {
				it->range_ = x.range_ + it->range_;
				break;

			// if none of the above, then continue searching other IEs
			} else continue;
		}
	}

	// if no suitable IE is found, add a new IE
	if ( it == gnt_.end() ) {
		hdr_.length() += GntIE::size();
		gnt_.push_front(x);
	}
}

void
WimshMshDsch::addContiguous (AvlIE& x)
{
	std::list<AvlIE>::iterator it;

	// find a contiguous grant to x in the same set of frames, if any
	for ( it = avl_.begin() ; it != avl_.end() ; ++it ) {
		if ( x.frame_ == it->frame_ &&
				x.persistence_ == it->persistence_ &&
				x.channel_ == it->channel_ ) {

			// check if (x, *it) are contiguous
			if ( x.start_ + x.range_ == it->start_ ) {
				it->start_ = x.start_;
				it->range_ = x.range_ + it->range_;
				break;

			// check if (*it, x) are contiguous
			} else if ( it->start_ + it->range_ == x.start_ ) {
				it->range_ = x.range_ + it->range_;
				break;

			// if none of the above, then continue searching other IEs
			} else continue;
		}
	}

	// if no suitable IE is found, add a new IE
	if ( it == avl_.end() ) {
		hdr_.length() += AvlIE::size();
		avl_.push_front(x);
	}
}
