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

#include <wimsh_weight_manager.h>

WimshWeightManager::WimshWeightManager (WimshMac* m) : mac_(m), timer_ (this)
{
	interval_ = 0;
	nFlowDesc_ = 0;
	normalizeFlow_ = false;
	for ( unsigned int i = 0 ; i < WimaxMeshCid::MAX_PRIO ; i++ )
		prioWeights_[i] = 1.0;
}

void
WimshWeightManager::recompute ()
{
	DescList::const_iterator it;
	NdxList::const_iterator jt;

	// get the number of neighbors from the MAC layer
	const unsigned int N = mac_->nneighs();

	// reset the weights arrays
	for ( unsigned int i = 0 ; i < N ; i++ ) {
		weightIn_[i] = 0;
		weightOut_[i] = 0;
	}

	// at this point, if there are no flows, return immediately
	if ( nFlowDesc_ == 0 ) return;

	// sum of the reciprocal of alpha values
	double sum = 0;

	// for each element in the incoming list of flow descriptors,
	// add one unit to the weights array of incoming links
	for ( it = in_.begin() ; it != in_.end() ; ++it ) {
		// search for multipath flows (ie. same source and destination
		// but different link) and divide the weight unit by their number
		for ( jt = it->ndx_.begin() ; jt != it->ndx_.end() ; ++jt ) {
			double unit = prioWeights_[it->prio_];
			if ( normalizeFlow_ ) unit /= (double)it->ndxSize_;
			weightIn_[*jt] += unit;
			sum += unit;
		}
	}

	// do the same for the weights array of outgoing links
	for ( it = out_.begin() ; it != out_.end() ; ++it ) {
		// search for multipath flows (ie. same source and destination
		// but different link) and divide the weight unit by their number
		for ( jt = it->ndx_.begin() ; jt != it->ndx_.end() ; ++jt ) {
			double unit = prioWeights_[it->prio_];
			if ( normalizeFlow_ ) unit /= (double)it->ndxSize_;
			weightOut_[*jt] += unit;
			sum += unit;
		}
	}

	// divide each weight for the sum of weights, ie. normalize weights
	for ( unsigned int i = 0 ; i < N ; i++ ) {
		weightIn_[i] /= sum;
		weightOut_[i] /= sum;
	}
}

void
WimshWeightManager::initialize ()
{
	// get the number of neighbors from the MAC layer
	const unsigned int N = mac_->nneighs();

	// resize the weight arrays and set all the weights to zero
	weightIn_.resize (N);
	weightOut_.resize (N);

	for ( unsigned int i = 0 ; i < N ; i++ ) {
		weightIn_[i] = 0;
		weightOut_[i] = 0;
	}

	// start the stale detection timer the first time
	if ( interval_ > 0 ) timer_.start ( interval_ );
}

void
WimshWeightManager::handle ()
{
	if ( WimaxDebug::trace("WWMN::handle") ) fprintf (stderr,
			"%.9f WWMN::handle     [%d]\n", NOW, mac_->nodeId());

	// if the user specified a zero or negative value for the interval
	// then he/she wants the weights never to expire => return immediately
	// without restarting the timer, hence this function will not be
	// called again
	if ( interval_ <= 0 ) return;

	const double now = Scheduler::instance().clock(); // alias for NOW

	DescList::iterator it;

	// remove stale incoming flows
	for ( it = in_.begin() ; it != in_.end() ; ) {
		if ( now - it->lastRcvd_ > interval_ ) {
			DescList::iterator drop = it++;
			in_.erase (drop);
			--nFlowDesc_;
			continue;
		}
		++it;
	}

	// remove stale outgoing flows
	for ( it = out_.begin() ; it != out_.end() ; ) {
		if ( now - it->lastRcvd_ > interval_ ) {
			DescList::iterator drop = it++;
			in_.erase (drop);
			--nFlowDesc_;
			continue;
		}
		++it;
	}

	// recompute weights
	recompute ();

	// restart the timer
	timer_.start ( interval_ );
}

void
WimshWeightManager::flow (
		WimaxNodeId src, WimaxNodeId dst,
		unsigned char prio,
		unsigned int ndx,
		wimax::LinkDirection dir)
{
	DescList::iterator it;
	NdxList::iterator jt;

	// demux the list based on the direction
	DescList& list = ( dir == wimax::IN ) ? in_ : out_;
	
	for ( it = list.begin() ; it != list.end() ; ++it ) {
		bool found = false;
		// if there is already such a flow, then we just update its timestamp
		if ( ! it->incomplete_ &&
				it->src_ == src && it->dst_ == dst && it->prio_ == prio ) {
			for ( jt = it->ndx_.begin() ; jt != it->ndx_.end() ; ++jt ) {
				if ( *jt == ndx ) {
					it->lastRcvd_ = NOW;
					found = true;
					break;
				}
			}
		}
		if ( found ) break;
	}

	// exit if we found an element that matches <src, dst, ndx>
	if ( it != list.end() ) return;

	// otherwise, we add a new element to the list
	// and we check for matching incomplete elements, which are removed

	for ( it = list.begin() ; it != list.end() ; ++it ) {
		if ( it->incomplete_ && *(it->ndx_.begin()) == ndx ) break;
	}

	// if we found an incomplete element, then remove it
	// and update the number of flow descriptors
	// note there can be at most one incomplete element per neighbor
	if ( it != list.end() ) {
		--nFlowDesc_;
		list.erase (it);
	}

	// check if we just have to add a neighbor to a multipath flow

	for ( it = list.begin() ; it != list.end() ; ++it ) {
		if ( ! it->incomplete_ &&
				it->src_ == src && it->dst_ == dst && it->prio_ == prio ) {
			it->ndx_.push_back (ndx);
			it->ndxSize_++;
			break;
		}
	}

	// exit if we added a neighbor to an existing flow
	if ( it != list.end() ) {
		recompute ();
		return;
	}

	// create a new flow descriptor and push it into the list
	FlowDesc newflow;
	newflow.src_  = src;
	newflow.dst_  = dst;
	newflow.prio_ = prio;
	*(newflow.ndx_.begin()) = ndx;  // an element is pushed by default
	newflow.incomplete_ = false;
	newflow.lastRcvd_ = NOW;

	// push the new flow descriptor into the list
	list.push_back (newflow);

	// update the number of flow descriptors
	++nFlowDesc_;

	// recompute weights
	recompute ();
}

void
WimshWeightManager::flow (unsigned int ndx, wimax::LinkDirection dir)
{
	DescList::iterator it;
	NdxList::iterator jt;

	// demux the list based on the direction
	DescList& list = ( dir == wimax::IN ) ? in_ : out_;
	
	// break from the loop as soon as an element associated to the
	// same link (ie. ndx) is encountered
	for ( it = list.begin() ; it != list.end() ; ++it ) {
		bool found = false;
		for ( jt = it->ndx_.begin() ; jt != it->ndx_.end() ; ++jt ) {
			if ( *jt == ndx ) { found = true; break; }
		}
		if ( found ) break;
	}

	// if there is already such a flow, then just exit
	if ( it != list.end() ) return;

	// otherwise, add a new incomplete flow descriptor to the list
	FlowDesc newflow (ndx);   // incomplete by default

	// update the number of flow descriptors
	++nFlowDesc_;

	// push the new flow descriptor into the list
	list.push_back (newflow);

	// recompute weights
	recompute ();
}

int
WimshWeightManager::command (int argc, const char*const* argv)
{
   if ( argc == 2 && strcmp (argv[0], "weight-flow") == 0 ) {
      if ( strcmp (argv[1], "on") == 0 ) {
         normalizeFlow_ = true;
      } else if ( strcmp (argv[1], "off") == 0 ) {
         normalizeFlow_ = false;
      } else {
         fprintf (stderr, "invalid weight-flow '%s' command. "
               "Choose either 'on' or 'off'", argv[1]);
         return TCL_ERROR;
      }
      return TCL_OK;
   } else if ( argc == 2 && strcmp (argv[0], "weight-timeout") == 0 ) {
      interval_ = atof (argv[1]);
      return TCL_OK;
   } else if ( argc == 3 && strcmp (argv[0], "prio-weight") == 0 ) {
      unsigned int i = atoi (argv[1]);
      double x = atof (argv[2]);
      if ( i >= WimaxMeshCid::MAX_PRIO ) {
         fprintf (stderr, "priority '%d' exceeds the maximum value '%d'\n",
               i, WimaxMeshCid::MAX_PRIO - 1 );
         return TCL_ERROR;
      }
      if ( x <= 0 ) {
         fprintf (stderr,
               "priority weight must be >= 0 ('%f' not allowed)\n", x);
         return TCL_ERROR;
      }
      prioWeights_[i] = x;
      return TCL_OK;
	}
	return TCL_ERROR;
}
