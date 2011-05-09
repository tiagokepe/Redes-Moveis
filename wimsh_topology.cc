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

#include <wimsh_topology.h>
#include <wimax_defs.h>

//#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/times.h>

/*
 *
 * WimshTopologySimple
 *
 */

static class WimshTopologySimpleClass : public TclClass {
public:
   WimshTopologySimpleClass() : TclClass("WimshTopology/Simple") {}
   TclObject* create(int, const char*const*) {
      return (new WimshTopologySimple);
   }
} class_wimsh_topology_simple;

WimshTopologySimple::WimshTopologySimple ()
{
	update_ = true;
	nextLink_ = 1;
}

int WimshTopologySimple::command (int argc, const char*const* argv)
{
	/*
    * add a direct link communication between two nodes
	 */
	if ( argc == 4 && strcmp (argv[1], "connect") == 0 ) {
		unsigned int i = (unsigned int) atoi (argv[2]);
		unsigned int j = (unsigned int) atoi (argv[3]);
		if ( i == j ) return TCL_ERROR;
		connectivity_.at (i, j) = nextLink_++;
		connectivity_.at (j, i) = nextLink_++;
		//update_ = false;
		update_ = 0;
		return TCL_OK;
	}
	/*
    * dump all the matrices to stderr
    */	 
	else if ( argc == 2 && strcmp (argv[1], "dump") == 0 ) {
		dump (stderr);
		return TCL_OK;
	}
	/*
    * return the distance between two nodes
    */	 
	else if ( argc == 4 && strcmp (argv[1], "distance") == 0 ) {

		
		// path matrix
		Matrix<unsigned int> path;
		// distance matrix
		Matrix<unsigned int> dist;
		// run the bellman ford algorithm
		bellmanFord( connectivity_, dist, path );

		WimaxNodeId a = (WimaxNodeId) atoi (argv[2]);
		WimaxNodeId b = (WimaxNodeId) atoi (argv[3]);
		Tcl::instance().resultf("%d", dist.at(a,b));
		return TCL_OK;
	}
	/*
    * dump all the matrices to stderr
    */	 
	else if ( argc == 2 && strcmp (argv[1], "initialize") == 0 ) {
		recompute ();
		return TCL_OK;
	}
	return command (argc, argv);
}

void WimshTopologySimple::recompute ()
{

	// alias for the connectivity matrix size
	const unsigned int N = connectivity_.getRows();
	// alias for the connectivity matrix
	Matrix<unsigned int>& M = connectivity_;
	// alias for the conflicy matrix
	Matrix<bool>& C = conflict_;
	// alias for the n-hop neighboorhood data structure
	NhopsNeighborsMap& NM = nHopsNeighbors_;

	// reset the links vector
	links_.clear();
	

	
	// fill the link vector
	for ( unsigned int src = 0 ; src < N ; src++ ) {
		for ( unsigned int dst = 0 ; dst <N ; dst++ ) {
				if ( M.at( src, dst ) > 0) links_.push_back( Link( src, dst ) );
				
		}
	}
	
	unsigned int num=links_.size();
	unsigned int linkssrc_[num];
	unsigned int linksdst_[num];
	
	
	for ( unsigned int src = 0 ; src < num ; src++ ) {
	    linkssrc_[src]=0;
	    linksdst_[src]=0;
	    }
	for ( unsigned int src = 0 ; src < N ; src++ ) {
		for ( unsigned int dst = 0 ; dst <N ; dst++ ) {	
				if ( M.at( src, dst ) > 0){	
				    linkssrc_[M.at(src,dst)-1]=src;
				    linksdst_[M.at(src,dst)-1]=dst;
				}
	    }
	}
	// reset the conflict matrix to empty
	conflict_.reset ();
	// fill the conflict matrix according to the definition given in the
	// comment of conflict_ declaration for "protocol model" details.
	// The else if conditions depends on the protocol model
	for ( unsigned int i = 0; i < links_.size(); i++ ) {
		for ( unsigned int j = 0; j < links_.size(); j++ ) {
			if ( i == j ) C.at ( i, j ) = 0;
			else if (
					linkssrc_[i] == linkssrc_[j] ||
					linkssrc_[i] == linksdst_[j] ||
					linksdst_[i] == linkssrc_[j] ||
				    linksdst_[i] == linksdst_[j] ||
					M.at( linkssrc_[i], linksdst_[j] ) ||
					M.at( linksdst_[i], linkssrc_[j] )
					)
					C.at ( i, j ) = 1;
			else C.at ( i, j ) = 0;
		}
	}


	
	// path matrix
	Matrix<unsigned int> path;
	// distance matrix
	Matrix<unsigned int> dist;
	// run the bellman ford algorithm
	bellmanFord( M, dist, path );

	// clear the n-hop neighbors structure
	NM.clear ();

	// elements of the n-hop maps
	std::map< unsigned int, std::vector<WimaxNodeId> > nhops_neigh;

	// compute the nhop neighboorhood for each node
	for ( unsigned int i = 0 ; i < N ; i++ ) {
		for ( unsigned int j = 0 ; j < N ; j++ ) {
			// if the distance does not exist, create it
			nhops_neigh[dist.at(i, j)].push_back(j);
		}
		NM[i] = nhops_neigh;
		nhops_neigh.clear();
	} 

	// compute the groups through coloring graph algorithm
	// computeGroupsOpt (); // XXX

	// compute the groups through heuristic algorithm
	// computeGroupsEur(); // XXX

	// derive the shortest path matrix
	for ( unsigned int i = 0 ; i < N ; i++ ) {
		for ( unsigned int j = 0 ; j < N ; j++ ) {

			// same node, continue
			if ( i == j ) {
				nextHop_.at(i, j) = 0;
				continue;

			// nodes are first-hop neighbors, easy
			} else if ( M.at(i, j) > 0 ) {
				nextHop_.at(i, j) = j;
				continue;
			}

			// get the destination vector from i to j
			std::vector<wimax::NextHopInfo> nhi = getDestinationVect (i, j);
			std::vector<wimax::NextHopInfo>::iterator it;

			// find the minimum number of hops
			unsigned int minDist = UINT_MAX;
			for ( it = nhi.begin() ; it != nhi.end() ; ++it ) {
				if ( it->nHops_ < minDist ) minDist = it->nHops_;
			}

			// :TODO: what if nhi.begin() == nhi.end() ?? for now, abort...
			if ( minDist == UINT_MAX ) abort ();

			// create a vector with all the next-hops at minimum distance
			std::vector<WimaxNodeId> minHops;
			for ( it = nhi.begin() ; it != nhi.end() ; ++it ) {
				if ( it->nHops_ == minDist ) minHops.push_back (it->nextHop_);
			}

			// choose a random next-hop among those at minimum distance
			nextHop_.at(i, j) = minHops[rng.uniform((int)minHops.size())];
		}
	}

	// set the update flag
	//update_ = true;
	update_ = 1;
}

unsigned int
WimshTopologySimple::getNextHop (
		unsigned int i,
		unsigned int j,
		Matrix<unsigned int>& path)
{
	if ( i != j && connectivity_.at(i, path.at(i, j) ) == 0 )
		return getNextHop (i, path.at(i, j), path);
	return path.at(i, j);
}

bool
WimshTopologySimple::interfere (
		WimaxNodeId a, WimaxNodeId b,
		WimaxNodeId x, WimaxNodeId y)
{
	// if there is not any link from a to b or from x to y return false
	if ( connectivity_.at(a, b) == 0 || connectivity_.at(x, y) == 0 ) return false;

	return conflict_.at (connectivity_.at(a, b), connectivity_.at(x, y));
}


void
WimshTopologySimple::neighbors (WimaxNodeId x, 
		std::vector<WimaxNodeId>& neigh, unsigned int nhops)
{
	NhopsNeighborsMap::iterator it = nHopsNeighbors_.find(x);
	std::map< unsigned int, std::vector<WimaxNodeId> >::iterator it2;
	
	// find all the nhops neighbors at once
	if ( it != nHopsNeighbors_.end() )
		if ( (it2 = it->second.find( nhops ) ) != it->second.end() )
			neigh = it2->second;
}


std::vector< wimax::NextHopInfo >
WimshTopologySimple::getDestinationVect ( WimaxNodeId src,  WimaxNodeId dst )
{
	std::vector< wimax::NextHopInfo > destVect;
	wimax::NextHopInfo hop;
	// src and dst are one-hop neighbor
	if ( connectivity_.at( src, dst ) ){
		hop.nextHop_ = dst;
		hop.nHops_ = 0;
		destVect.push_back ( hop );
		return destVect;
	}
	// Compute the general case, i.e. src e dst are not neighbors
	Matrix<unsigned int> C = connectivity_;

	// get all the source neighbors
	std::vector < WimaxNodeId > neigh;
	neighbors( src, neigh );
	//// run bellman-ford algorithm with the new matrix
	Matrix<unsigned int> path;
	Matrix<unsigned int> dist;

	// check whether there exists other paths by disconnecting the source node
	// disconnect the source node
	for ( unsigned int i = 0; i < C.getRows(); i++ ){
		C.at( src, i ) = 0;
		C.at( i, src ) = 0;
	}
	// run bellman-ford algorithm with the new matrix
	bellmanFord ( C, dist, path );

	Matrix<unsigned int> nextHop;

	// derive the new shortest path matrix
	for ( unsigned int i = 0 ; i < C.getRows() ; i++ ) {
		for ( unsigned int j = 0 ; j < C.getRows() ; j++ ) {
			nextHop.at(i, j) = 
				( i == j ) ? 0 :
				( C.at(i, j) > 0 ) ? j :
				getNextHop (i, j, path);
		}
	}
	// fill the destination vector
	for ( unsigned int i = 0; i < neigh.size(); i++ ){
		// a path exists other than through the source
		if ( nextHop.at ( neigh.at( i ), dst ) != src ) {
			// add the node
			hop.nextHop_ = neigh.at( i );
			hop.nHops_ = dist.at ( neigh.at( i ), dst );
			if(hop.nHops_ < UINT_MAX/2) destVect.push_back ( hop );
		}
	}
	return destVect;
}

void
WimshTopologySimple::bellmanFord ( Matrix<unsigned int>& M,
								   Matrix<unsigned int>& dist,
					               Matrix<unsigned int>& path )
{
	// alias for the connectivity matrix size
	const unsigned int N = M.getRows();
	// apply the O(N^3) all-pairs-shortest-path algorithm
	// (modified Bellman-Ford algorithm)

	// this value of 'infinity' is assigned as the distance between
	// two nodes that do not share a link
	// it was chosen so that the sum of any two 'infinity' values
	// does not wrap-around the representation space of unsigned integers
	// of course, this is only true for the current implementation of
	// the modified Bell-Fordman algorithm, which sums at most two distances
	const unsigned int infinity = UINT_MAX/2;

	for ( unsigned int i = 0 ; i < N ; i++ ) {
		for ( unsigned int j = 0 ; j < N ; j++ ) {
			if ( i != j && M.at(i, j) == 0 ) dist.at(i, j) = infinity;
			else if ( M.at(i, j) > 0 ) dist.at(i, j) = 1;
			else dist.at(i, j) = 0;
		}
	}

	for ( unsigned int i = 0 ; i < N ; i++ ) {
		for ( unsigned int j = 0 ; j < N ; j++ ) {
			if ( dist.at(i, j) > 0 ) path.at(i, j) = i;
			else path.at(i, j) = 0;
		}
	}

	// run the main algorithm
	for ( unsigned int k = 0 ; k < N ; k++ ) {
		for ( unsigned int i = 0 ; i < N ; i++ ) {
			for ( unsigned int j = 0 ; j < N ; j++ ) {
				if ( dist.at(i, j) > dist.at(i, k) + dist.at(k, j) ) {
					dist.at(i, j) = dist.at(i, k) + dist.at(k, j);
					path.at(i, j) = path.at(k, j);
				}
			}
		}
	}
}

std::vector< WimaxNodeId >
WimshTopologySimple::getMaxTwoHopNeighborhood ()
{
	NhopsNeighborsMap::iterator it;
	std::vector<WimaxNodeId> oldNeigh;
	std::vector<WimaxNodeId> newNeigh;

	for ( it = nHopsNeighbors_.begin() ; it != nHopsNeighbors_.end() ; ++it ) {

		// These are the one-hop neighbors of this node.
		std::vector<WimaxNodeId> one = (it->second)[1];
		// These are the two-hop neighbors of this node.
		std::vector<WimaxNodeId> two = (it->second)[2];

		// If this is the first iteration, then we do not have any other
		// two-hop neighborhood to compare with. Therefore, we just create
		// the 'old' vector.
		if ( it == nHopsNeighbors_.begin() ) {
			// Concatenate the above vectors into 'oldNeigh'.
			for ( unsigned int i = 0 ; i < one.size() ; i++ )
				oldNeigh.push_back (one[i]);
			for ( unsigned int i = 0 ; i < two.size() ; i++ )
				oldNeigh.push_back (two[i]);

		// Otherwise, we compare the size of the old two-hop neighborhood
		// to that of the new (i.e., current) two-hop neighborhood.
		} else {
			newNeigh.clear();
			// Concatenate the above vectors into 'newNeigh'.
			for ( unsigned int i = 0 ; i < one.size() ; i++ )
				newNeigh.push_back (one[i]);
			for ( unsigned int i = 0 ; i < two.size() ; i++ )
				newNeigh.push_back (two[i]);

			if ( newNeigh.size() > oldNeigh.size() ) oldNeigh = newNeigh;
		}
	}

	return oldNeigh;
}

void
WimshTopologySimple::dump (FILE* os)
{
	if ( ! update_ ) recompute();

	fprintf (os, "** connectivity graph **\n");
	for ( unsigned int i = 0 ; i < connectivity_.getRows() ; i++ ) {
		for ( unsigned int j = 0 ; j < connectivity_.getCols() ; j++ ) {
			if ( j > 0 ) fprintf (os, " ");
			fprintf (os, "%3u", connectivity_.at(i, j));
		}
		fprintf (os, "\n");
	}
	fprintf (os, "** link naming **\n");
	for ( unsigned int i = 0; i < links_.size(); i++ ) {
		fprintf (os, "Link ID: %d => src: %d, dst: %d", 
			 i , links_.at( i ).src_, links_.at( i ).dst_ );
		fprintf (os, "\n");
	}

	fprintf (os, "** conflict graph **\n");
	for ( unsigned int i = 0 ; i < conflict_.getRows() ; i++ ) {
		for ( unsigned int j = 0 ; j < conflict_.getCols() ; j++ ) {
			if ( j > 0 ) fprintf (os, " ");
			fprintf (os, "%u", ( conflict_.at(i, j) == true ) ? 1 : 0);
		}
		fprintf (os, "\n");
	}

	fprintf (os, "** next-hop graph **\n");
	for ( unsigned int i = 0 ; i < nextHop_.getRows() ; i++ ) {
		for ( unsigned int j = 0 ; j < nextHop_.getCols() ; j++ ) {
			if ( j > 0 ) fprintf (os, " ");
			fprintf (os, "%3u", nextHop_.at(i, j));
		}
		fprintf (os, "\n");
	}

	fprintf (os, "** n-hops data structure **\n");
	NhopsNeighborsMap::iterator it;
	std::map< unsigned int, std::vector<WimaxNodeId> >::iterator it2;
	for ( it = nHopsNeighbors_.begin(); it != nHopsNeighbors_.end(); ++it ) {
		fprintf (os, "%s %d", "Node ", it->first);
		fprintf (os, "\n");
		for ( it2 = it->second.begin(); it2 != it->second.end(); ++it2 ) {
			if ( it2 != it->second.end() ) fprintf (os, "%s %d", "dist:", it2->first);
			fprintf (os, "%s", " nodes: ");
			for ( unsigned int i = 0; i < it2->second.size(); i++){
				fprintf (os, "%d ", it2->second[i]);
			}
			fprintf (os, "\n");
		}
		fprintf (os, "\n");
	}

	fprintf (os, "\n");


}

