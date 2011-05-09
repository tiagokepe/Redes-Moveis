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

#define UINT_MAX 4294967295U

#ifndef __NS2_WIMSH_TOPOLOGY_H
#define __NS2_WIMSH_TOPOLOGY_H

#include <wimax_common.h>

#include <rng.h>

#include <vector>

// struct used in computeGroupsEur().
struct lista_{
    unsigned int gruppo;
    unsigned int elementi;
    lista_ *next;
};

/*!
  Abstract topology class for WiMAX networks.
  */
class WimshTopology : public TclObject {
public:
	//! Do nothing.
	WimshTopology () { }
	//! Do nothing.
	virtual ~WimshTopology () { }

	//! Return true if the links from a to b and from x to y interfere.
	virtual bool interfere (WimaxNodeId a, WimaxNodeId b,
			WimaxNodeId x, WimaxNodeId y) = 0;

	//! Return the list of the n-hops neighbors of a node.
	virtual void neighbors (WimaxNodeId, std::vector<WimaxNodeId>& neigh, unsigned int nhops = 1) = 0;

	//! Return true if two nodes are neighbors.
	virtual bool neighbors (WimaxNodeId, WimaxNodeId) = 0;

	//! Return the next-hop node from one node to the other.
	virtual WimaxNodeId nextHop (WimaxNodeId x, WimaxNodeId y) = 0;

	//! Return the maximum two-hop neighborhood in the network.
	virtual std::vector< WimaxNodeId > getMaxTwoHopNeighborhood () = 0;

	//! Return destination Vector
	/*!
	  Given a source and a destination computes the possible next hops
	  vector
	  */
	virtual std::vector< wimax::NextHopInfo > getDestinationVect 
								( WimaxNodeId src, WimaxNodeId dst ) = 0;


	//! Return the number of node in the scenario
	virtual unsigned int numNodes() = 0;

	//! Tcl interface.
	virtual int command(int argc, const char*const* argv) = 0;
private:
	WimshTopology (const WimshTopology&);   // not defined
	void operator= (const WimshTopology&);  // not defined
};

/*!
  Simple topology for WiMAX networks, where nodes do not have an explicit
  spatial location (i.e. coordinates). Topology is based on logical
  links between nodes, and it is represented as a connectivity matrix
  (vertices = nodes, edges = links) and a conflit matrix (vertices = links,
  edges = conflicts).
  Additionally, an all-pairs-shortest-path matrix is computed, so that
  the MAC/routing layers can query the topology to know the shortest
  path to a destination.
  */
class WimshTopologySimple : public WimshTopology {
	typedef std::map< WimaxNodeId, std::map< unsigned int,
		std::vector<WimaxNodeId> > > NhopsNeighborsMap;

	//! Link structure
	struct Link {
		WimaxNodeId src_;
		WimaxNodeId dst_;
		Link (WimaxNodeId src, WimaxNodeId dst ){
			src_ = src; dst_ = dst;
		}
	};
	//! Connectivity graph.
	/*!
	  Element (i, j) is either zero (i.e. no direct communication is possible
	  between nodes i and j) or it contains the link identifier used in
	  the conflict matrix.
	  */
	Matrix<unsigned int> connectivity_;
	//! Link vector.
	/*!
	  Hold all links in the topology. The vector index is assumed to be the
	  name of the vector
	  */
	std::vector< Link > links_;
	//! Conflict graph.
	/*!
	  Two links (a,b) and (x,y) interfere iff a == x or a == y or
	  b == x or b == y OR there exists at least 1 link between the
	  two pairs.
	 */
	Matrix<bool> conflict_;
	//! Next-hop matrix.
	/*!
	  Each element (i, j) contains the next-hop from i to reach j.
	  Diagonal elements are 0. If a non-diagonal element is 0, then
	  those nodes cannot reach each other (i.e. the connectivity graph
	  is not connected).
	  */
	Matrix<unsigned int> nextHop_;
	//! N-hops neighbor data structure
	/*!
	  This map contains, for each nodeID, the n-hop neighbors nodeID.
	  Note that nodes X and Y are considered n-hop neighbors only
	  if there are exactly n links between them.
	  */
	NhopsNeighborsMap nHopsNeighbors_;

	//! Up-to-date flag.
	bool update_;
	//! Next link identifier for the conflict matrix.
	unsigned int nextLink_;

	//! Random number generator to pick up next hop randomly.
	/*!
	  The next-hop is picked up randomly among those at minimum distance
	  towards the destination.
	  */
	RNG rng;
	
public:
	//! Create an empty topology object.
	WimshTopologySimple ();
	//! Do nothing.
	~WimshTopologySimple () { }
	//! Tcl interface.
	int command (int argc, const char*const* argv);
	//! Return true if the links from a to b and from x to y interfere.
	/*!
	  We assume that nodes a, b, x, and y exist.
	 */
	bool interfere (WimaxNodeId a, WimaxNodeId b, WimaxNodeId x, WimaxNodeId y);
	//! Return the list of nhops-neighbors of node x (which is assumed to exist). Default first-hop
	void neighbors (WimaxNodeId x, std::vector<WimaxNodeId>& neigh, unsigned int nhop = 1);
	//! Return true if two nodes are neighbors.
	bool neighbors (WimaxNodeId x, WimaxNodeId y) {
		return ( connectivity_.at (x, y) > 0 ) ? true : false; }
	//! Return the next-hop node from x to y (which are assumed to exist).
	WimaxNodeId nextHop (WimaxNodeId x, WimaxNodeId y) {
		return nextHop_.at(x, y); }
	//! Return the maximum two-hop neighborhood in the network.
	std::vector< WimaxNodeId > getMaxTwoHopNeighborhood ();
	//! Return destination Vector
	/*!
	  Given a source and a destination computes the possible next hops
	  vector
	  */
	std::vector< wimax::NextHopInfo > getDestinationVect 
								( WimaxNodeId src, WimaxNodeId dst );
	//! Return the number of node in the scenario
	unsigned int numNodes() { return connectivity_.getRows(); }
	
protected:
	//! Compute the conflict and apsp matrices.
	void recompute ();
	//! Dump the content of all the graphs to the specified stream.
	void dump (FILE* os);

private:
	//! Utility function to retrieve the next-hop from the path matrix.
	unsigned int getNextHop (unsigned int, unsigned int, Matrix<unsigned int>&);
	//! Bellman-Ford algorithm
	void bellmanFord ( Matrix<unsigned int>& connectivity,
					   Matrix<unsigned int>& distance,
					   Matrix<unsigned int>& path );

};

#endif // __NS2_WIMSH_TOPOLOGY_H
