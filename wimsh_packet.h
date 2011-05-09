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
#ifndef __NS2_WIMSH_PACKET_H
#define __NS2_WIMSH_PACKET_H

#include <list>

#include <wimax_packet.h>

//! MSH-DSCH control message. We assume coordinated message scheduling.
/*!
  The maximum size is not enforced while adding new IEs. Thus, all the
  functions to add IEs return the size of the available space. Since
  IEs cannot be removed (unless you do it "manually" using the returned
  list, which you should do), the amount of available space should be
  checked before adding IEs using the appropriate functions.

  The same discussion on the LinkID of WimaxCid holds also for all the
  information elements defined here.
  */
struct WimshMshDsch {
public:
	//! Maximum sequence counter value.
	enum { MAX_SEQ = 63 };
	//! Maximum number of requests.
	enum { MAX_REQ = 15 };
	//! Maximum number of availabilities.
	enum { MAX_AVL = 15 };
	//! Maximum number of grants.
	enum { MAX_GNT = 63 };
	//! Maximum number of advertised neighbors.
	enum { MAX_NGH = 255 };
	//! Maximum size of the MSH-DSCH message (in bytes).
	enum { MAX_SIZE = 96 };

	//! Persistence level of availabilities/requests/grants.
	enum Persistence {
		CANCEL = 0, FRAME1, FRAME2, FRAME4,
		FRAME8, FRAME32, FRAME128, FOREVER };

	//! Direction for availabilities IEs.
	enum Direction { UNAVAILABLE = 0, TX_ONLY, RX_ONLY, AVAILABLE };

	//! Request data structure (3 bytes).
	struct ReqIE {
		//! Destination NodeID (IRL, LinkID - 8 bits).
		WimaxNodeId nodeId_;
		//! Demand level (8 bits).
		unsigned char level_;
		//! Demand persistence (3 bits).
		Persistence persistence_;

		//! Return the size (in bytes) of this IE.
		static unsigned int size () { return 3; }
	};
	
	//! Availability data structure (4 bytes).
	struct AvlIE {
		//! Start frame number. IRL = 8 bits. Here we use the full frame number.
		unsigned int frame_;
		//! Minislot start (8 bits).
		unsigned char start_;
		//! Minislot range (7 bits).
		unsigned char range_;
		//! Direction (2 bits).
		Direction direction_;
		//! Persistence (3 bits).
		Persistence persistence_;
		//! Channel number (4 bits).
		unsigned char channel_;

		//! Return the size (in bytes) of this IE.
		static unsigned int size () { return 4; }
	};

	//! Grant data structure (5 bytes).
	struct GntIE {
		//! Destination NodeID (IRL, LinkID - 8 bits).
		WimaxNodeId nodeId_;
		//! Start frame number. IRL = 8 bits. Here we use the full frame number.
		unsigned int frame_;
		//! Minislot start (8 bits).
		unsigned char start_;
		//! Minislot range (8 bits).
		unsigned char range_;
		//! Direction (1 bit). True = from requester to granter. Otherwise, false.
		bool fromRequester_;
		//! Persistence (3 bits).
		Persistence persistence_;
		//! Channel number (4 bits).
		unsigned char channel_;

		//! Return the size (in bytes) of this IE.
		static unsigned int size () { return 5; }
	};

	//! Neighbor coordination IE (3 bytes).
	struct NghIE {
		//! NodeID (8 bits, this field does not exist for information on itself).
		WimaxNodeId nodeId_;
		//! NextXmtMx (3 bits).
		unsigned char nextXmtMx_;
		//! XmtHoldoffExponent (5 bits).
		unsigned char xmtHoldoffExponent_;
		//! Next transmit time, in slots (does not exist IRL).
		unsigned int nextXmtTime_;
		//! Next transmit time, in seconds (does not exist IRL).
		double nextXmtTimeSec_;

		//! Return the size (in bytes) of this IE.
		static unsigned int size () { return 3; }
	};

	//! Allocation type.
	enum AllocationType { BASIC, CONTIGUOUS };

private:
	//! MAC header (always present). MSH-DSCH messages are never fragmented.
	WimaxMacHeader hdr_;
	//! Mesh subheader = transmitter NodeId (16 bits).
	WimaxNodeId src_;
	
	//! Sequence counter (6 bits).
	unsigned int nseq_;
	//! Number of requests (4 bits).
	unsigned int nreq_;
	//! Number of availabilities (4 bits).
	unsigned int navl_;
	//! Number of grants (6 bits).
	unsigned int ngnt_;
	//! Number of advertised neighbors (8 bits) not including itself.
	unsigned int nngh_;

	//! List of availabilities.
	std::list<AvlIE> avl_;
	//! List of requests.
	std::list<ReqIE> req_;
	//! List of grants.
	std::list<GntIE> gnt_;
	//! List of neighbors.
	std::list<NghIE> ngh_;

	//! Information on myself, about the distributed election mechanism.
	NghIE myself_;

	//! Allocation type. Changed via allocationType(). Default = BASIC.
	static AllocationType allocationType_;
	
public:
	//! Create an empty MSH-DSCH message.
	WimshMshDsch () {
		hdr_.crc() = true;
		hdr_.fragmentation() = false;
		hdr_.length() = 15;  // includes fixed fields
		// the hdr_.cid() is set to management values by default
		nseq_ = 0;
		nreq_ = 0;
		navl_ = 0;
		ngnt_ = 0;
		nngh_ = 0;
	}
	//! Return the current message size (in bytes).
	unsigned int size () { return hdr_.length(); }
	//! Return the available space (in bytes).
	unsigned int remaining () { return MAX_SIZE - size(); }

	//! Return the MAC header.
	WimaxMacHeader hdr () { return hdr_; }
	//! Return the transmitter NodeID (in the mesh subheader).
	WimaxNodeId& src () { return src_; }

	//! Add an availabilities IE. Return the available space (in bytes).
	unsigned int add (AvlIE x) {
		if ( allocationType_ == BASIC ) {
			hdr_.length() += AvlIE::size(); avl_.push_front(x);
		} else if ( allocationType_ == CONTIGUOUS ) addContiguous (x);
		return remaining(); }
	//! Add a request IE. Return the available space (in bytes).
	unsigned int add (ReqIE x) {
		hdr_.length() += ReqIE::size(); req_.push_front(x); return remaining(); }
	//! Add a grants IE. Return the available space (in bytes).
	unsigned int add (GntIE x) {
		if ( allocationType_ == BASIC ) {
			hdr_.length() += GntIE::size(); gnt_.push_front(x);
		} else if ( allocationType_ == CONTIGUOUS ) addContiguous (x);
		return remaining(); }
	//! Add a neighbors IE. Return the available space (in bytes).
	unsigned int add (NghIE x) {
		hdr_.length() += NghIE::size(); ngh_.push_front(x); return remaining(); }

	//! Get the distribute election information on myself.
	NghIE& myself () { return myself_; }
	//! Get the list of availabilities IEs.
	std::list<AvlIE>& avl () { return avl_; }
	//! Get the list of requests IEs.
	std::list<ReqIE>& req () { return req_; }
	//! Get the list of grants IEs.
	std::list<GntIE>& gnt () { return gnt_; }
	//! Get the list of neighbors IEs.
	std::list<NghIE>& ngh () { return ngh_; }

	//! Convert a number of minislots into a pair <level, persistence> for req.
	static void slots2level (unsigned int N, unsigned int minislots,
			unsigned char& level, Persistence& persistence);

	//! Get/set the allocation type.
	static AllocationType& allocationType () { return allocationType_; }

	//! Convert the persistence into a number of frames (except FOREVER).
	static unsigned int pers2frames (Persistence p) {
		return ( p == CANCEL ) ? 0 :
			( p == FRAME1 ) ? 1 :
			( p == FRAME2 ) ? 2 :
			( p == FRAME4 ) ? 4 :
			( p == FRAME8 ) ? 8 :
			( p == FRAME32 ) ? 32 :
			( p == FRAME128 ) ? 128 : UINT_MAX; }

protected:
	//! Add a grant IE with contiguous allocation.
	void addContiguous (GntIE& x);
	//! Add an availability IE with contiguous allocation.
	void addContiguous (AvlIE& x);
};

//! MSH-NCFG control message. We assume coordinated message scheduling.
struct WimshMshNcfg {
public:
	//! Maximum number of advertised neighbors.
	enum { MAX_NGH = 255 };
	//! Maximum size of the MSH-NCFG message (in bytes).
	enum { MAX_SIZE = 96 };
	//! MSH-NCFG message types.
	/*!
	 * The following MSH-NCFG message types are implemented:
	 * - The NET_ENTRY_OPEN message is sent by the sponsor node in
	 *   response to an MSH-NENT REQUEST message.
	 * - The CHALLENGE message is sent by the new node to a neighbor
	 *   with which it is trying to establish a logical link.
	 * - The RESPONSE message is sent by the neighbor in response to
	 *   an MSH-NCFG CHALLENGE message.
	 * - The ACK message is sent by the new node to a neighbor with which
	 *   it is trying to establish a logical link to complete the procedure.
	 */
	enum Type { NET_ENTRY_OPEN, CHALLENGE, RESPONSE, ACK };

	//! Neighbor coordination IE (3 bytes).
	struct NghIE {
		//! NodeID (8 bits, this field does not exist for information on itself).
		WimaxNodeId nodeId_;
		//! NextXmtMx (3 bits).
		unsigned char nextXmtMx_;
		//! XmtHoldoffExponent (5 bits).
		unsigned char xmtHoldoffExponent_;
		//! Next transmit time (does not exist IRL).
		unsigned int nextXmtTime_;

		//! Return the size (in bytes) of this IE.
		static unsigned int size () { return 3; }
	};

private:
	//! MAC header (always present). MSH-NCFG messages are never fragmented.
	WimaxMacHeader hdr_;
	//! Mesh subheader = transmitter NodeId (16 bits).
	WimaxNodeId src_;
	
	//! Number of advertised neighbors (8 bits) not including itself.
	unsigned int nngh_;

	//! List of neighbors.
	std::list<NghIE> ngh_;

	//! Information on myself, about the distributed election mechanism.
	NghIE myself_;
	
	// MSH-NCFG message type.
	Type type_;

public:
	//! Create an empty MSH-NCFG message.
	WimshMshNcfg () {
		hdr_.crc() = true;
		hdr_.fragmentation() = false;
		hdr_.length() = 20;  // includes fixed fields  :XXX: fix to the real value
		// the hdr_.cid() is set to management values by default
		nngh_ = 0;
		type_ = NET_ENTRY_OPEN;
	}
	//! Return the current message size (in bytes).
	unsigned int size () { return hdr_.length(); }
	//! Return the available space (in bytes).
	unsigned int remaining () { return MAX_SIZE - size(); }

	//! Return the MAC header.
	WimaxMacHeader& hdr () { return hdr_; }
	//! Return the transmitter NodeID (in the mesh subheader).
	WimaxNodeId& src () { return src_; }
	//! Get/set the message type.
	Type& type () { return type_; }

	//! Add a neighbors IE. Return the available space (in bytes).
	unsigned int add (NghIE x) {
		hdr_.length() += NghIE::size(); ngh_.push_front(x); return remaining(); }

	//! Get the distribute election information on myself.
	NghIE& myself () { return myself_; }
	//! Get the list of neighbors IEs.
	std::list<NghIE>& ngh () { return ngh_; }
};

//! MSH-NENT control message. We assume coordinated message scheduling.
struct WimshMshNent {
public:
	//! MSH-NENT message types.
	/*!
	 * The following MSH-NENT message types are implemented:
	 * - The REQUEST message is sent by a new node to its sponsor node to
	 *   open a sponsor channel.
	 * - The ACK message is sent by a new node to complete the sponsor channel
	 *   open procedure.
	 */
	enum Type { REQUEST, ACK };
	//! Maximum size of the MSH-NENT message (in bytes).
	enum { MAX_SIZE = 96 };

private:
	//! MAC header (always present). MSH-NENT messages are never fragmented.
	WimaxMacHeader hdr_;
	//! Mesh subheader = transmitter NodeId (16 bits).
	WimaxNodeId src_;
	//! MSH-NENT message type.
	Type type_;

public:
	//! Create an empty MSH-NENT message.
	WimshMshNent () {
		hdr_.crc() = true;
		hdr_.fragmentation() = false;
		hdr_.length() = 20;  // includes fixed fields  :XXX: fix to the real value
		// the hdr_.cid() is set to management values by default
		type_ = REQUEST;
	}
	//! Return the current message size (in bytes).
	unsigned int size () { return hdr_.length(); }
	//! Return the available space (in bytes).
	unsigned int remaining () { return MAX_SIZE - size(); }

	//! Return the MAC header.
	WimaxMacHeader& hdr () { return hdr_; }
	//! Return the transmitter NodeID (in the mesh subheader).
	WimaxNodeId& src () { return src_; }
	//! Get/set the MSH-NENT message type.
	Type& type () { return type_; }
};

//! WiMAX mesh burst of MAC PDUs.
/*!
  A burst of PDUs is either a list of MAC PDUs, or a container for one
  MSH-DSCH message, according to the burst type.
  If you want to copy a burst, then use the copy() function.
  On the other hand, the destructor can be used safely to delete a burst.
  In any case, the ns2 packets, nor the MSH-DSCH message, are *not* copied.
  If you really want to deallocate the ns2 packets, which are encapsulated into
  each SDU, you have to do it manually.
  The rationale behind this is that we want only *one* copy of each
  ns2 packet to travel from the source node to the destination, to save
  both memory (a packet) and time (copying a packet).
  However, care must be taken to destroy these objects. We assume that
  only the final recipient of the communication destroys ns2 packets,
  which should be safe enough with unicast communications.
  */
struct WimshBurst {
public:
	typedef std::list<WimaxPdu*> List;

private:
	//--------------------------------------------------------//
	//-- Payload of the burst, depending on the burst type. --//
	//--------------------------------------------------------//

	//! List of PDUs (if type == wimax::DATA).
	List pdus_;
	//! Pointer to the MSH-DSCH message (if type == wimax::MSHDSCH).
	WimshMshDsch* mshDsch_;

	//! Pointer to the MSH-DSCH message (if type == wimax::MSHNCFG).
	WimshMshNcfg* mshNcfg_;

	//! Pointer to the MSH-DSCH message (if type == wimax::MSHNENT).
	WimshMshNent* mshNent_;
	
	//------------------//
	//-- Other fields --//
	//------------------//

	//! Transmission time of this burst (in seconds, including preambles).
	double txtime_;
	//! True if the whole burst is corrupt.
	bool error_;
	//! Burst profile.
	wimax::BurstProfile profile_;
	//! PDU burst type.
	wimax::BurstType type_;
	//! Size, in bytes.
	unsigned int size_;
	//! Source node ID.
	WimaxNodeId src_;

public:
	//! Build an empty burst of PDUs.
	WimshBurst () {
		error_ = false; size_ = 0; mshDsch_ = 0; mshNcfg_ = 0; mshNent_ = 0;
		profile_ = wimax::QPSK_1_2; type_ = wimax::DATA; }
	//! Destroy everything ;)
	~WimshBurst ();
	//! Allocate a copy of this burst and return a pointer to it.
	/*!
	  The ns2 packets (ie. the SDUs' payload) are *not* copied.
	  On the other hand, the MSH-DSCH message, if any, is copied.
	  */
	WimshBurst (const WimshBurst& obj);

	//! Add a PDU to the burst.
	void addData (WimaxPdu* pdu, bool tail = true ) {
		if ( tail ) pdus_.push_back (pdu); else pdus_.push_front (pdu);
		size_ += pdu->size(); }
	//! Remove a PDU from the burst. The PDU must be freed afterwards.
	WimaxPdu* pdu () {
		if ( pdus_.empty() ) return 0; 
		WimaxPdu* tmp = pdus_.front(); pdus_.pop_front(); return tmp; }

	//! Return the list of PDUs.
	List& pdus () { return pdus_; }
	//! Return the number of PDUs in the burst.
	unsigned int npdus () { return pdus_.size(); }

	//! Get a pointer to the MSH-DSCH message, if any.
	WimshMshDsch*& mshDsch () { return mshDsch_; }
	//! Add an MSH-DSCH message to the burst.
	void addMshDsch (WimshMshDsch* m);
	
	//! Get a pointer to the MSH-NCFG message, if any.
	WimshMshNcfg*& mshNcfg () { return mshNcfg_; }
	//! Add an MSH-NCFG message to the burst.
	void addMshNcfg (WimshMshNcfg* m);

	//! Get a pointer to the MSH-NENT message, if any.
	WimshMshNent*& mshNent () { return mshNent_; }
	//! Add an MSH-NENT message to the burst.
	void addMshNent (WimshMshNent* m);


	//! Get/set the transmission time (in seconds, including preambles).
	double& txtime () { return txtime_; }
	//! Get/set the error status of the whole burst.
	bool& error () { return error_; }
	//! Get/set the burst profile.
	wimax::BurstProfile& profile () { return profile_; }

	//! Get the burst type.
	wimax::BurstType& type () { return type_; }
	//! Get the burst size (in bytes).
	unsigned int size () { return size_; }

	//! Get/set the source node.
	WimaxNodeId& source () { return src_; }
private:
	void operator= (const WimshBurst&);
};

#endif /* __NS2_WIMSH_PACKET_H */
