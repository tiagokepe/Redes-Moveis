

#ifndef __NS2_WIMSH_BW_MANAGER_FEBA_H
#define __NS2_WIMSH_BW_MANAGER_FEBA_H

#include <wimsh_weight_manager.h>
#include <wimsh_bwmanager.h>
#include <wimsh_packet.h>
#include <wimsh_mac.h>

#include <rng.h>

//! Fair round robin bandwidth manager for 802.16. Single-radio only.
/*!
  :TODO: more documentation. Much more.
  */
class WimshBwManagerFeba : public WimshBwManager {
	//! Typedef for an array of bitsets (representing slots within frames).
	typedef std::vector< std::bitset<MAX_SLOTS> > Bitmap;

protected:
	struct NeighbourDesc {

		/* Número de bytes requisitados ao vizinho. */

		unsigned int req_in_;

		/* Concedidos. */

		unsigned int gnt_in_;

		/* Confirmados. */

		unsigned int cnf_in_;

		/* Compensação. Usado no DRR */

		unsigned int def_in_;

		/* A serem requisitados */
		unsigned int backlog_;


		unsigned int req_out_;

		unsigned int gnt_out_;

		unsigned int cnf_out_;

		unsigned int def_out_;

		NeighbourDesc() {
			req_in_=0;
			req_out_=0;
			gnt_in_=0;
			gnt_out_=0;
			def_in_=0;
			def_out_=0;
			backlog_=0;
		}


	};

	bool deficitOverflow_;
	bool ddTimer_;
	bool ddTimeout_;

	// Lista de fluxos ativos
	CircularList< wimax::LinkId > activeList_;


	// Descritor geral de vizinhos.
	std::vector< NeighbourDesc > neigh_;

	//Mapa de indisponibilidades de transmissão para cada vizinho, usado para conceder banda:
	std::vector< vector< Bitmap > > neigh_tx_unavl_;

	// Mapa de indisponibilidades deste nodo:
	// para transmissão, usado para confirmar banda:
	std::vector< Bitmap >  self_tx_unavl_;
	// para escuta:
	std::vector< Bitmap >  self_rx_unavl_;


	Bitmap busy_;

	//! Two-dimension bitmap representing the slots unconfirmed by this node.
	/*!
	  Updated when a granted addressed to this node is received.
	  Used to confirm only minislots that have not been reserved
	  by other nodes. As soon as slot is confirmed, it is marked as
	  busy (ie. the corresponding bit in busy_ is set to true) so that
	  the same slot can never be confirmed twice.

	  The F-th entry of this data structure is reset by handle(), where
	  F is the frame number (modulo HORIZON) of the current data frame
	  at the end of each frame.
	  */
	Bitmap unconfirmedSlots_;

	//! List of unconfirmed grants directed to this node.
	std::list<WimshMshDsch::GntIE> unconfirmed_;

	//! List of pending availabilities to send out.
	std::list<WimshMshDsch::AvlIE> availabilities_;



	unsigned int regrantOffset_;
	unsigned int regrantDuration_;
	bool sameRegrantHorizon_;

	bool regrantEnabled_;

	bool fairRegrant_;

	bool fairGrant_;

	bool fairRequest_;

	unsigned int maxDeficit_;

	unsigned int maxBacklog_;


	//! Weight manager.
	WimshWeightManager wm_;

	//! Sum of the quanta, in bytes.
	/*!
	  Each quantum for bandwidth requesting/granting is computed as this
	  value times the weight of the input/output link.

	  This value is set via a Tcl command.
	  */
	unsigned int roundDuration_;

	//! Random number generator to pick up channel/frame/slot when granting.
	RNG grantFitRng;

	//! True if the starting channel is picked up randomly when granting.
	bool grantFitRandomChannel_;


	//! Minimum grant size, in OFDM symbols, preamble not included. Default = 1.
	unsigned int minGrant_;

public:

	WimshBwManagerFeba (WimshMac* m);

	~WimshBwManagerFeba () { }

	void recvMshDsch (WimshMshDsch* dsch);

	void schedule (WimshMshDsch* dsch);

	void initialize ();

	void backlog (WimaxNodeId src, WimaxNodeId dst, unsigned char prio,
			WimaxNodeId nexthop, unsigned int bytes);

	void backlog (WimaxNodeId nexthop, unsigned int bytes);

	void sent (WimaxNodeId nexthop, unsigned int bytes);

	void received (WimaxNodeId src, WimaxNodeId dst, unsigned char prio,
			WimaxNodeId source, unsigned int bytes) {
		wm_.flow (src, dst, prio, source, wimax::IN); }

	int command (int argc, const char*const* argv);

private:

	void rcvGrants (WimshMshDsch* dsch);
	void rcvAvailabilities (WimshMshDsch* dsch);
	void rcvRequests (WimshMshDsch* dsch);
   void confirm (WimshMshDsch* dsch);
   void availabilities (WimshMshDsch* dsch);
   void requestGrant (WimshMshDsch* dsch);
	void regrant (WimshMshDsch* dsch);
	void realPersistence (unsigned int frame_start, WimshMshDsch::Persistence frame_range,
			unsigned int& actual_frame_start, unsigned int& actual_frame_range);

	WimshMshDsch::GntIE grantFit (unsigned int ndx, unsigned int bytes,
			unsigned int minFrame, unsigned int maxFrame);

	void confFit ( unsigned int fstart, unsigned int frange,
			unsigned int mstart, unsigned int mrange,
			WimshMshDsch::GntIE& gnt);

	unsigned int quantum (unsigned int ndx, wimax::LinkDirection dir) {
		return (unsigned int) (ceil(wm_.weight (ndx, dir) * roundDuration_)); }

};

#endif
