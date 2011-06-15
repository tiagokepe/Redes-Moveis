#ifndef __NS2_WIMSH_BW_MANAGER_FEBA_H
#define __NS2_WIMSH_BW_MANAGER_FEBA_H


#include <wimsh_weight_manager.h>
#include <wimsh_bwmanager.h>
#include <wimsh_packet.h>
#include <wimsh_mac.h>

#define ROUND_DURATION 20



class WimshBwManagerFeba : public WimshBwManager {
	
	typedef std::vector< std::bitset< MAX_SLOTS > > Bitmap;


protected:
	struct NeighbourDesc {
	
		/* Número de bytes requisitados ao vizinho. */

		unsigned int req_in_;

		/* Concedidos. */

		unsigned int gnt_in_;

		/* Confirmados. */

		unsigned int cnf_in_;

		/* Compensação. Usado no DRR */

		unsigned int lag_in_;

		/* A serem requisitados */
		unsigned int backlog_;

		
		unsigned int req_out_;

		unsigned int gnt_out_;

		unsigned int cnf_out_;
		
		unsigned int lag_out_;

		NeighbourDesc() {
			req_in_=0;
			req_out_=0;
			gnt_in_=0;
			gnt_out_=0;
			lag_in_=0;
			lag_out_=0;
			backlog_=0;
		}


	};
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


	// Confirmações pendentes
	CircularList< WimshMshDsch::GntIE > pending_confirmations_;
	// Lista de disponibilidades a serem enviadas
	CircularList< WimshMshDsch::AvlIE > availabilities_;
	// Lista de fluxos ativos
	CircularList< wimax::LinkId > activeList_;

	Bitmap unconfirmedSlots_;



public:
	/* Construtor */
	WimshBwManagerFeba (WimshMac* m);
	~WimshBwManagerFeba () { }

	/* Weight Manager */
	WimshWeightManager wm_;


	/* Interface com o Tcl */
	int command (int argc, const char*const* argv);


	void recvMshDsch (WimshMshDsch* dsch);

	void schedule (WimshMshDsch* dsch);

	void initialize ();
	
	void backlog (WimaxNodeId src, WimaxNodeId dst, unsigned char prio, WimaxNodeId nexthop, unsigned int bytes);
	
	void backlog (WimaxNodeId nexthop, unsigned int bytes);

	void sent (WimaxNodeId nexthop, unsigned int bytes);
	
	void received (WimaxNodeId src, WimaxNodeId dst, unsigned char prio, WimaxNodeId source, unsigned int bytes) {
		wm_.flow (src, dst, prio, source, wimax::IN); 
	}
	//! O quantm será usado para o tratamento da lista ativa, que é baseado no Deficit Round Robin.
	unsigned int quantum (unsigned int ndx, wimax::LinkDirection dir) {
		return (unsigned int) (ceil(wm_.weight (ndx, dir) * ROUND_DURATION)); }

private:
	void recvReq(WimshMshDsch* dsch);
	void recvGnt(WimshMshDsch* dsch);
	void recvAvl(WimshMshDsch* dsch);
	void realPersistence(unsigned int frame_start, WimshMshDsch::Persistence frame_range, unsigned int &actual_frame_start, unsigned int &actual_frame_range);
	WimshMshDsch::AvlIE createAvl(WimshMshDsch::GntIE gnt,WimshMshDsch::Direction dir);
};

#endif

