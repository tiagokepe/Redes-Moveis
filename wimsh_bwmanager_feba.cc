#include <wimsh_bwmanager_feba.h>


#include <wimsh_mac.h>
#include <wimsh_topology.h>
#include <wimsh_packet.h>

#include <random.h>
#include <stat.h>

#include <iostream>
using namespace std;

WimshBwManagerFeba::WimshBwManagerFeba ( WimshMac* m) :
	WimshBwManager (m), wm_ (m)
{
}


int WimshBwManagerFeba::command (int argc, const char*const* argv) {
	return TCL_OK;
}


void WimshBwManagerFeba::recvMshDsch (WimshMshDsch* dsch){

	recvReq(dsch);
	recvGnt(dsch);
	recvAvl(dsch);
}


void WimshBwManagerFeba::recvReq(WimshMshDsch* dsch){

	/* Lista de requisições */
	std::list< WimshMshDsch::ReqIE >req = dsch->req();

	std::list< WimshMshDsch::ReqIE >::iterator it;

	int ngh_index = mac_->neigh2ndx(dsch->src());

	for( it = req.begin(); it != req.end(); it++)
	{
		if( it->nodeId_ == mac_->nodeId()  )
		{

			 neigh_[ngh_index].req_in_+= WimshMshDsch::pers2frames(it->persistence_) * mac_->slots2bytes(ngh_index, it->level_, false);
		}
	}
}

void WimshBwManagerFeba::recvGnt(WimshMshDsch* dsch){

	/* Lista de grants */
	std::list< WimshMshDsch::GntIE >gnt = dsch->gnt();

	std::list< WimshMshDsch::GntIE >::iterator it;

	for( it = gnt.begin(); it != gnt.end(); it++ )
	{
		//Indice deste vizinho
		unsigned int ngh_index = it->nodeId_;

		// Duração do frame
		unsigned int frame_range;
		unsigned int frame_start;
		
		realPersistence(it->frame_,it->persistence_,frame_start,frame_range);
		
		// Grant destinado a este nodo.
		if(  it->nodeId_ == mac_->nodeId() )
		{
			// É uma confirmação
			if ( it->fromRequester_ )
			{
				neigh_[ngh_index].cnf_in_ += frame_range * mac_->slots2bytes(ngh_index, it->range_, false);
			}
			else // É uma concessão
			{
				// O grant é transformado em uma confirmação e guardada.
				// Alteramos o destino desta confirmação agora proque temos esta informação na mensagem.
				it->nodeId_ = dsch->src();
				it->fromRequester_ = true;

				pending_confirmations.insert(*it);

				neigh_[ngh_index].gnt_out_+= frame_range * mac_->slots2bytes(ngh_index, it->range_, false);
			}
		}
		else  // Grant destinado a outro nodo
		{
			// Concessão
			if ( !(it->fromRequester_)   )
			{
				// Não podemos transmitir nos slots concedidos
				setSlots(self_tx_unavl_[it->channel_], frame_start, frame_range, it->start_, it->range_, true );
			}
			else if ( mac_->topology()->neighbors(it->nodeId_,mac_->nodeId()) ) // Confirmação
			{
				// Devemos ver se não é feita para outro vizinho nosso.
				// Neste caso já teriamos cuidado de seu grant.

				// O vizinho que confirmou não poderá transmitir em nenhum canal.
				// Nós não poderemos escutar neste canal
				for(unsigned int ch_index = 0; ch_index < mac_->nchannels(); ch_index++)
				{
					setSlots(neigh_tx_unavl_[ngh_index][ch_index],frame_start,frame_range,it->start_,it->range_,true);
				}
				setSlots(self_rx_unavl_,frame_start,frame_range,it->start_,it->range_,true);
			}
		}
	}
}

void WimshBwManagerFeba::recvAvl(WimshMshDsch* dsch){
	/* Lista de indisponibilidades */
	std::list< WimshMshDsch::AvlIE >avl = dsch->avl();

	std::list< WimshMshDsch::AvlIE >::iterator it;


	for( it = avl.begin(); it != avl.end(); it++ )
	{

	}
}

void WimshBwManagerFeba::schedule (WimshMshDsch* dsch){
}

void WimshBwManagerFeba::initialize (){
	
	int nneighbors = mac_->nneighs();
	neigh_.resize(nneighbors);

	neigh_tx_unavl_.resize(nneighbors);

	busy_.resize(HORIZON);
	for(int fr_index = 0; fr_index < HORIZON; fr_index++)
	{
		busy_[fr_index].reset();
	}

	// Para cada vizinho
	for(int ngh_index=0; ngh_index < nneighbors; ngh_index++)
	{
		neigh_tx_unavl_[ngh_index].resize(mac_->nchannels());
		//Para cada canal
		for(unsigned int ch_index=0; ch_index < mac_->nchannels(); ch_index++)
		{
			neigh_tx_unavl_[ngh_index][ch_index].resize(HORIZON);

			// Para cada frame
			for( int fr_index = 0; fr_index < HORIZON; fr_index++)
			{
				neigh_tx_unavl_[ngh_index][ch_index][fr_index].reset();
			}
		}
	}

	self_rx_unavl_.resize(mac_->nchannels());
	self_tx_unavl_.resize(mac_->nchannels());
	//Para cada canal
	for(unsigned int ch_index=0; ch_index < mac_->nchannels(); ch_index++)
	{
		self_rx_unavl_[ch_index].resize(HORIZON);
		self_tx_unavl_[ch_index].resize(HORIZON);
		// Para cada frame
		for(int fr_index = 0; fr_index < HORIZON; fr_index++)
		{
			self_rx_unavl_[ch_index][fr_index].reset();
			self_tx_unavl_[ch_index][fr_index].reset();
		}
	}


}
	
void WimshBwManagerFeba::backlog (WimaxNodeId src, WimaxNodeId dst, unsigned char prio, WimaxNodeId nexthop, unsigned int bytes){
}
	
void WimshBwManagerFeba::backlog (WimaxNodeId nexthop, unsigned int bytes){
}

void WimshBwManagerFeba::sent (WimaxNodeId nexthop, unsigned int bytes){
}

void WimshBwManagerFeba::realPersistence(unsigned int frame_start, WimshMshDsch::Persistence frame_range, unsigned int &actual_frame_start, unsigned int &actual_frame_range){
	actual_frame_range = WimshMshDsch::pers2frames(frame_range);

	actual_frame_range -= ( frame_start > mac_->frame() )? 0 : mac_->frame() - frame_start  ;

	if ( actual_frame_range < 0  ) actual_frame_range = 0;

	actual_frame_start = ( frame_start >= mac_->frame() )? frame_start : mac_->frame();
}
