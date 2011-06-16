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

			 // Inserimos este vizinho na lista de conexões ativas,
			 // caso ele ainda não esteja lá
			 if (   neigh_[ngh_index].req_in_ > neigh_[ngh_index].gnt_in_ && !( activeList_.find(wimax::LinkId(ngh_index,wimax::IN)) )   )
				 activeList_.insert(wimax::LinkId(ngh_index,wimax::IN));
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
				// Incrementamos o número de bytes de entrada confirmados.
				neigh_[ngh_index].cnf_in_ += frame_range * mac_->slots2bytes(ngh_index, it->range_, false);

				// Devemos escutar neste canal

				setSlots(channel_,frame_start,frame_range,it->start_,it->range_,it->channel_);


			}
			else // É uma concessão
			{
				// O grant é transformado em uma confirmação e guardada.
				// Alteramos o destino desta confirmação agora proque temos esta informação na mensagem.
				it->nodeId_ = dsch->src();
				it->fromRequester_ = true;

				pending_confirmations_.insert(*it);

				neigh_[ngh_index].gnt_out_+= frame_range * mac_->slots2bytes(ngh_index, it->range_, false);

				// Slots não confirmados se tornam indisponíveis

				setSlots(unconfirmedSlots_,frame_start,frame_range,it->start_,it->range_,true);
			}
		}
		else  // Grant destinado a outro nodo
		{
			// Concessão
			if ( !(it->fromRequester_)   )
			{
				// Se o destinatário deste grant é nosso vizinho, ele não poderá transmitir
				// durante estes slots de tempo em nenhum canal. Sem grants a ele.
				if (  mac_->topology()->neighbors(it->nodeId_,mac_->nodeId()) )
				{
					for(unsigned int ch_index; ch_index < mac_->nchannels();ch_index++)
					{
						setSlots(neigh_tx_unavl_[ch_index],frame_start,frame_range,it->start_,it->range_,true);
					}
				}

				// Não poderemos dar grants a nenhum outro vizinho do concessor neste canal.
				std::vector< WimaxNodeId > mutual_neighbors;
				mac_->topology()->neighbors(dsch->src(),mutual_neighbors);
				std::vector< WimaxNodeId >::iterator nid_it;
				for(nid_it = mutual_neighbors.begin(); nid_it != mutual_neighbors.end();nid_it++)
				{
					// Se somos vizinhos
					if( mac_->topology()->neighbors(mac_->nodeId(),*nid_it) )
					{
						int mutual_nghindex = mac_->neigh2ndx(*nid_it);

						setSlots(neigh_tx_unavl_[mutual_nghindex][it->channel_],frame_start,frame_range,it->start_,it->range_,true);
					}
				}

				// Não podemos transmitir nos slots concedidos. Não confirmaremos nenhum grant neste canal.
				setSlots(self_tx_unavl_[it->channel_], frame_start, frame_range, it->start_, it->range_, true );

				// Criaremos um aviso de indisponibilidade:
				WimshMshDsch::AvlIE avl;

				avl = createAvl(*it,WimshMshDsch::RX_ONLY);
				availabilities_.insert(avl);
			}
			else if ( mac_->topology()->neighbors(it->nodeId_,mac_->nodeId()) ) // Confirmação
			{
				// O destinatário desta confirmação não pode ser nosso vizinho.
				// Neste caso já teriamos cuidado de seu grant.

				// O vizinho que confirmou não poderá transmitir em nenhum canal.
				for(unsigned int ch_index = 0; ch_index < mac_->nchannels(); ch_index++)
				{
					setSlots(neigh_tx_unavl_[ngh_index][ch_index],frame_start,frame_range,it->start_,it->range_,true);
				}

				// Nós não poderemos escutar neste canal
				setSlots(self_rx_unavl_[it->channel_],frame_start,frame_range,it->start_,it->range_,true);
			}
		}
	}
}

void WimshBwManagerFeba::recvAvl(WimshMshDsch* dsch){
	/* Lista de indisponibilidades */
	std::list< WimshMshDsch::AvlIE >avl = dsch->avl();

	std::list< WimshMshDsch::AvlIE >::iterator it;

	// Índice do meu vizinho.
	int ngh_index;

	// Dados atualizados do meu frame
	unsigned int frame_start;
	unsigned int frame_range;

	// Atualizamos nossa estrutura de dados de acordo com as disponibilidades recebidas.
	for( it = avl.begin(); it != avl.end(); it++ )
	{
		if ( ( it->direction_ == WimshMshDsch::RX_ONLY) || (it->direction_ == WimshMshDsch::UNAVAILABLE ) )
		{
			ngh_index = mac_->neigh2ndx(dsch->src());
			realPersistence(it->frame_,it->persistence_,frame_start,frame_range);
			setSlots(neigh_tx_unavl_[ngh_index][it->channel_],frame_start,frame_range,it->start_,it->range_,true);
		}

	}
}

void WimshBwManagerFeba::schedule (WimshMshDsch* dsch){

	//  Encaixar as disponibilidades
	// availabilities(dsch);

	// Confirmar concessões
	// confirm(dsch);

	// Concessões extras para requisições que não puderam ser atendidas.
	//regrant(dsch);

	// Varre a lista de ativos para tratar requisições e concessões.
	//requestAndGrant(dsch);


}

void WimshBwManagerFeba::requestAndGrant(WimshMshDsch* dsch){

	// Enquanto a lista de conexões ainda estiver ativa, continue
	while( !activeList_.empty() )
	{
		wimax::LinkDirection direction = activeList_.current().dir_;
		unsigned int ngh_index = activeList_.current().ndx_;

		// Se for um fluxo de entrada, eu devo conceder
		// Ver grant(i) no artigo do FEBA
		if( direction == wimax::IN ){

			// Se a mensagem estiver cheia, eu não incluo mais grants
			if ( dsch->remaining() > WimshMshDsch::GntIE::size()  )
				break;


			// lag_i_in <- min{lag_i_in + quantum, req_i_in - gnt_i_in}
			unsigned int pending_bytes = neigh_[ngh_index].req_in_ - neigh_[ngh_index].gnt_in_;
			neigh_[ngh_index].lag_in_+=quantum(ngh_index,wimax::IN);
			neigh_[ngh_index].lag_in_= ( neigh_[ngh_index].lag_in_ > pending_bytes) ? pending_bytes : neigh_[ngh_index].lag_in_;


			// Adiciono grants de acordo com o DRR
			while ( ( dsch->remaining() > WimshMshDsch::GntIE::size() ) && (neigh_[ngh_index].lag_in_ > 0 ) ){

				WimshMshDsch::GntIE gnt;

				//gnt = fit(ngh_index, initial horizon, limit horizon );

				/*
				 * Se não achamos nenhum espaço no horizonte de alocação,
				 * então não há mais o que alocar.
				 */
				if ( gnt.range_ <= 0 ) break;

				dsch->add(gnt);

				// Precisamos descobrir o número de bytes concedidos
				unsigned int frame_range = WimshMshDsch::pers2frames(gnt.persistence_);
				unsigned int bytes_granted = frame_range * mac_->slots2bytes(ngh_index,gnt.range_,true);

				//gnt_i_in = gnt_i_in + granted
				neigh_[ngh_index].gnt_in_+=bytes_granted;

				// lag_i_in = lag_i_in - granted
				neigh_[ngh_index].lag_in_ = ( bytes_granted > neigh_[ngh_index].lag_in_ ) ? 0 : neigh_[ngh_index].lag_in_ - bytes_granted;

				// Não podemos mais receber nenhuma transmissão neste canal
				setSlots(busy_, gnt.frame_, frame_range, gnt.start_, gnt.range_, true)
			}

			// if ( lag_i_in > lag_max ) terminate
			if ( neigh_[ngh_index].lag_in_ > LAG_MAX )
				break;

			/*
			 * Precisamos mover o ponteiro de nossa lista ativa,
			 * há dois casos:
			 * - As requisições já foram satisfeitas
			 * - ainda não
			 */

			if ( neigh_[ngh_index].gnt_in_ >= neigh_[ngh_index].req_in_ )
			{
				neigh_[ngh_index].lag_in_ = 0;
				activeList_.erase();
			}
			else
			{
				activeList_.move();
			}
		} // Se for um fluxo de saída, eu devo requisitar
		else{


		}

	}
}

void WimshBwManagerFeba::initialize (){
	
	int nneighbors = mac_->nneighs();
	neigh_.resize(nneighbors);

	neigh_tx_unavl_.resize(nneighbors);

	busy_.resize(HORIZON);
	unconfirmedSlots_.resize(HORIZON);

	for(int fr_index = 0; fr_index < HORIZON; fr_index++)
	{
		busy_[fr_index].reset();
		unconfirmedSlots_[fr_index].reset();
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
/*
 * Desejamos enviar uma quantidade de bytes a um determinado nodo
 */
void WimshBwManagerFeba::backlog (WimaxNodeId src, WimaxNodeId dst, unsigned char prio, WimaxNodeId nexthop, unsigned int bytes){

	// Índice do meu vizinho
	const unsigned int ndx = mac_->neigh2ndx(nexthop);

	// Adicionamos este fluxo ao pesador de fluxos
	wm_.flow(src,dst,prio,ndx,wimax::IN);


	neigh_[ndx].backlog_+=bytes;

	// Este fluxo precisa ser adicionado se não existir, para que o alocador em Deficit Round Robin possa ser usado.
	if (  !activeList_.find(wimax::LinkId(ndx, wimax::IN))  )
		activeList_.insert(wimax::LinkId(ndx,wimax::IN));
}
	
void WimshBwManagerFeba::backlog (WimaxNodeId nexthop, unsigned int bytes){

	// Índice do meu vizinho
	const unsigned int ndx = mac_->neigh2ndx(nexthop);

	wm_.flow(ndx, wimax::IN);

	neigh_[ndx].backlog_+=bytes;

	if (  !activeList_.find(wimax::LinkId(ndx, wimax::IN))  )
		activeList_.insert(wimax::LinkId(ndx,wimax::IN));
}

void WimshBwManagerFeba::sent (WimaxNodeId nexthop, unsigned int bytes){
}

void WimshBwManagerFeba::realPersistence(unsigned int frame_start, WimshMshDsch::Persistence frame_range, unsigned int &actual_frame_start, unsigned int &actual_frame_range){
	actual_frame_range = WimshMshDsch::pers2frames(frame_range);

	actual_frame_range -= ( frame_start > mac_->frame() )? 0 : mac_->frame() - frame_start  ;

	if ( actual_frame_range < 0  ) actual_frame_range = 0;

	actual_frame_start = ( frame_start >= mac_->frame() )? frame_start : mac_->frame();
}

WimshMshDsch::AvlIE WimshBwManagerFeba::createAvl(WimshMshDsch::GntIE gnt, WimshMshDsch::Direction dir) {
	WimshMshDsch::AvlIE avl;

	avl.channel_ = gnt.channel_;
	avl.frame_ = gnt.frame_;
	avl.persistence_ = gnt.persistence_;
	avl.start_ = gnt.start_;
	avl.range_ = gnt.range_;
	avl.direction_ = dir;

	return avl;
}

