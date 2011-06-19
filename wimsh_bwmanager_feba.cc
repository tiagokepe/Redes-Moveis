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

				pending_confirmations_.push_back(*it);

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
				availabilities_.push_back(avl);
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
	 availabilities(dsch);

	// Confirmar concessões
	confirm(dsch);

	// Concessões extras para requisições que não puderam ser atendidas.
	//regrant(dsch);

	// Varre a lista de ativos para tratar requisições e concessões.
	requestAndGrant(dsch);

}



void WimshBwManagerFeba::availabilities(WimshMshDsch* dsch) {


	// Adicione o maior número de disponibilidades possíveis
	while ( ! availabilities_.empty() ) {

		// Se não há mais espaço, paramos
		if ( dsch->remaining() < WimshMshDsch::AvlIE::size() ) break;

		WimshMshDsch::AvlIE avl = availabilities_.front();
		availabilities_.pop_front();

		// Incluimos a disponibilidade na mensagem, a não ser que esta informação não seja mais válida.
		// Corresponda a um frame que não exite mais.
		if ( mac_->frame() <=
				avl.frame_ + WimshMshDsch::pers2frames (avl.persistence_) ) {
			dsch->add (avl);
		}
	}
}

void WimshBwManagerFeba::confirm (WimshMshDsch * dsch  ){

	// Devemos confirmar o maior numero possível de concessões
	while( !pending_confirmations_.empty() ) {

		// Se não há mais espaço na mensagem, terminamos
		if ( dsch->remaining() < WimshMshDsch::GntIE::size() ) break;


		WimshMshDsch::GntIE gnt;


		// Número de bytes confirmados
		unsigned int confirmed = 0;

		gnt = pending_confirmations_.front();
		pending_confirmations_.pop_front();

		unsigned int frame_start;
		unsigned int frame_range;

		realPersistence(gnt.frame_,gnt.persistence_,frame_start, frame_range);

		unsigned int ngh_index = mac_->neigh2ndx(gnt.nodeId_);


		while ( dsch->remaining() >= WimshMshDsch::GntIE::size() ) {

			confFit(frame_start, frame_range, gnt.start_, gnt.range_, gnt);


			// Se não há mais bytes a serem confirmados, termine
			if ( gnt.range_ == 0 )
				break;

			dsch->add(gnt);

			unsigned int actual_frame_start;
			unsigned int actual_frame_range;

			realPersistence(gnt.frame_,gnt.persistence_,actual_frame_start,actual_frame_range);

			// Computamos o valor dos bytes confirmados
			confirmed += actual_frame_range * mac_->slots2bytes (ngh_index, gnt.range_, true);

			// Marcamos as estruturas de dados. Para avisar onde transmitiremos.
			setSlots(busy_,actual_frame_start,actual_frame_range,gnt.start_,gnt.range_,true);
			setSlots (grants_, actual_frame_start, actual_frame_range, gnt.start_, gnt.range_, true);
			setSlots (dst_, actual_frame_start, actual_frame_range, gnt.start_, gnt.range_, gnt.nodeId_);
			setSlots (channel_, actual_frame_start, actual_frame_range, gnt.start_, gnt.range_, gnt.channel_);
		}


		neigh_[ngh_index].cnf_out_ += confirmed;
		neigh_[ngh_index].cnf_out_ =	( neigh_[ngh_index].cnf_out_ < neigh_[ngh_index].gnt_out_ ) ?
			neigh_[ngh_index].cnf_out_ : neigh_[ngh_index].gnt_out_;



		// Se o número de requisições e menor que o de confirmações, a simulação é abortada.
		if ( neigh_[ngh_index].req_out_ < neigh_[ngh_index].cnf_out_ ) abort();

		// Os dados sobre os bytes desta confirmação são esquecidos.
		neigh_[ngh_index].gnt_out_ -= neigh_[ngh_index].cnf_out_;
		neigh_[ngh_index].req_out_ -= neigh_[ngh_index].cnf_out_;
		neigh_[ngh_index].cnf_out_ = 0;



	}

}



void WimshBwManagerFeba::confFit(unsigned int frame_start, unsigned int frame_range, unsigned int slot_start, unsigned int slot_range, WimshMshDsch::GntIE& gnt) {

	// Para cada frame
	for( unsigned int f = frame_start; f < frame_start + frame_range; f++) {
		unsigned int actual_frame = f % HORIZON;

		// Verificamos as impossibilidades de transmissão.
		const std::bitset<MAX_SLOTS> map = busy_[actual_frame] | self_tx_unavl_[gnt.channel_][actual_frame];

		// Para cada minislot
		for ( unsigned int slot = slot_start; slot < slot_start + slot_range; slot++ ) {

			if ( map[slot] == false ) {
				gnt.frame_ = f;
				gnt.start_ = slot;
				gnt.persistence_ = WimshMshDsch::FRAME1;

				// Procure o maior número de minislots
				for ( ; slot < ( slot_start + slot_range ) && map[slot] == false ; slot++ ) { }

				gnt.range_ = slot - gnt.start_;
				return;
			}
		}
	}

	// Se chegamos até aqui, retornamos um grant vazio. Não é possível conceder mais nada.
	gnt.range_ = 0;
}


void WimshBwManagerFeba::requestAndGrant(WimshMshDsch* dsch){

	// Pego o tempo do meu handshake
	unsigned int HSlf = handshake(mac_->nodeId());

	// Enquanto a lista de conexões ainda estiver ativa, continue
	while( !activeList_.empty() )
	{
		wimax::LinkDirection direction = activeList_.current().dir_;

		unsigned int ngh_index = activeList_.current().ndx_;

		// Se for um fluxo de entrada, eu devo conceder
		// Ver grant(i) no artigo do FEBA
		if( direction == wimax::IN ){

			unsigned int HDst = handshake(mac_->ndx2neigh(ngh_index));

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

				// Procuramos um espaço para nosso grant
				// granted = fit(i, lag_i_in, h_neigh, h_neigh + h_self
				gnt = fit(ngh_index, neigh_[ngh_index].lag_in_ ,mac_->frame() + HDst , mac_->frame() + 2*HDst + HSlf); //initial horizon, limit horizon );

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
				setSlots(busy_, gnt.frame_, frame_range, gnt.start_, gnt.range_, true);
			}

			// if ( lag_i_in > lag_max ) terminate
			if ( neigh_[ngh_index].lag_in_ > LAG_MAX )
				break;

			/*
			 * Precisamos mover o ponteiro de nossa lista ativa,
			 * há dois casos:
			 * - As requisições já foram satisfeitas
			 * - Round Robin.
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
		} // Se for um fluxo de saída, eu devo requisitar. wimax::OUT
		else{

			// Número de bytes a serem requisitados
			unsigned int pending_bytes = ( neigh_[ngh_index].backlog_ > neigh_[ngh_index].req_out_ )? neigh_[ngh_index].backlog_ - neigh_[ngh_index].req_out_:0;

			// Se não há espaço suficiente na mensagem, paramos
			if ( dsch->remaining() < WimshMshDsch::ReqIE::size() )
				break;

			// lag_i_out = min{ lag_i_out + quantum, blog_i_out}
			neigh_[ngh_index].lag_out_ += quantum(ngh_index,wimax::OUT);
			neigh_[ngh_index].lag_out_ = ( neigh_[ngh_index].lag_out_ > pending_bytes )?pending_bytes: neigh_[ngh_index].lag_out_;

			unsigned int ungranted = ( neigh_[ngh_index].req_out_ > neigh_[ngh_index].gnt_out_ )? neigh_[ngh_index].req_out_ - neigh_[ngh_index].gnt_out_ : 0;

			// if ( req_i_out - cnf_i_out > pending_max )
			// Aqui usamos gnt_i_out ao invés de cnf_i_out. As confirmações sempre são feitas posteriormente.
			if (  ungranted > PENDING_MAX  ) {


				// If (lag_i_out > lag_max) terminate
				if ( neigh_[ngh_index].lag_out_ > LAG_MAX )
					break;


				// Movemos o ponteiro do Round Robin para o próximo elemento
				activeList_.move();
				continue;
			}

			if ( pending_bytes > 0) {
				WimshMshDsch::ReqIE reqie;

				reqie.nodeId_ = mac_->ndx2neigh(ngh_index);

				// Adicionamos a requisição
				unsigned int nslots = mac_->bytes2slots(ngh_index,neigh_[ngh_index].lag_out_,false);
				WimshMshDsch::slots2level(mac_->phyMib()->slotPerFrame(), nslots ,reqie.level_,reqie.persistence_ );

				dsch->add(reqie);

				// Atualizamos o número de bytes requisitados
				// req_i_out = req_i_out + needed
				neigh_[ngh_index].req_out_ += neigh_[ngh_index].lag_out_;

				// lag_i_out = 0
				neigh_[ngh_index].lag_out_ = 0;

				// O backlog só será atualizado após a mensagem ter sido enviada de fato.
			}


			if ( pending_bytes <= 0  )
			{
				activeList_.erase();
			}
			else
			{
				activeList_.move();
			}


		}

	}
}

/*
 * Encaixa um grant em um espaço livre no horizonte de tempo definido.
 */
WimshMshDsch::GntIE  WimshBwManagerFeba::fit( unsigned int ngh_index, unsigned int bytes, unsigned int min_frame, unsigned int max_frame) {


	// Grant a ser retornado por esta função
	WimshMshDsch::GntIE gnt;

	gnt.nodeId_ = mac_->ndx2neigh(ngh_index);


	// Número de slots por frame
	unsigned int slots_frame = mac_->phyMib()->slotPerFrame();


	// Para cada frame no horizonte
	for(unsigned int f = min_frame; f<=max_frame ; f++)
	{
		// Colocamos o frame em valores reais
		unsigned int actual_frame = f % HORIZON;

		// Canal a ser usado
		unsigned int channel;
		channel = fitGnt.uniform((int) mac_->nchannels());

		// Para cada canal
		for(unsigned int c = 0; c < mac_->nchannels(); c++) {

				// Marcamos os slots que não podemos conceder.
				std::bitset<MAX_SLOTS> map = unconfirmedSlots_[actual_frame] |
					busy_[actual_frame] | self_rx_unavl_[channel][actual_frame] |
					neigh_tx_unavl_[ngh_index][channel][actual_frame];

				for( unsigned int slot; slot < slots_frame ; slot++)
				{
					gnt.frame_ = f;
					gnt.start_ = slot;
					gnt.persistence_ = WimshMshDsch::FRAME1;
					gnt.fromRequester_ = false;
					gnt.channel_ = channel;

				}
				// Pulamos para o próximo canal.
				channel = ( channel + 1 )%mac_->nchannels();
		}
	}

	// Se chegamos até aqui não foi possível alocar banda.
	gnt.range_ = 0;
	return gnt;
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

	const unsigned int ngh_index = mac_->ndx2neigh(nexthop);

	// Atualizamos o backlog aqui ( ver requestAndgrant) .
	neigh_[ngh_index].backlog_ -= bytes;
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

