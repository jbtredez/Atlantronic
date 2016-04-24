#ifndef DYNAMIXEL_ID_H
#define DYNAMIXEL_ID_H

//!< id 1 réservé pour les nouveaux dynamixel non configurés
//!< laisser les id ordonés (si nécessaire, reconfigurer les dynamixel)
//!< le but est de maintenir un tableau de taille DYNAMIXEL_MAX_ID avec les codes d'erreur
//!< verification de la présence des dynamixel
enum ax12_id
{
	AX12_NOT_CONFIGURED = 1,
	AX12_STAR_LEFT_FISH_WING,
	AX12_STAR_LEFT_FISH_REMOVER,
	AX12_STAR_RIGHT_FISH_WING,
	AX12_STAR_RIGHT_FISH_REMOVER,
	AX12_STAR_LEFT_DOOR,
	AX12_STAR_RIGHT_DOOR,
	AX12_STAR_TOWER_PLIERS,
	AX12_STAR_PARASOL,
	AX12_GATE_PARASOL,
	AX12_MAX_ID,
	AX12_BROADCAST = 0xfe,
};

enum rx24_id
{
	RX24_NOT_CONFIGURED = 1,
	RX24_MAX_ID,
	RX24_BROADCAST = 0xfe,
};

#endif
