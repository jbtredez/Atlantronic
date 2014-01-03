#ifndef DYNAMIXEL_ID_H
#define DYNAMIXEL_ID_H

//!< id 1 réservé pour les nouveaux dynamixel non configurés
//!< laisser les id ordonés (si nécessaire, reconfigurer les dynamixel)
//!< le but est de maintenir un tableau de taille DYNAMIXEL_MAX_ID avec les codes d'erreur
//!< verification de la présence des dynamixel
enum ax12_id
{
	AX12_NOT_CONFIGURED = 1,
	AX12_PINCE_RIGHT,
	AX12_PINCE_LEFT,
	AX12_ARM_1,
	AX12_ARM_2,
//	AX12_RAB_1,
//	AX12_RAB_2,
	AX12_MAX_ID,
	AX12_BROADCAST = 0xfe,
};

enum rx24_id
{
	RX24_NOT_CONFIGURED = 1,
	RX24_RAB1,
	RX24_RAB2,
//	RX24_RAB3,
//	RX24_RAB4,
//	RX24_RAB5,
//	RX24_RAB6,
	RX24_MAX_ID,
	RX24_BROADCAST = 0xfe,
};

#endif
