#ifndef DYNAMIXEL_ID_H
#define DYNAMIXEL_ID_H

//!< id 1 réservé pour les nouveaux dynamixel non configurés
//!< laisser les id ordonés (si nécessaire, reconfigurer les dynamixel)
//!< le but est de maintenir un tableau de taille DYNAMIXEL_MAX_ID avec les codes d'erreur
//!< verification de la présence des dynamixel
enum ax12_id
{
	AX12_NOT_CONFIGURED = 1,
	AX12_FINGER_RIGHT,
	AX12_FINGER_LEFT,
	AX12_CANON_STOCK_RIGHT,
	AX12_CANON_STOCK_LEFT,
	AX12_ARM_1,
	AX12_ARM_2,
	AX12_ARM_3,
	AX12_ARM_4,
	AX12_MAX_ID,
	AX12_BROADCAST = 0xfe,
};

enum rx24_id
{
	RX24_NOT_CONFIGURED = 1,
	RX24_CANON_SHOOT_RIGHT,
	RX24_CANON_SHOOT_LEFT,
	RX24_ARM_0,
//	RX24_RAB4,
//	RX24_RAB5,
//	RX24_RAB6,
	RX24_MAX_ID,
	RX24_BROADCAST = 0xfe,
};

#endif
