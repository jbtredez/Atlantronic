#ifndef AX12_ID_H
#define AX12_ID_H

//!< id 1 réservé pour les nouveaux ax12 non configurés
//!< laisser les id ordonés (si nécessaire, reconfigurer les ax12)
//!< le but est de maintenir un tableau de taille AX12_MAX_ID avec les codes d'erreur
//!< verification de la présence des ax12
enum ax12_id
{
	AX12_NOT_CONFIGURED = 1,
	AX12_PINCE_RIGHT,
	AX12_PINCE_LEFT,
	AX12_ARM_1,
	AX12_ARM_2,
	AX12_RAB_1,
	AX12_RAB_2,
	AX12_MAX_ID,
	AX12_BROADCAST = 0xfe,
};

#endif