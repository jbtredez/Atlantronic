#ifndef ENVIRONNEMENT_H
#define ENVIRONNEMENT_H

#include <irrlicht/irrlicht.h>
#include <Newton.h>
#include <pthread.h>
#include "Robot.h"
#include "Model.h"
#include "Table.h"
#include "Pion.h"

class Environnement : public irr::IEventReceiver
{
public:
	Environnement();
	~Environnement();

	void loop();
	bool configure(unsigned int a, unsigned int b, unsigned int c);
	void start(const char* prog1, const char* prog2, int gdb_port1, int gdb_port2);

protected:
	void irrlichtInit();
	void newtonInit();
	void loadAll();
	static void* newtonTaskInit(void* arg);
	void newtonTask();
	bool OnEvent(const irr::SEvent& event);

	// irrlicht (boucle sur la tache principale) : basse fréquence
	irr::IrrlichtDevice* device;
	irr::video::IVideoDriver* driver;
	irr::scene::ISceneManager* smgr;
	irr::gui::IGUIEnvironment* guienv;
	irr::scene::ICameraSceneNode* camera[2];

	// newton (cadencé par une seconde tache) : haute fréquence
	NewtonWorld *newtonWorld;
	pthread_t id;

	// elements
	Pion* pions[19];// 15 pions, 2 roi, 2 reines
	Table* table;
	Robot* robot[2];

	// configuration
	int confCarte[3]; // numéro de la configuration (cf spec TODO mettre à jour les spec)
	irr::core::vector3df config[19];
};

#endif
