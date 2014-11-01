#include "table3d.h"

bool Table3d::init()
{
	bool res = table.init("media/table2015.obj");
	res &= dispenser.init("media/distributeur.obj");
	res &= clapYellow.init("media/clap_jaune.obj");
	res &= clapGreen.init("media/clap_vert.obj");
	return res;
}

void Table3d::draw()
{
	glPushMatrix();
	glTranslatef( -table.sceneCenter.x, -table.sceneCenter.y, -table.sceneMin.z-22 );
	table.draw();
	glPopMatrix();

	drawDispenser(-1200);
	drawDispenser(- 900);
	drawDispenser(  900);
	drawDispenser( 1200);

	drawClap(-1100, true);
	drawClap(- 800, false);
	drawClap(- 500, true);

	drawClap(  500, false);
	drawClap(  800, true);
	drawClap( 1100, false);
}

void Table3d::drawDispenser(float x)
{
	glPushMatrix();
	glTranslatef(x, 1022, 0);
	glRotatef(180, 0, 0, 1);
	glTranslatef( -dispenser.sceneCenter.x, -dispenser.sceneMin.y, -dispenser.sceneMin.z );
	dispenser.draw();
	glPopMatrix();
}

void Table3d::drawClap(float x, bool yellow)
{
	Object3d* clap = &clapYellow;
	if( ! yellow )
	{
		clap = &clapGreen;
	}
	glPushMatrix();
	glTranslatef(x, -1000, 78);
	if( x > 0)
	{
		glRotatef(180, 0, 0, 1);
	}
	if( x < 0)
	{
		glTranslatef( -clap->sceneMax.x, -clap->sceneMax.y, -clap->sceneMin.z );
	}
	else
	{
		glTranslatef( -clap->sceneMax.x, -clap->sceneMin.y, -clap->sceneMin.z );
	}
	clap->draw();
	glPopMatrix();
}
