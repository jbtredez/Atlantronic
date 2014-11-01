#include "table3d.h"

bool Table3d::init()
{
	bool res = table.init("media/table2015.obj");
	res &= dispenser.init("media/distributeur.obj");
	res &= clapYellow.init("media/clap_jaune.obj");
	res &= clapGreen.init("media/clap_vert.obj");
	res &= feet.init("media/pied.obj");
	res &= glass.init("media/verre.obj");

	feetPosition[0] = aiVector3D(-1410, -850, 0);
	feetPosition[1] = aiVector3D(-1410, -750, 0);
	feetPosition[2] = aiVector3D(-1410,  800, 0);
	feetPosition[3] = aiVector3D(- 650,  800, 0);
	feetPosition[4] = aiVector3D(- 650,  900, 0);
	feetPosition[5] = aiVector3D(- 630, -355, 0);
	feetPosition[6] = aiVector3D(- 400, -750, 0);
	feetPosition[7] = aiVector3D(- 200, -400, 0);

	feetPosition[8]  = aiVector3D(1410, -850, 0);
	feetPosition[9]  = aiVector3D(1410, -750, 0);
	feetPosition[10] = aiVector3D(1410,  800, 0);
	feetPosition[11] = aiVector3D( 650,  800, 0);
	feetPosition[12] = aiVector3D( 650,  900, 0);
	feetPosition[13] = aiVector3D( 630, -355, 0);
	feetPosition[14] = aiVector3D( 400, -750, 0);
	feetPosition[15] = aiVector3D( 200, -400, 0);

	glassPosition[0] = aiVector3D(-1250, -750, 0);
	glassPosition[1] = aiVector3D(- 590,  170, 0);
	glassPosition[2] = aiVector3D(    0, -650, 0);
	glassPosition[3] = aiVector3D(  590,  170, 0);
	glassPosition[4] = aiVector3D( 1250, -750, 0);

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

	drawFeets();
	drawGlass();
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

void Table3d::drawFeets()
{
	for(unsigned int i = 0; i < sizeof(feetPosition) / sizeof(feetPosition[0]); i++)
	{
		glPushMatrix();
		glTranslatef( -feet.sceneCenter.x + feetPosition[i].x, -feet.sceneCenter.y + feetPosition[i].y, -feet.sceneMin.z + feetPosition[i].z );
		feet.draw();
		glPopMatrix();
	}
}

void Table3d::drawGlass()
{
	for(unsigned int i = 0; i < sizeof(glassPosition) / sizeof(glassPosition[0]); i++)
	{
		glPushMatrix();
		glTranslatef( -glass.sceneCenter.x + glassPosition[i].x, -glass.sceneCenter.y + glassPosition[i].y, -glass.sceneMin.z + glassPosition[i].z );
		glass.draw();
		glPopMatrix();
	}
}
