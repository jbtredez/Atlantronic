#include "table_scene.h"

TableScene::TableScene()
{
	tableThetaZ = 0;
	opponentRobotPos = VectPlan(1300, 0, M_PI);
	viewMatrix.rotateX(M_PI/6);
	viewMatrix.translate(0, 0, 3800);
}

bool TableScene::init(GlFont* font, RobotInterface* robot)
{
	int glSelectFeetName[16] =
	{
		GL_NAME_FEET_0,
		GL_NAME_FEET_1,
		GL_NAME_FEET_2,
		GL_NAME_FEET_3,
		GL_NAME_FEET_4,
		GL_NAME_FEET_5,
		GL_NAME_FEET_6,
		GL_NAME_FEET_7,
		GL_NAME_FEET_8,
		GL_NAME_FEET_9,
		GL_NAME_FEET_10,
		GL_NAME_FEET_11,
		GL_NAME_FEET_12,
		GL_NAME_FEET_13,
		GL_NAME_FEET_14,
		GL_NAME_FEET_15,
	};

	int glSelectGlassName[5] =
	{
		GL_NAME_GLASS_0,
		GL_NAME_GLASS_1,
		GL_NAME_GLASS_2,
		GL_NAME_GLASS_3,
		GL_NAME_GLASS_4,
	};

	glfont = font;
	robotItf = robot;

	if( ! table3d.init(glSelectFeetName, glSelectGlassName) )
	{
		fprintf(stderr, "table3d.init() error - Exiting.\n");
		return false;
	}

	if( ! robot3d.init() )
	{
		fprintf(stderr, "robot3d.init() error - Exiting.\n");
		return false;
	}

	if( ! opponentRobot3d.init("media/opponentRobot.obj") )
	{
		fprintf(stderr, "opponentRobot3d.init() error - Exiting.\n");
		return false;
	}

	return true;
}

void TableScene::draw_plus(float x, float y, float rx, float ry)
{
	glBegin(GL_LINES);
	glVertex2f(x-rx, y);
	glVertex2f(x+rx, y);
	glVertex2f(x, y-ry);
	glVertex2f(x, y+ry);
	glEnd();
}

VectPlan TableScene::projectMouseOnTable(int x, int y)
{
	VectPlan posTable;
	GLint viewport[4];
	GLdouble posX, posY, posZ;
	GLfloat winX, winY;

	glGetIntegerv( GL_VIEWPORT, viewport );

	winX = (float)x;
	winY = (float)viewport[3] - (float)y;

	gluUnProject( winX, winY, 0, tableModelview, tableProjection, viewport, &posX, &posY, &posZ);

	// calcul de l'intersection entre la droite AB (A = position camera, B = (posX, posY, posZ) avec le plan z=0
	MatrixHomogeneous m = viewMatrix;
	float ax = m.val[3];
	float ay = m.val[7];
	float az = m.val[11];
	float t = -az / (posZ - az);
	posTable.x = t * (posX - ax) + ax;
	posTable.y = t * (posY - ay) + ay;
	posTable.theta = 0;

	return posTable;
}

void TableScene::rotateView(float dx, float dy)
{
	(void) dx;
	viewMatrix.rotate(dy, viewMatrix.val[0], viewMatrix.val[4], viewMatrix.val[8]);
	//viewMatrix.rotate(-dx, viewMatrix.val[1], viewMatrix.val[5], viewMatrix.val[9]);
}

void TableScene::translateView(float dx, float dy, float dz)
{
	viewMatrix.translate(dx, dy , dz);
}

void TableScene::mouseSelect(int x, int y, Graphique* graph)
{
	GLuint openglSelectBuffer[512];

	mouseX = x;
	mouseY = y;

	glSelectBuffer(sizeof(openglSelectBuffer)/sizeof(openglSelectBuffer[0]), openglSelectBuffer);
	glRenderMode(GL_SELECT);
	glInitNames();
	glPushName(GL_NAME_NONE);

	draw(GL_SELECT, graph);
	GLint hits = glRenderMode(GL_RENDER);
	processHits(hits, openglSelectBuffer);
}

void TableScene::mouseMoveSelection(int x, int y)
{
	VectPlan p1 = projectMouseOnTable(mouseX, mouseY);
	VectPlan p2 = projectMouseOnTable(x, y);
	VectPlan delta = p2 - p1;

	if( opponentRobot3d.selected )
	{
		opponentRobotPos = opponentRobotPos + delta;
	}
	table3d.moveSelected(delta.x, delta.y);
	mouseX = x;
	mouseY = y;
}

void TableScene::processHits(GLint hits, GLuint buffer[])
{
	GLuint names, *ptr;
	table3d.unselectAll();
	opponentRobot3d.selected = false;
	ptr = (GLuint *) buffer;
	for(int i = 0; i < hits; i++)
	{
		names = *ptr;
		ptr+= 3;
		if( names )
		{
			ptr += names - 1;
			int name_id = *ptr;
			if( name_id >= GL_NAME_FEET_0 && name_id <= GL_NAME_FEET_15)
			{
				table3d.selectFeet(name_id - GL_NAME_FEET_0);
			}
			else if( name_id >= GL_NAME_GLASS_0 && name_id <= GL_NAME_GLASS_4)
			{
				table3d.selectGlass(name_id - GL_NAME_GLASS_0);
			}
			else if( name_id == GL_NAME_OPPONENT )
			{
				opponentRobot3d.selected = true;
			}
#if 0
			if( name_id != GL_NAME_NONE )
			{
				printf("name %d\n", name_id);
			}
#endif
			ptr++;
		}
	}
}

void TableScene::drawOpponentRobot()
{
	glPushName(GL_NAME_OPPONENT);
	glPushMatrix();

	glColor3f(0, 0, 0);
	glTranslatef(opponentRobotPos.x, opponentRobotPos.y, 0);
	glRotated(opponentRobotPos.theta * 360.0f / (2*M_PI), 0, 0, 1);
#if 0
	glBegin(GL_LINE_STRIP);
	for(i = 0; i < oponent_robot.size; i++)
	{
		glVertex2f(oponent_robot.pt[i].x, oponent_robot.pt[i].y);
	}
	glEnd();
#endif
	glRotatef(90, 0, 0, 1);
	glDisable(GL_COLOR_MATERIAL);
	opponentRobot3d.draw();
	glEnable(GL_COLOR_MATERIAL);
	glDisable(GL_CULL_FACE);

	glPopMatrix();
	glPopName();
}

void TableScene::drawRobot(Graphique* graph)
{
	int max = robotItf->control_usb_data_count % CONTROL_USB_DATA_MAX;
	if( graph->courbes_activated[SUBGRAPH_TABLE_POS_ROBOT] && max > 0)
	{
		// robot
		VectPlan pos_robot = robotItf->control_usb_data[max-1].pos;
		glPushName(GL_NAME_ROBOT);
		glPushMatrix();
		glTranslatef(pos_robot.x, pos_robot.y, 0);
		glRotatef(pos_robot.theta * 180 / M_PI, 0, 0, 1);

		glColor3f(0, 0, 0);
		glBegin(GL_LINES);
		glVertex2f(0, 0);
		glVertex2f(50, 0);
		glVertex2f(0, 0);
		glVertex2f(0, 50);
		glEnd();

		glBegin(GL_LINE_STRIP);
		glVertex2f(PARAM_NP_X, PARAM_RIGHT_CORNER_Y);
		glVertex2f(PARAM_NP_X, PARAM_LEFT_CORNER_Y);
		glVertex2f(PARAM_LEFT_CORNER_X, PARAM_LEFT_CORNER_Y);
		glVertex2f(PARAM_RIGHT_CORNER_X, PARAM_RIGHT_CORNER_Y);
		glVertex2f(PARAM_NP_X, PARAM_RIGHT_CORNER_Y);
		glEnd();

		glPushMatrix();
		glDisable(GL_COLOR_MATERIAL);
		robot3d.rightWingTheta = robotItf->ax12[AX12_RIGHT_WING].pos;
		robot3d.leftWingTheta = robotItf->ax12[AX12_LEFT_WING].pos;
		robot3d.lowFingerTheta = robotItf->ax12[AX12_LOW_FINGER].pos;
		robot3d.highFingerTheta = robotItf->ax12[AX12_HIGH_FINGER].pos;
		robot3d.elevatorHeight = robotItf->last_control_usb_data.elevatorHeight;
		robot3d.draw();
		glEnable(GL_COLOR_MATERIAL);
		glDisable(GL_CULL_FACE);
		glPopMatrix();

		glPopMatrix();
		glPopName();
	}
}

void TableScene::printInfos(Graphique* graph)
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(graph->roi_xmin, graph->roi_xmax, graph->roi_ymin, graph->roi_ymax, 0, 1);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glColor3f(0,0,0);
	glPushMatrix();
	int lineHeight = -1.5*glfont->digitHeight * graph->ratio_y;
	float x = graph->roi_xmax - 45*glfont->width * graph->ratio_x;
	float y = graph->roi_ymax + lineHeight;
	glfont->glPrintf(x, y, "time  %13.6f", robotItf->current_time);
	y += lineHeight;
	struct control_usb_data* data = &robotItf->last_control_usb_data;
	double match_time = 0;
	if( robotItf->start_time )
	{
		match_time = robotItf->current_time - robotItf->start_time;
	}
	glfont->glPrintf(x, y, "match %13.6f", match_time);
	y += lineHeight;
	glfont->glPrintf(x, y, "power off %#9x", data->power_state);
	y += lineHeight;
	glfont->glPrintf(x, y, "pos  %6.0f %6.0f %6.2f",
			data->pos.x, data->pos.y,
			data->pos.theta * 180 / M_PI);
	y += lineHeight;
	glfont->glPrintf(x, y, "wpos %6.0f %6.0f %6.2f",
			data->wanted_pos.x, data->wanted_pos.y,
			data->wanted_pos.theta * 180 / M_PI);
	y += lineHeight;
	glfont->glPrintf(x, y, "v %5.2f %5.2f (wanted %5.2f %5.2f)",
			data->mes_motors[0].v, data->mes_motors[1].v, data->cons_motors_v[0], data->cons_motors_v[1]);
	y += lineHeight;
	glfont->glPrintf(x, y, "gyro %6.2f",
					data->pos_theta_gyro_euler * 180 / M_PI);
	y += lineHeight;
	glfont->glPrintf(x, y, "vBat  %6.3f", data->vBat);
	y += lineHeight;
	for(int i = 0; i < 4; i++)
	{
		glfont->glPrintf(x, y, "iPwm%i  %6.3f", i, data->iPwm[i]);
		y += lineHeight;
	}
	glfont->glPrintf(x, y, "cod  %6d %6d %6d", data->encoder[0], data->encoder[1], data->encoder[2]);
	y += lineHeight;
	glfont->glPrintf(x, y, "io %d%d %d%d %d%d %d%d %d%d %d%d ingo %d go %d",
			data->gpio & 0x01,
			(data->gpio >> 1) & 0x01,
			(data->gpio >> 2) & 0x01,
			(data->gpio >> 3) & 0x01,
			(data->gpio >> 4) & 0x01,
			(data->gpio >> 5) & 0x01,
			(data->gpio >> 6) & 0x01,
			(data->gpio >> 7) & 0x01,
			(data->gpio >> 8) & 0x01,
			(data->gpio >> 9) & 0x01,
			(data->gpio >> 10) & 0x01,
			(data->gpio >> 11) & 0x01,
			(data->gpio >> 12) & 0x01,
			(data->gpio >> 13) & 0x01);
	y += lineHeight;
	glfont->glPrintf(x, y, "pump blocked %d %d %d %d",
			(data->pumpState & 0x01),
			((data->pumpState >> 1) & 0x01),
			((data->pumpState >> 2) & 0x01),
			((data->pumpState >> 3) & 0x01));
	y += lineHeight;
	for(int i = 2; i < AX12_MAX_ID; i++)
	{
		glfont->glPrintf(x, y, "ax12 %2d pos %7.2f target %d stuck %d error %2x", i,
				robotItf->ax12[i].pos * 180 / M_PI,
				(robotItf->ax12[i].flags & DYNAMIXEL_FLAG_TARGET_REACHED)?1:0,
				(robotItf->ax12[i].flags & DYNAMIXEL_FLAG_STUCK) ? 1:0,
				(robotItf->ax12[i].error.transmit_error << 8) + robotItf->ax12[i].error.internal_error);
		y += lineHeight;
	}
	for(int i = 2; i < RX24_MAX_ID; i++)
	{
		glfont->glPrintf(x, y, "rx24 %2d pos %7.2f target %d stuck %d error %2x", i,
				robotItf->rx24[i].pos * 180 / M_PI,
				(robotItf->rx24[i].flags & DYNAMIXEL_FLAG_TARGET_REACHED)?1:0,
				(robotItf->rx24[i].flags & DYNAMIXEL_FLAG_STUCK) ? 1:0,
				(robotItf->rx24[i].error.transmit_error << 8) + robotItf->rx24[i].error.internal_error);
		y += lineHeight;
	}
	glfont->glPrintf(x, y, "elevator %5.2f", data->elevatorHeight);
	y += lineHeight;
	/*float* mat = data->arm_matrix.val;
	glfont->glPrintf(x, y, "arm_mat %5.2f %5.2f %5.2f %5.2f", mat[0], mat[1], mat[2], mat[3]);
	y += lineHeight;
	glfont->glPrintf(x, y, "arm_mat %5.2f %5.2f %5.2f %5.2f", mat[4], mat[5], mat[6], mat[7]);
	y += lineHeight;
	glfont->glPrintf(x, y, "arm_mat %5.2f %5.2f %5.2f %5.2f", mat[8], mat[9], mat[10], mat[11]);
	y += lineHeight;*/
	glPopMatrix();
}
void TableScene::draw(GLenum mode, Graphique* graph)
{
	int res = pthread_mutex_lock(&robotItf->mutex);
	if(res != 0)
	{
		return;
	}

	// efface le frame buffer
	glClearColor(1,1,1,1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	float ratio_x = graph->ratio_x;
	float ratio_y = graph->ratio_y;
	float plus_dx = 2 * ratio_x;
	float plus_dy = 2 * ratio_y;

	printInfos(graph);
	MatrixHomogeneous m = viewMatrix;
	float ax = m.val[3];
	float ay = m.val[7];
	float az = m.val[11];
	float ax0 = ax - m.val[2];
	float ay0 = ay - m.val[6];
	float az0 = az - m.val[10];

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	if( mode == GL_SELECT)
	{
		GLint viewport[4];;
		glGetIntegerv(GL_VIEWPORT, viewport);
		gluPickMatrix((GLdouble) mouseX, (GLdouble) (viewport[3] - mouseY), 5.0, 5.0, viewport);
	}

	gluPerspective(45, (float)graph->screen_width/(float)graph->screen_height, 1, 10000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gluLookAt(ax, ay, az, ax0, ay0, az0, m.val[1], m.val[5], m.val[9]);

	glGetDoublev( GL_MODELVIEW_MATRIX, tableModelview );
	glGetDoublev( GL_PROJECTION_MATRIX, tableProjection );

	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);

	//GLfloat global_ambient[] = { 0.5, 0.5, 0.5, 1 };
	GLfloat ambientLight[] = { 0.3, 0.3, 0.3, 1 };
	GLfloat diffuseLight[] = { 0.4, 0.4, 0.4, 1 };
	GLfloat specularLight[] = { 0.4, 0.4, 0.4, 1 };
	GLfloat light_position0[] = { 1500, 1000, 1000, 1 };
	GLfloat light_direction0[] = { -1500, -1000, -1000, 0};
	GLfloat light_position1[] = { -1500, 1000, 1000, 1 };
	GLfloat light_direction1[] = { 1500, -1000, -1000, 0};

	//glLightModelfv(GL_LIGHT_MODEL_AMBIENT, global_ambient);
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position0);
	glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, light_direction0);
	//glLightf(GL_LIGHT0, GL_SPOT_CUTOFF, 45.0);
	//glLightf(GL_LIGHT0, GL_SPOT_EXPONENT, 2.0);
	//glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, 0.00001f);
	//glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 1);

	glLightfv(GL_LIGHT1, GL_AMBIENT, ambientLight);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuseLight);
	glLightfv(GL_LIGHT1, GL_SPECULAR, specularLight);
	glLightfv(GL_LIGHT1, GL_POSITION, light_position1);
	glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, light_direction1);
	//glLightf(GL_LIGHT1, GL_SPOT_CUTOFF, 45.0);
	//glLightf(GL_LIGHT1, GL_SPOT_EXPONENT, 2.0);
	//glLightf(GL_LIGHT1, GL_QUADRATIC_ATTENUATION, 0.00001f);
	//glLightf(GL_LIGHT1, GL_CONSTANT_ATTENUATION, 1);

	table3d.showTable = graph->courbes_activated[SUBGRAPH_TABLE_TABLE3D];
	table3d.showElements = graph->courbes_activated[SUBGRAPH_TABLE_ELM3D];
	glDisable(GL_COLOR_MATERIAL);
	table3d.draw(mode);
	glEnable(GL_COLOR_MATERIAL);
	glDisable(GL_CULL_FACE);

	if( mode != GL_SELECT )
	{
		if(graph->courbes_activated[SUBGRAPH_TABLE_STATIC_ELM])
		{
			glColor3f(0, 0, 0);
			// éléments statiques de la table partagés avec le code du robot (obstacles statiques)
			for(int i = 0; i < TABLE_OBJ_SIZE; i++)
			{
				glBegin(GL_LINE_STRIP);
				for(int j = 0; j < table_obj[i].size; j++)
				{
					glVertex2f(table_obj[i].pt[j].x, table_obj[i].pt[j].y);
				}
				glEnd();
			}
		}

		/*if( graph->courbes_activated[SUBGRAPH_TABLE_HOKUYO1_SEG])
		{
			glColor3fv(&graph->color[3*SUBGRAPH_TABLE_HOKUYO1_SEG]);
			for(i = 0; i < robotItf->detection_dynamic_object_size; i++)
			{
				if(robotItf->detection_dynamic_obj[i].size > 1)
				{
					glBegin(GL_LINE_STRIP);
					for(j = 0; j < robotItf->detection_dynamic_obj[i].size; j++)
					{
						Vect2 pt = robotItf->detection_dynamic_obj[i].pt[j];
						glVertex2f(pt.x, pt.y);
					}
					glEnd();
				}
			}
		}*/

		for(int i = 0; i < (int)robotItf->detection_dynamic_object_count1; i++)
		{
			glColor3f(0,0,1);
			draw_plus(robotItf->detection_obj1[i].x, robotItf->detection_obj1[i].y, 50, 50);
			//printf("obj_h1 : %7.2f %7.2f\n", robotItf->detection_obj1[i].x, robotItf->detection_obj1[i].y);
		}

		for(int i = 0; i < (int)robotItf->detection_dynamic_object_count2; i++)
		{
			glColor3f(0,1,1);
			draw_plus(robotItf->detection_obj2[i].x, robotItf->detection_obj2[i].y, 50, 50);
			//printf("obj_h2 : %7.2f %7.2f\n", robotItf->detection_obj2[i].x, robotItf->detection_obj2[i].y);
		}

		if( graph->courbes_activated[SUBGRAPH_TABLE_HOKUYO1] )
		{
			glColor3fv(&graph->color[3*SUBGRAPH_TABLE_HOKUYO1]);
			for(int i=HOKUYO1*HOKUYO_NUM_POINTS; i < (HOKUYO1+1)*HOKUYO_NUM_POINTS; i++)
			{
				draw_plus(robotItf->detection_hokuyo_pos[i].x, robotItf->detection_hokuyo_pos[i].y, plus_dx, plus_dy);
			}
		}

		if( graph->courbes_activated[SUBGRAPH_TABLE_HOKUYO2] )
		{
			glColor3fv(&graph->color[3*SUBGRAPH_TABLE_HOKUYO2]);
			for(int i=HOKUYO2*HOKUYO_NUM_POINTS; i < (HOKUYO2+1)*HOKUYO_NUM_POINTS; i++)
			{
				draw_plus(robotItf->detection_hokuyo_pos[i].x, robotItf->detection_hokuyo_pos[i].y, plus_dx, plus_dy);
			}
		}

		int max = robotItf->control_usb_data_count % CONTROL_USB_DATA_MAX;

		if( graph->courbes_activated[SUBGRAPH_TABLE_POS_CONS] )
		{
			glColor3fv(&graph->color[3*SUBGRAPH_TABLE_POS_CONS]);
			for(int i=0; i< max; i++)
			{
				if(robotItf->control_usb_data[i].motion_state != MOTION_ENABLED && robotItf->control_usb_data[i].motion_state != MOTION_DISABLED)
				{
					draw_plus(robotItf->control_usb_data[i].cons.x, robotItf->control_usb_data[i].cons.y, plus_dx, plus_dy);
				}
			}
		}

		if( graph->courbes_activated[SUBGRAPH_TABLE_POS_MES] )
		{
			glColor3fv(&graph->color[3*SUBGRAPH_TABLE_POS_MES]);
			for(int i=0; i < max; i++)
			{
				draw_plus(robotItf->control_usb_data[i].pos.x, robotItf->control_usb_data[i].pos.y, plus_dx, plus_dy);
			}
		}

		if( graph->courbes_activated[SUBGRAPH_TABLE_GRAPH_LINK] )
		{
			glEnable(GL_LINE_STIPPLE);
			glLineStipple(1, 0xAAAA);
			glColor3fv(&graph->color[3*SUBGRAPH_TABLE_GRAPH_LINK]);
			for(int i=0; i < GRAPH_NUM_LINK; i++)
			{
				int a = graph_link[i].a;
				int b = graph_link[i].b;
				// on trace les liens une seule fois
				if( a < b)
				{
					float x1 = graph_node[a].pos.x;
					float y1 = graph_node[a].pos.y;
					float x2 = graph_node[b].pos.x;
					float y2 = graph_node[b].pos.y;
					glBegin(GL_LINES);
					glVertex2f(x1, y1);
					glVertex2f(x2, y2);
					glEnd();
					glfont->glPrintf_xcenter_ycenter(0.5f * (x1 + x2), 0.5f * (y1 + y2), ratio_x, ratio_y, "%d", graph_link[i].dist);
				}
			}
			glDisable(GL_LINE_STIPPLE);
		}

		if( graph->courbes_activated[SUBGRAPH_TABLE_GRAPH] )
		{
			glColor3fv(&graph->color[3*SUBGRAPH_TABLE_GRAPH]);
			for(int i=0; i < GRAPH_NUM_NODE; i++)
			{
				draw_plus(graph_node[i].pos.x, graph_node[i].pos.y, plus_dx, plus_dy);
				glfont->glPrintf_xcenter_yhigh2(graph_node[i].pos.x, graph_node[i].pos.y, ratio_x, ratio_y, "%d", i);
			}
		}
	}

	// robot adverse
	drawOpponentRobot();

	if( mode != GL_SELECT)
	{
		// affichage du robot
		drawRobot(graph);
	}

	pthread_mutex_unlock(&robotItf->mutex);
}
