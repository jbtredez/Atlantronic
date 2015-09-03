#include "table_scene.h"


static float robot2dVectrices[] =
{
	PARAM_NP_X, PARAM_RIGHT_CORNER_Y,
	PARAM_NP_X, PARAM_LEFT_CORNER_Y,
	PARAM_LEFT_CORNER_X, PARAM_LEFT_CORNER_Y,
	PARAM_RIGHT_CORNER_X, PARAM_RIGHT_CORNER_Y,
	PARAM_NP_X, PARAM_RIGHT_CORNER_Y
};

TableScene::TableScene()
{
	m_opponentRobotPos = VectPlan(-2000, 1000, 0);
	m_viewMatrix.rotateX(M_PI/6);
	m_viewMatrix.translate(0, 0, 3000);
}

bool TableScene::init(GlFont* font, RobotInterface* robot, MainShader* shader)
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

	m_glfont = font;
	m_robotItf = robot;
	m_shader = shader;

	bool res = m_table3d.init(glSelectFeetName, glSelectGlassName, shader);
	res &= m_robot3d.init(shader);
	res &= m_opponentRobot3d.init("media/opponentRobot.obj", shader);
	res &= m_robot2d.init(robot2dVectrices, 2, sizeof(robot2dVectrices)/sizeof(robot2dVectrices[0]), shader);

	float plus_pt[2*CONTROL_USB_DATA_MAX];
	memset(plus_pt, 0, sizeof(plus_pt));
	res &= m_graphPointObject.init(plus_pt, 2, CONTROL_USB_DATA_MAX, shader, true);

	for(int i = 0; i < TABLE_OBJ_SIZE; i++)
	{
		res &= m_table2d[i].init((float*)&table_obj[i].pt[0], 2, table_obj[i].size, shader);
	}

	return res;
}

VectPlan TableScene::projectMouseOnTable(int x, int y)
{
	VectPlan posTable;
	GLint viewportInt[4];
	glGetIntegerv( GL_VIEWPORT, viewportInt );

	glm::vec4 viewport = glm::vec4((float)viewportInt[0], (float)viewportInt[1], (float)viewportInt[2], (float)viewportInt[3]);
	glm::vec3 wincoord = glm::vec3(x, (float)viewportInt[3] - y - 1, 0);
	glm::vec3 objcoord = glm::unProject(wincoord, m_tableModelview, m_tableProjection, viewport);

	// calcul de l'intersection entre la droite AB (A = position camera, B = (posX, posY, posZ) avec le plan z=0
	MatrixHomogeneous m = m_viewMatrix;
	float ax = m.val[3];
	float ay = m.val[7];
	float az = m.val[11];
	float t = -az / (objcoord.z - az);
	posTable.x = t * (objcoord.x - ax) + ax;
	posTable.y = t * (objcoord.y - ay) + ay;
	posTable.theta = 0;

	return posTable;
}

void TableScene::rotateView(float dx, float dy)
{
	(void) dx;
	m_viewMatrix.rotate(dy, m_viewMatrix.val[0], m_viewMatrix.val[4], m_viewMatrix.val[8]);
	//viewMatrix.rotate(-dx, viewMatrix.val[1], viewMatrix.val[5], viewMatrix.val[9]);
}

void TableScene::translateView(float dx, float dy, float dz)
{
	m_viewMatrix.translate(dx, dy , dz);
}

void TableScene::mouseSelect(int x, int y, Graphique* graph)
{
	unsigned int id = 0;

	m_mouseX = x;
	m_mouseY = y;
	m_table3d.unselectAll();
	m_opponentRobot3d.selected = false;

	glReadPixels(x, graph->screen_height - y - 1, 1, 1, GL_STENCIL_INDEX, GL_UNSIGNED_INT, &id);
	//printf("mouse index %d\n", index);

	if( id >= GL_NAME_FEET_0 && id <= GL_NAME_FEET_15)
	{
		m_table3d.selectFeet(id - GL_NAME_FEET_0);
	}
	else if( id >= GL_NAME_GLASS_0 && id <= GL_NAME_GLASS_4)
	{
		m_table3d.selectGlass(id - GL_NAME_GLASS_0);
	}
	else if( id == GL_NAME_OPPONENT )
	{
		m_opponentRobot3d.selected = true;
	}
}

void TableScene::mouseMoveSelection(int x, int y)
{
	VectPlan p1 = projectMouseOnTable(m_mouseX, m_mouseY);
	VectPlan p2 = projectMouseOnTable(x, y);
	VectPlan delta = p2 - p1;

	if( m_opponentRobot3d.selected )
	{
		m_opponentRobotPos = m_opponentRobotPos + delta;
	}
	m_table3d.moveSelected(delta.x, delta.y);
	m_mouseX = x;
	m_mouseY = y;
}

void TableScene::drawOpponentRobot(Graphique* graph)
{
	if( ! graph->courbes_activated[SUBGRAPH_TABLE_OPPONENT_ROBOT] )
	{
		return;
	}

	glStencilFunc(GL_ALWAYS, GL_NAME_OPPONENT, ~0);
	glm::mat4 oldModelView = m_shader->getModelView();
	glm::mat4 modelView = glm::translate(oldModelView, glm::vec3(m_opponentRobotPos.x, m_opponentRobotPos.y, 0));
	modelView = glm::rotate(modelView, m_opponentRobotPos.theta, glm::vec3(0, 0, 1));
	m_shader->setModelView(modelView);

	modelView = glm::rotate(modelView, (float)M_PI/2, glm::vec3(0, 0, 1));
	m_shader->setModelView(modelView);
	m_opponentRobot3d.draw();

	m_shader->setModelView(oldModelView);
	glStencilFunc(GL_ALWAYS, 0, ~0);
}

void TableScene::drawRobot(Graphique* graph)
{
	int max = m_robotItf->control_usb_data_count % CONTROL_USB_DATA_MAX;
	if( graph->courbes_activated[SUBGRAPH_TABLE_ROBOT] && max > 0)
	{
		// robot
		VectPlan pos_robot = m_robotItf->control_usb_data[max-1].pos;
		glStencilFunc(GL_ALWAYS, GL_NAME_ROBOT, ~0);
		glm::mat4 oldModelView = m_shader->getModelView();
		glm::mat4 modelView = glm::translate(oldModelView, glm::vec3(pos_robot.x, pos_robot.y, 0));
		modelView = glm::rotate(modelView, pos_robot.theta, glm::vec3(0, 0, 1));
		m_shader->setModelView(modelView);

		m_shader->setColor(0, 0, 0);
		m_robot2d.render(GL_LINE_STRIP);

		m_robot3d.rightWingTheta = m_robotItf->ax12[AX12_RIGHT_WING].pos;
		m_robot3d.leftWingTheta = m_robotItf->ax12[AX12_LEFT_WING].pos;
		m_robot3d.lowFingerTheta = m_robotItf->ax12[AX12_LOW_FINGER].pos;
		m_robot3d.highFingerTheta = m_robotItf->ax12[AX12_HIGH_FINGER].pos;
		m_robot3d.rightCarpetTheta = m_robotItf->ax12[AX12_RIGHT_CARPET].pos;
		m_robot3d.leftCarpetTheta = m_robotItf->ax12[AX12_LEFT_CARPET].pos;
		m_robot3d.elevatorHeight = m_robotItf->last_control_usb_data.elevatorHeight;
		m_robot3d.draw();

		m_shader->setModelView(oldModelView);
		glStencilFunc(GL_ALWAYS, 0, ~0);
	}
}

void TableScene::printInfos(Graphique* graph)
{
	m_glfont->m_textShader.setProjection(m_tableProjection * m_tableModelview);

	if( graph->courbes_activated[SUBGRAPH_TABLE_GRAPH_LINK] )
	{
		m_glfont->m_textShader.setColor(graph->color[3*SUBGRAPH_TABLE_GRAPH_LINK], graph->color[3*SUBGRAPH_TABLE_GRAPH_LINK+1], graph->color[3*SUBGRAPH_TABLE_GRAPH_LINK+2], 1);
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
				m_glfont->glPrintf_xcenter_ycenter(0.5f * (x1 + x2), 0.5f * (y1 + y2), 2.5, 2.5, "%d", graph_link[i].dist);
			}
		}
	}


	if( graph->courbes_activated[SUBGRAPH_TABLE_GRAPH] )
	{
		m_glfont->m_textShader.setColor(graph->color[3*SUBGRAPH_TABLE_GRAPH],graph->color[3*SUBGRAPH_TABLE_GRAPH+1],graph->color[3*SUBGRAPH_TABLE_GRAPH+2],1);
		for(int i=0; i < GRAPH_NUM_NODE; i++)
		{
			m_glfont->glPrintf_xcenter_yhigh2(graph_node[i].pos.x, graph_node[i].pos.y, 2.5, 2.5, "%d", i);
		}
	}

	float rx = graph->ratio_x;
	float ry = graph->ratio_y;
	glm::mat4 projection = glm::ortho(graph->roi_xmin, graph->roi_xmax, graph->roi_ymin, graph->roi_ymax, 0.0f, 1.0f);
	m_glfont->m_textShader.setColor(0,0,0,1);
	m_glfont->m_textShader.setProjection(projection);

	int lineHeight = -1.5*m_glfont->digitHeight * ry;
	float x = graph->roi_xmax - 45*m_glfont->width*rx;
	float y = graph->roi_ymax + lineHeight;
	m_glfont->glPrintf(x, y, rx, ry, "time  %13.6f", m_robotItf->current_time);
	y += lineHeight;
	struct control_usb_data* data = &m_robotItf->last_control_usb_data;
	double match_time = 0;
	if( m_robotItf->start_time )
	{
		match_time = m_robotItf->current_time - m_robotItf->start_time;
	}
	m_glfont->glPrintf(x, y, rx, ry, "match %13.6f", match_time);
	y += lineHeight;
	m_glfont->glPrintf(x, y, rx, ry, "power off %#9x", data->power_state);
	y += lineHeight;
	m_glfont->glPrintf(x, y, rx, ry, "pos  %6.0f %6.0f %6.2f",
			data->pos.x, data->pos.y,
			data->pos.theta * 180 / M_PI);
	y += lineHeight;
	m_glfont->glPrintf(x, y, rx, ry, "wpos %6.0f %6.0f %6.2f",
			data->wanted_pos.x, data->wanted_pos.y,
			data->wanted_pos.theta * 180 / M_PI);
	y += lineHeight;
	m_glfont->glPrintf(x, y, rx, ry, "v %5.2f %5.2f (wanted %5.2f %5.2f)",
			data->mes_motors[0].v, data->mes_motors[1].v, data->cons_motors_v[0], data->cons_motors_v[1]);
	y += lineHeight;
/*	glfont->glPrintf(x, y, rx, ry, "gyro %6.2f",
					data->pos_theta_gyro_euler * 180 / M_PI);
	y += lineHeight;*/
	m_glfont->glPrintf(x, y, rx, ry, "vBat  %6.3f", data->vBat);
	y += lineHeight;
	for(int i = 0; i < 4; i++)
	{
		m_glfont->glPrintf(x, y, rx, ry, "iPwm%i  %6.3f", i, data->iPwm[i]);
		y += lineHeight;
	}
	m_glfont->glPrintf(x, y, rx, ry, "cod  %6d %6d %6d", data->encoder[0], data->encoder[1], data->encoder[2]);
	y += lineHeight;
	m_glfont->glPrintf(x, y, rx, ry, "io %d%d %d%d %d%d %d%d %d%d %d%d ingo %d go %d",
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
	m_glfont->glPrintf(x, y, rx, ry, "pump blocked %d %d %d %d",
			(data->pumpState & 0x01),
			((data->pumpState >> 1) & 0x01),
			((data->pumpState >> 2) & 0x01),
			((data->pumpState >> 3) & 0x01));
	y += lineHeight;
	for(int i = 2; i < AX12_MAX_ID; i++)
	{
		m_glfont->glPrintf(x, y, rx, ry, "ax12 %2d pos %7.2f target %d stuck %d error %2x", i,
				m_robotItf->ax12[i].pos * 180 / M_PI,
				(m_robotItf->ax12[i].flags & DYNAMIXEL_FLAG_TARGET_REACHED)?1:0,
				(m_robotItf->ax12[i].flags & DYNAMIXEL_FLAG_STUCK) ? 1:0,
				(m_robotItf->ax12[i].error.transmit_error << 8) + m_robotItf->ax12[i].error.internal_error);
		y += lineHeight;
	}
	for(int i = 2; i < RX24_MAX_ID; i++)
	{
		m_glfont->glPrintf(x, y, rx, ry, "rx24 %2d pos %7.2f target %d stuck %d error %2x", i,
				m_robotItf->rx24[i].pos * 180 / M_PI,
				(m_robotItf->rx24[i].flags & DYNAMIXEL_FLAG_TARGET_REACHED)?1:0,
				(m_robotItf->rx24[i].flags & DYNAMIXEL_FLAG_STUCK) ? 1:0,
				(m_robotItf->rx24[i].error.transmit_error << 8) + m_robotItf->rx24[i].error.internal_error);
		y += lineHeight;
	}
	m_glfont->glPrintf(x, y, rx, ry, "elevator %5.2f", data->elevatorHeight);
	y += lineHeight;
	/*float* mat = data->arm_matrix.val;
	glfont->glPrintf(x, y, rx, ry, "arm_mat %5.2f %5.2f %5.2f %5.2f", mat[0], mat[1], mat[2], mat[3]);
	y += lineHeight;
	glfont->glPrintf(x, y, rx, ry, "arm_mat %5.2f %5.2f %5.2f %5.2f", mat[4], mat[5], mat[6], mat[7]);
	y += lineHeight;
	glfont->glPrintf(x, y, rx, ry, "arm_mat %5.2f %5.2f %5.2f %5.2f", mat[8], mat[9], mat[10], mat[11]);
	y += lineHeight;*/
}
void TableScene::draw(Graphique* graph)
{
	static Vect2 pt[CONTROL_USB_DATA_MAX];

	int res = pthread_mutex_lock(&m_robotItf->mutex);
	if(res != 0)
	{
		return;
	}

	MatrixHomogeneous m = m_viewMatrix;
	float ax = m.val[3];
	float ay = m.val[7];
	float az = m.val[11];
	float ax0 = ax - m.val[2];
	float ay0 = ay - m.val[6];
	float az0 = az - m.val[10];


	glm::mat4 model = glm::mat4(1.0f);
	glm::mat4 view = glm::lookAt(glm::vec3(ax, ay, az), glm::vec3(ax0, ay0, az0), glm::vec3(m.val[1], m.val[5], m.val[9]));
	m_tableProjection = glm::perspective(45.0f, (float)graph->screen_width/graph->screen_height, 1.0f, 10000.0f);
	m_tableModelview = view * model;
	m_shader->setProjection(m_tableProjection);
	m_shader->setModelView(m_tableModelview);

	m_table3d.showTable = graph->courbes_activated[SUBGRAPH_TABLE_TABLE3D];
	m_table3d.showElements = graph->courbes_activated[SUBGRAPH_TABLE_ELM3D];
	m_table3d.draw();

	glDisable(GL_CULL_FACE);
	if(graph->courbes_activated[SUBGRAPH_TABLE_STATIC_ELM])
	{
		m_shader->setColor(0,0,0);
		for(int i = 0; i < TABLE_OBJ_SIZE; i++)
		{
			m_table2d[i].render(GL_LINE_STRIP);
		}
	}

	if( graph->courbes_activated[SUBGRAPH_TABLE_HOKUYO1_SEG])
	{
		m_shader->setColor3f(&graph->color[3*SUBGRAPH_TABLE_HOKUYO1_SEG]);
		for(int i = 0; i < m_robotItf->detection_dynamic_object_size; i++)
		{
			if(m_robotItf->detection_dynamic_obj[i].size > 1)
			{
				m_graphPointObject.update((float*)m_robotItf->detection_dynamic_obj[i].pt, m_robotItf->detection_dynamic_obj[i].size);
				m_graphPointObject.render(GL_LINE_STRIP);
			}
		}
	}

	for(int i = 0; i < (int)m_robotItf->detection_dynamic_object_count1; i++)
	{
		m_shader->setColor(0,0,1);
		pt[i].x = m_robotItf->detection_obj1[i].x;
		pt[i].y = m_robotItf->detection_obj1[i].y;
		//printf("obj_h1 : %7.2f %7.2f\n", robotItf->detection_obj1[i].x, robotItf->detection_obj1[i].y);
	}

	m_graphPointObject.update((float*)pt, (int)m_robotItf->detection_dynamic_object_count1);
	m_graphPointObject.render(GL_POINTS);

	for(int i = 0; i < (int)m_robotItf->detection_dynamic_object_count2; i++)
	{
		m_shader->setColor(0,1,1);
		pt[i].x = m_robotItf->detection_obj2[i].x;
		pt[i].y = m_robotItf->detection_obj2[i].y;
		//printf("obj_h2 : %7.2f %7.2f\n", robotItf->detection_obj2[i].x, robotItf->detection_obj2[i].y);
	}

	m_graphPointObject.update((float*)pt, (int)m_robotItf->detection_dynamic_object_count2);
	m_graphPointObject.render(GL_POINTS);

	if( graph->courbes_activated[SUBGRAPH_TABLE_HOKUYO1] )
	{
		m_shader->setColor3f(&graph->color[3*SUBGRAPH_TABLE_HOKUYO1]);
		for(int i=HOKUYO1*HOKUYO_NUM_POINTS; i < (HOKUYO1+1)*HOKUYO_NUM_POINTS; i++)
		{
			pt[i] = m_robotItf->detection_hokuyo_pos[i];
		}
		m_graphPointObject.update((float*)pt, HOKUYO_NUM_POINTS);
		m_graphPointObject.render(GL_POINTS);
	}

	if( graph->courbes_activated[SUBGRAPH_TABLE_HOKUYO2] )
	{
		m_shader->setColor3f(&graph->color[3*SUBGRAPH_TABLE_HOKUYO2]);
		for(int i=HOKUYO2*HOKUYO_NUM_POINTS; i < (HOKUYO2+1)*HOKUYO_NUM_POINTS; i++)
		{
			pt[i] = m_robotItf->detection_hokuyo_pos[i];
		}
		m_graphPointObject.update((float*)pt, HOKUYO_NUM_POINTS);
		m_graphPointObject.render(GL_POINTS);
	}

	int max = m_robotItf->control_usb_data_count % CONTROL_USB_DATA_MAX;

	if( graph->courbes_activated[SUBGRAPH_TABLE_POS_CONS] )
	{
		unsigned int pointCount = 0;
		m_shader->setColor3f(&graph->color[3*SUBGRAPH_TABLE_POS_CONS]);

		for(int i=0; i< max; i++)
		{
			int state = m_robotItf->control_usb_data[i].motion_state;
			if( state != MOTION_ENABLED && state != MOTION_DISABLED)
			{
				pt[pointCount] = m_robotItf->control_usb_data[i].cons;
				pointCount++;
			}
		}
		m_graphPointObject.update((float*)pt, pointCount);
		m_graphPointObject.render(GL_POINTS);
	}

	if( graph->courbes_activated[SUBGRAPH_TABLE_POS_MES] )
	{
		m_shader->setColor3f(&graph->color[3*SUBGRAPH_TABLE_POS_MES]);
		for(int i=0; i < max; i++)
		{
			pt[i] = m_robotItf->control_usb_data[i].pos;
		}
		m_graphPointObject.update((float*)pt, max);
		m_graphPointObject.render(GL_POINTS);
	}

	if( graph->courbes_activated[SUBGRAPH_TABLE_GRAPH_LINK] )
	{
		glEnable(GL_LINE_STIPPLE);
		glLineStipple(1, 0xAAAA);
		m_shader->setColor3f(&graph->color[3*SUBGRAPH_TABLE_GRAPH_LINK]);
		unsigned int pointCount = 0;
		for(int i=0; i < GRAPH_NUM_LINK; i++)
		{
			int a = graph_link[i].a;
			int b = graph_link[i].b;
			// on trace les liens une seule fois
			if( a < b)
			{
				pt[pointCount] = graph_node[a].pos;
				pointCount++;
				pt[pointCount] = graph_node[b].pos;
				pointCount++;
			}
		}
		m_graphPointObject.update((float*)pt, pointCount);
		m_graphPointObject.render(GL_LINES);
		glDisable(GL_LINE_STIPPLE);
	}

	if( graph->courbes_activated[SUBGRAPH_TABLE_GRAPH] )
	{
		m_shader->setColor3f(&graph->color[3*SUBGRAPH_TABLE_GRAPH]);
		for(int i=0; i < GRAPH_NUM_NODE; i++)
		{
			pt[i] = graph_node[i].pos;
		}
		m_graphPointObject.update((float*)pt, GRAPH_NUM_NODE);
		m_graphPointObject.render(GL_POINTS);
	}

	// robot adverse
	drawOpponentRobot(graph);

	// affichage du robot
	drawRobot(graph);

	pthread_mutex_unlock(&m_robotItf->mutex);
}
