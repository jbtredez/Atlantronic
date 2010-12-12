#include <irrlicht/irrlicht.h>
#include <irrlicht/driverChoice.h>
#include "Environnement.h"

#if 0
using namespace irr;
using namespace gui;

/*
Some global variables used later on
*/
IrrlichtDevice *Device = 0;
core::stringc StartUpModelFile;
core::stringw MessageText;
core::stringw Caption;
scene::ISceneNode* Model = 0;
scene::ISceneNode* SkyBox = 0;
bool Octree=false;
bool UseLight=false;
scene::IAnimatedMeshSceneNode* animModel2 = 0; //robot
scene::IAnimatedMeshSceneNode* animModel3 = 0;
scene::ICameraSceneNode* Camera[2] = {0, 0};

// Values used to identify individual GUI elements
enum
{
	GUI_ID_DIALOG_ROOT_WINDOW  = 0x10000,

	GUI_ID_X_SCALE,
	GUI_ID_Y_SCALE,
	GUI_ID_Z_SCALE,

	GUI_ID_OPEN_MODEL,
	GUI_ID_SET_MODEL_ARCHIVE,
	GUI_ID_LOAD_AS_OCTREE,

	GUI_ID_SKY_BOX_VISIBLE,
	GUI_ID_TOGGLE_DEBUG_INFO,

	GUI_ID_DEBUG_OFF,
	GUI_ID_DEBUG_BOUNDING_BOX,
	GUI_ID_DEBUG_NORMALS,
	GUI_ID_DEBUG_SKELETON,
	GUI_ID_DEBUG_WIRE_OVERLAY,
	GUI_ID_DEBUG_HALF_TRANSPARENT,
	GUI_ID_DEBUG_BUFFERS_BOUNDING_BOXES,
	GUI_ID_DEBUG_ALL,

	GUI_ID_MODEL_MATERIAL_SOLID,
	GUI_ID_MODEL_MATERIAL_TRANSPARENT,
	GUI_ID_MODEL_MATERIAL_REFLECTION,

	GUI_ID_CAMERA_MAYA,
	GUI_ID_CAMERA_FIRST_PERSON,

	GUI_ID_POSITION_TEXT,

	GUI_ID_ABOUT,
	GUI_ID_QUIT,

	GUI_ID_TEXTUREFILTER,
	GUI_ID_SKIN_TRANSPARENCY,
	GUI_ID_SKIN_ANIMATION_FPS,

	GUI_ID_BUTTON_SET_SCALE,
	GUI_ID_BUTTON_SCALE_MUL10,
	GUI_ID_BUTTON_SCALE_DIV10,
	GUI_ID_BUTTON_OPEN_MODEL,
	GUI_ID_BUTTON_SHOW_ABOUT,
	GUI_ID_BUTTON_SHOW_TOOLBOX,
	GUI_ID_BUTTON_SELECT_ARCHIVE,

	GUI_ID_ANIMATION_INFO,

	// And some magic numbers
	MAX_FRAMERATE = 80,
	DEFAULT_FRAMERATE = 30
};


/*
Toggle between various cameras
*/
void setActiveCamera(scene::ICameraSceneNode* newActive)
{
	if (0 == Device)
		return;

	scene::ICameraSceneNode * active = Device->getSceneManager()->getActiveCamera();
	active->setInputReceiverEnabled(false);

	newActive->setInputReceiverEnabled(true);
	Device->getSceneManager()->setActiveCamera(newActive);
}

/*
	Set the skin transparency by changing the alpha values of all skin-colors
*/
void setSkinTransparency(s32 alpha, irr::gui::IGUISkin * skin)
{
	for (s32 i=0; i<irr::gui::EGDC_COUNT ; ++i)
	{
		video::SColor col = skin->getColor((EGUI_DEFAULT_COLOR)i);
		col.setAlpha(alpha);
		skin->setColor((EGUI_DEFAULT_COLOR)i, col);
	}
}

/*
  Update the display of the model scaling
*/
void updateScaleInfo(scene::ISceneNode* model)
{
	IGUIElement* toolboxWnd = Device->getGUIEnvironment()->getRootGUIElement()->getElementFromId(GUI_ID_DIALOG_ROOT_WINDOW, true);
	if (!toolboxWnd)
		return;
	if (!model)
	{
		toolboxWnd->getElementFromId(GUI_ID_X_SCALE, true)->setText( L"-" );
		toolboxWnd->getElementFromId(GUI_ID_Y_SCALE, true)->setText( L"-" );
		toolboxWnd->getElementFromId(GUI_ID_Z_SCALE, true)->setText( L"-" );
	}
	else
	{
		core::vector3df scale = model->getScale();
		toolboxWnd->getElementFromId(GUI_ID_X_SCALE, true)->setText( core::stringw(scale.X).c_str() );
		toolboxWnd->getElementFromId(GUI_ID_Y_SCALE, true)->setText( core::stringw(scale.Y).c_str() );
		toolboxWnd->getElementFromId(GUI_ID_Z_SCALE, true)->setText( core::stringw(scale.Z).c_str() );
	}
}

/*
The three following functions do several stuff used by the mesh viewer. The
first function showAboutText() simply displays a messagebox with a caption and
a message text. The texts will be stored in the MessageText and Caption
variables at startup.
*/
void showAboutText()
{
	// create modal message box with the text
	// loaded from the xml file.
	Device->getGUIEnvironment()->addMessageBox(
		Caption.c_str(), MessageText.c_str());
}

/*
The second function loadModel() loads a model and displays it using an
addAnimatedMeshSceneNode and the scene manager. Nothing difficult. It also
displays a short message box, if the model could not be loaded.
*/
void loadModel() //const c8* fn)
{
	// load a model into the engine

	if (Model)
		Model->remove();

	Model = 0;

	scene::IAnimatedMesh* m = Device->getSceneManager()->getMesh( "media/table.3ds"); //filename.c_str() );

	scene::IAnimatedMeshSceneNode* animModel = Device->getSceneManager()->addAnimatedMeshSceneNode(m);
	animModel->setPosition(core::vector3df(0,-100,0));
	Model = animModel;

	scene::ITriangleSelector *selector = 0;
	if(animModel)
	{
// 	    selector = Device->getSceneManager()->createTriangleSelector(animModel); //createOctreeTriangleSelector(m, animModel, 128);
	    selector = Device->getSceneManager()->createOctreeTriangleSelector(m,animModel, 128);
	    animModel->setTriangleSelector(selector);
	}
   
	Model->setMaterialFlag(video::EMF_LIGHTING, UseLight);
	Model->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, UseLight);
//	Model->setMaterialFlag(video::EMF_BACK_FACE_CULLING, false);
	Model->setDebugDataVisible(scene::EDS_OFF);

	
	
	scene::IAnimatedMesh* m2 = Device->getSceneManager()->getMesh( "media/robot2009.3ds"); //filename.c_str() );
	animModel2 = Device->getSceneManager()->addAnimatedMeshSceneNode(m2);

	if(selector)
	{
  	      scene::ISceneNodeAnimator *anim = Device->getSceneManager()->createCollisionResponseAnimator(selector, animModel2, core::vector3df(100, 1, 150),core::vector3df(0,-10,0),core::vector3df(0,-10,0), 0.005);    
	      animModel2->addAnimator(anim);
	      anim->drop();
	}   

	animModel2->setAnimationSpeed(30);
	Model = animModel2;
	Model->setMaterialFlag(video::EMF_LIGHTING, UseLight);
	Model->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, UseLight);
//	Model->setMaterialFlag(video::EMF_BACK_FACE_CULLING, false);
	Model->setDebugDataVisible(scene::EDS_OFF);
	

	
	scene::IAnimatedMesh* m3 = Device->getSceneManager()->getMesh( "media/tomate.3ds"); //filename.c_str() );
	animModel3 = Device->getSceneManager()->addAnimatedMeshSceneNode(m3);
	
	animModel3->setPosition(core::vector3df(0,0,100));
	animModel3->setAnimationSpeed(30);
	animModel3->setPosition(core::vector3df(1000,0,0));
	Model = animModel3;
	Model->setMaterialFlag(video::EMF_LIGHTING, UseLight);
	Model->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, UseLight);
//	Model->setMaterialFlag(video::EMF_BACK_FACE_CULLING, false);
	Model->setDebugDataVisible(scene::EDS_OFF);
	
	// we need to uncheck the menu entries. would be cool to fake a menu event, but
	// that's not so simple. so we do it brute force
	gui::IGUIContextMenu* menu = (gui::IGUIContextMenu*)Device->getGUIEnvironment()->getRootGUIElement()->getElementFromId(GUI_ID_TOGGLE_DEBUG_INFO, true);
	if (menu)
		for(int item = 1; item < 6; ++item)
			menu->setItemChecked(item, false);
	updateScaleInfo(Model);
}


/*
Finally, the third function creates a toolbox window. In this simple mesh
viewer, this toolbox only contains a tab control with three edit boxes for
changing the scale of the displayed model.
*/
void createToolBox()
{
	// remove tool box if already there
	IGUIEnvironment* env = Device->getGUIEnvironment();
	IGUIElement* root = env->getRootGUIElement();
	IGUIElement* e = root->getElementFromId(GUI_ID_DIALOG_ROOT_WINDOW, true);
	if (e)
		e->remove();

	// create the toolbox window
	IGUIWindow* wnd = env->addWindow(core::rect<s32>(0,0,200,435),
		false, L"Toolset", 0, GUI_ID_DIALOG_ROOT_WINDOW);

	// create tab control and tabs
	IGUITabControl* tab = env->addTabControl(
		core::rect<s32>(2,20,800-602,480-7), wnd, true, true);

	IGUITab* t1 = tab->addTab(L"Config");

	// add some edit boxes and a button to tab one
	env->addStaticText(L"Scale:",
			core::rect<s32>(10,20,60,45), false, false, t1);
	env->addStaticText(L"X:", core::rect<s32>(22,48,40,66), false, false, t1);
	env->addEditBox(L"1.0", core::rect<s32>(40,46,130,66), true, t1, GUI_ID_X_SCALE);
	env->addStaticText(L"Y:", core::rect<s32>(22,82,40,96), false, false, t1);
	env->addEditBox(L"1.0", core::rect<s32>(40,76,130,96), true, t1, GUI_ID_Y_SCALE);
	env->addStaticText(L"Z:", core::rect<s32>(22,108,40,126), false, false, t1);
	env->addEditBox(L"1.0", core::rect<s32>(40,106,130,126), true, t1, GUI_ID_Z_SCALE);

	env->addButton(core::rect<s32>(10,134,85,165), t1, GUI_ID_BUTTON_SET_SCALE, L"Set");

	// quick scale buttons
	env->addButton(core::rect<s32>(65,20,95,40), t1, GUI_ID_BUTTON_SCALE_MUL10, L"* 10");
	env->addButton(core::rect<s32>(100,20,130,40), t1, GUI_ID_BUTTON_SCALE_DIV10, L"* 0.1");

	updateScaleInfo(Model);

	// add transparency control
	env->addStaticText(L"GUI Transparency Control:",
			core::rect<s32>(10,200,150,225), true, false, t1);
	IGUIScrollBar* scrollbar = env->addScrollBar(true,
			core::rect<s32>(10,225,150,240), t1, GUI_ID_SKIN_TRANSPARENCY);
	scrollbar->setMax(255);
	scrollbar->setPos(255);

	// add framerate control
	env->addStaticText(L":", core::rect<s32>(10,240,150,265), true, false, t1);
	env->addStaticText(L"Framerate:",
			core::rect<s32>(12,240,75,265), false, false, t1);
	env->addStaticText(L"", core::rect<s32>(75,240,200,265), false, false, t1,
			GUI_ID_ANIMATION_INFO);
	scrollbar = env->addScrollBar(true,
			core::rect<s32>(10,265,150,280), t1, GUI_ID_SKIN_ANIMATION_FPS);
	scrollbar->setMax(MAX_FRAMERATE);
	scrollbar->setMin(-MAX_FRAMERATE);
	scrollbar->setPos(DEFAULT_FRAMERATE);
	scrollbar->setSmallStep(1);
}

void updateToolBox()
{
	IGUIEnvironment* env = Device->getGUIEnvironment();
	IGUIElement* root = env->getRootGUIElement();
	IGUIElement* dlg = root->getElementFromId(GUI_ID_DIALOG_ROOT_WINDOW, true);
	if (!dlg )
		return;

	// update the info we have about the animation of the model
	IGUIStaticText *  aniInfo = (IGUIStaticText *)(dlg->getElementFromId(GUI_ID_ANIMATION_INFO, true));
	if (aniInfo)
	{
		if ( Model && scene::ESNT_ANIMATED_MESH == Model->getType() )
		{
			scene::IAnimatedMeshSceneNode* animatedModel = (scene::IAnimatedMeshSceneNode*)Model;

			core::stringw str( (s32)core::round_(animatedModel->getAnimationSpeed()) );
			str += L" Frame: ";
			str += core::stringw((s32)animatedModel->getFrameNr());
			aniInfo->setText(str.c_str());
		}
		else
			aniInfo->setText(L"");
	}
}

/*
To get all the events sent by the GUI Elements, we need to create an event
receiver. This one is really simple. If an event occurs, it checks the id of
the caller and the event type, and starts an action based on these values. For
example, if a menu item with id GUI_ID_OPEN_MODEL was selected, if opens a file-open-dialog.
*/
class MyEventReceiver : public IEventReceiver
{
public:
	virtual bool OnEvent(const SEvent& event)
	{
		// Escape swaps Camera Input
		if (event.EventType == EET_KEY_INPUT_EVENT &&
			event.KeyInput.PressedDown == false)
		{
			if ( OnKeyUp(event.KeyInput.Key) )
				return true;
		}
		
		// The state of each connected joystick is sent to us
		// once every run() of the Irrlicht device.  Store the
		// state of the first joystick, ignoring other joysticks.
		// This is currently only supported on Windows and Linux.
		if (event.EventType == irr::EET_JOYSTICK_INPUT_EVENT
			&& event.JoystickEvent.Joystick == 0)
		{
			JoystickState = event.JoystickEvent;
		}

		
		if (event.EventType == EET_GUI_EVENT)
		{
			s32 id = event.GUIEvent.Caller->getID();
			IGUIEnvironment* env = Device->getGUIEnvironment();

			switch(event.GUIEvent.EventType)
			{
			case EGET_MENU_ITEM_SELECTED:
					// a menu item was clicked
					OnMenuItemSelected( (IGUIContextMenu*)event.GUIEvent.Caller );
				break;

			case EGET_FILE_SELECTED:
				{
					// load the model file, selected in the file open dialog
					IGUIFileOpenDialog* dialog =
						(IGUIFileOpenDialog*)event.GUIEvent.Caller;
					loadModel(); //core::stringc(dialog->getFileName()).c_str());
				}
				break;

			case EGET_SCROLL_BAR_CHANGED:

				// control skin transparency
				if (id == GUI_ID_SKIN_TRANSPARENCY)
				{
					const s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					setSkinTransparency(pos, env->getSkin());
				}
				// control animation speed
				else if (id == GUI_ID_SKIN_ANIMATION_FPS)
				{
					const s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					if (scene::ESNT_ANIMATED_MESH == Model->getType())
						((scene::IAnimatedMeshSceneNode*)Model)->setAnimationSpeed((f32)pos);
				}
				break;

			case EGET_COMBO_BOX_CHANGED:

				// control anti-aliasing/filtering
				if (id == GUI_ID_TEXTUREFILTER)
				{
					OnTextureFilterSelected( (IGUIComboBox*)event.GUIEvent.Caller );
				}
				break;

			case EGET_BUTTON_CLICKED:

				switch(id)
				{
				case GUI_ID_BUTTON_SET_SCALE:
					{
						// set scale
						gui::IGUIElement* root = env->getRootGUIElement();
						core::vector3df scale;
						core::stringc s;

						s = root->getElementFromId(GUI_ID_X_SCALE, true)->getText();
						scale.X = (f32)atof(s.c_str());
						s = root->getElementFromId(GUI_ID_Y_SCALE, true)->getText();
						scale.Y = (f32)atof(s.c_str());
						s = root->getElementFromId(GUI_ID_Z_SCALE, true)->getText();
						scale.Z = (f32)atof(s.c_str());

						if (Model)
							Model->setScale(scale);
						updateScaleInfo(Model);
					}
					break;
				case GUI_ID_BUTTON_SCALE_MUL10:
					if (Model)
						Model->setScale(Model->getScale()*10.f);
					updateScaleInfo(Model);
					break;
				case GUI_ID_BUTTON_SCALE_DIV10:
					if (Model)
						Model->setScale(Model->getScale()*0.1f);
					updateScaleInfo(Model);
					break;
				case GUI_ID_BUTTON_OPEN_MODEL:
					env->addFileOpenDialog(L"Please select a model file to open");
					break;
				case GUI_ID_BUTTON_SHOW_ABOUT:
					showAboutText();
					break;
				case GUI_ID_BUTTON_SHOW_TOOLBOX:
					createToolBox();
					break;
				case GUI_ID_BUTTON_SELECT_ARCHIVE:
					env->addFileOpenDialog(L"Please select your game archive/directory");
					break;
				}

				break;
			default:
				break;
			}
		}
 
 		return false;
 	}


	/*
		Handle key-up events
	*/
	bool OnKeyUp(irr::EKEY_CODE keyCode)
	{
		if (keyCode == irr::KEY_ESCAPE)
		{
			if (Device)
			{
				scene::ICameraSceneNode * camera =
					Device->getSceneManager()->getActiveCamera();
				if (camera)
				{
					camera->setInputReceiverEnabled( !camera->isInputReceiverEnabled() );
				}
				return true;
			}
		}
		else if (keyCode == irr::KEY_F1)
		{
			if (Device)
			{
				IGUIElement* elem = Device->getGUIEnvironment()->getRootGUIElement()->getElementFromId(GUI_ID_POSITION_TEXT);
				if (elem)
					elem->setVisible(!elem->isVisible());
			}
		}
		else if (keyCode == irr::KEY_KEY_M)
		{
			if (Device)
				Device->minimizeWindow();
		}
		else if (keyCode == irr::KEY_KEY_L)
		{
			UseLight=!UseLight;
			if (Model)
			{
				Model->setMaterialFlag(video::EMF_LIGHTING, UseLight);
				Model->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, UseLight);
			}
		}
		return false;
	}


	/*
		Handle "menu item clicked" events.
	*/
	void OnMenuItemSelected( IGUIContextMenu* menu )
	{
		s32 id = menu->getItemCommandId(menu->getSelectedItem());
		IGUIEnvironment* env = Device->getGUIEnvironment();

		switch(id)
		{
		case GUI_ID_OPEN_MODEL: // FilOnButtonSetScalinge -> Open Model
			env->addFileOpenDialog(L"Please select a model file to open");
			break;
		case GUI_ID_SET_MODEL_ARCHIVE: // File -> Set Model Archive
			env->addFileOpenDialog(L"Please select your game archive/directory");
			break;
		case GUI_ID_LOAD_AS_OCTREE: // File -> LoadAsOctree
			Octree = !Octree;
			menu->setItemChecked(menu->getSelectedItem(), Octree);
			break;
		case GUI_ID_QUIT: // File -> Quit
			Device->closeDevice();
			break;
		case GUI_ID_SKY_BOX_VISIBLE: // View -> Skybox
			menu->setItemChecked(menu->getSelectedItem(), !menu->isItemChecked(menu->getSelectedItem()));
			SkyBox->setVisible(!SkyBox->isVisible());
			break;
		case GUI_ID_DEBUG_OFF: // View -> Debug Information
			menu->setItemChecked(menu->getSelectedItem()+1, false);
			menu->setItemChecked(menu->getSelectedItem()+2, false);
			menu->setItemChecked(menu->getSelectedItem()+3, false);
			menu->setItemChecked(menu->getSelectedItem()+4, false);
			menu->setItemChecked(menu->getSelectedItem()+5, false);
			menu->setItemChecked(menu->getSelectedItem()+6, false);
			if (Model)
				Model->setDebugDataVisible(scene::EDS_OFF);
			break;
		case GUI_ID_DEBUG_BOUNDING_BOX: // View -> Debug Information
			menu->setItemChecked(menu->getSelectedItem(), !menu->isItemChecked(menu->getSelectedItem()));
			if (Model)
				Model->setDebugDataVisible((scene::E_DEBUG_SCENE_TYPE)(Model->isDebugDataVisible()^scene::EDS_BBOX));
			break;
		case GUI_ID_DEBUG_NORMALS: // View -> Debug Information
			menu->setItemChecked(menu->getSelectedItem(), !menu->isItemChecked(menu->getSelectedItem()));
			if (Model)
				Model->setDebugDataVisible((scene::E_DEBUG_SCENE_TYPE)(Model->isDebugDataVisible()^scene::EDS_NORMALS));
			break;
		case GUI_ID_DEBUG_SKELETON: // View -> Debug Information
			menu->setItemChecked(menu->getSelectedItem(), !menu->isItemChecked(menu->getSelectedItem()));
			if (Model)
				Model->setDebugDataVisible((scene::E_DEBUG_SCENE_TYPE)(Model->isDebugDataVisible()^scene::EDS_SKELETON));
			break;
		case GUI_ID_DEBUG_WIRE_OVERLAY: // View -> Debug Information
			menu->setItemChecked(menu->getSelectedItem(), !menu->isItemChecked(menu->getSelectedItem()));
			if (Model)
				Model->setDebugDataVisible((scene::E_DEBUG_SCENE_TYPE)(Model->isDebugDataVisible()^scene::EDS_MESH_WIRE_OVERLAY));
			break;
		case GUI_ID_DEBUG_HALF_TRANSPARENT: // View -> Debug Information
			menu->setItemChecked(menu->getSelectedItem(), !menu->isItemChecked(menu->getSelectedItem()));
			if (Model)
				Model->setDebugDataVisible((scene::E_DEBUG_SCENE_TYPE)(Model->isDebugDataVisible()^scene::EDS_HALF_TRANSPARENCY));
			break;
		case GUI_ID_DEBUG_BUFFERS_BOUNDING_BOXES: // View -> Debug Information
			menu->setItemChecked(menu->getSelectedItem(), !menu->isItemChecked(menu->getSelectedItem()));
			if (Model)
				Model->setDebugDataVisible((scene::E_DEBUG_SCENE_TYPE)(Model->isDebugDataVisible()^scene::EDS_BBOX_BUFFERS));
			break;
		case GUI_ID_DEBUG_ALL: // View -> Debug Information
			menu->setItemChecked(menu->getSelectedItem()-1, true);
			menu->setItemChecked(menu->getSelectedItem()-2, true);
			menu->setItemChecked(menu->getSelectedItem()-3, true);
			menu->setItemChecked(menu->getSelectedItem()-4, true);
			menu->setItemChecked(menu->getSelectedItem()-5, true);
			menu->setItemChecked(menu->getSelectedItem()-6, true);
			if (Model)
				Model->setDebugDataVisible(scene::EDS_FULL);
			break;
		case GUI_ID_ABOUT: // Help->About
			showAboutText();
			break;
		case GUI_ID_MODEL_MATERIAL_SOLID: // View -> Material -> Solid
			if (Model)
				Model->setMaterialType(video::EMT_SOLID);
			break;
		case GUI_ID_MODEL_MATERIAL_TRANSPARENT: // View -> Material -> Transparent
			if (Model)
				Model->setMaterialType(video::EMT_TRANSPARENT_ADD_COLOR);
			break;
		case GUI_ID_MODEL_MATERIAL_REFLECTION: // View -> Material -> Reflection
			if (Model)
				Model->setMaterialType(video::EMT_SPHERE_MAP);
			break;

		case GUI_ID_CAMERA_MAYA:
			setActiveCamera(Camera[0]);
			break;
		case GUI_ID_CAMERA_FIRST_PERSON:
			setActiveCamera(Camera[1]);
			break;
		}
	}

	/*
		Handle the event that one of the texture-filters was selected in the corresponding combobox.
	*/
	void OnTextureFilterSelected( IGUIComboBox* combo )
	{
		s32 pos = combo->getSelected();
		switch (pos)
		{
			case 0:
			if (Model)
			{
				Model->setMaterialFlag(video::EMF_BILINEAR_FILTER, false);
				Model->setMaterialFlag(video::EMF_TRILINEAR_FILTER, false);
				Model->setMaterialFlag(video::EMF_ANISOTROPIC_FILTER, false);
			}
			break;
			case 1:
			if (Model)
			{
				Model->setMaterialFlag(video::EMF_BILINEAR_FILTER, true);
				Model->setMaterialFlag(video::EMF_TRILINEAR_FILTER, false);
			}
			break;
			case 2:
			if (Model)
			{
				Model->setMaterialFlag(video::EMF_BILINEAR_FILTER, false);
				Model->setMaterialFlag(video::EMF_TRILINEAR_FILTER, true);
			}
			break;
			case 3:
			if (Model)
			{
				Model->setMaterialFlag(video::EMF_ANISOTROPIC_FILTER, true);
			}
			break;
			case 4:
			if (Model)
			{
				Model->setMaterialFlag(video::EMF_ANISOTROPIC_FILTER, false);
			}
			break;
		}
	}
	//JOYSTICK
	const SEvent::SJoystickEvent & GetJoystickState(void) const
	{
		return JoystickState;
	}


	MyEventReceiver()
	{
	}

private:
	SEvent::SJoystickEvent JoystickState;
};
#endif

/*
Most of the hard work is done. We only need to create the Irrlicht Engine
device and all the buttons, menus and toolbars. We start up the engine as
usual, using createDevice(). To make our application catch events, we set our
eventreceiver as parameter. As you can see, there is also a call to
IrrlichtDevice::setResizeable(). This makes the render window resizeable, which
is quite useful for a mesh viewer.
*/
int main(int argc, char* argv[])
{
	(void) argc;
	(void) argv;

	Environnement env;

	bool continuer;

	do
	{
		continuer = env.update();
		usleep(100000);
	} while(continuer);

#if 0
	// create device and exit if creation failed
	MyEventReceiver receiver;
	Device = createDevice(video::EDT_OPENGL, core::dimension2d<u32>(1080, 960),
		16, false, false, false, &receiver);

	if (Device == 0)
		return 1; // could not create selected driver.
		
	//JOYSITCK init
	core::array<SJoystickInfo> joystickInfo;
	if(Device->activateJoysticks(joystickInfo))
	{
		std::cout << "Joystick support is enabled and " << joystickInfo.size() << " joystick(s) are present." << std::endl;

		for(u32 joystick = 0; joystick < joystickInfo.size(); ++joystick)
		{
			std::cout << "Joystick " << joystick << ":" << std::endl;
			std::cout << "\tName: '" << joystickInfo[joystick].Name.c_str() << "'" << std::endl;
			std::cout << "\tAxes: " << joystickInfo[joystick].Axes << std::endl;
			std::cout << "\tButtons: " << joystickInfo[joystick].Buttons << std::endl;

			std::cout << "\tHat is: ";

			switch(joystickInfo[joystick].PovHat)
			{
			case SJoystickInfo::POV_HAT_PRESENT:
				std::cout << "present" << std::endl;
				break;

			case SJoystickInfo::POV_HAT_ABSENT:
				std::cout << "absent" << std::endl;
				break;

			case SJoystickInfo::POV_HAT_UNKNOWN:
			default:
				std::cout << "unknown" << std::endl;
				break;
			}
		}
	}
	else
	{
		std::cout << "Joystick support is not enabled." << std::endl;
	}

	core::stringw tmp = L"Irrlicht Joystick Example (";
	tmp += joystickInfo.size();
	tmp += " joysticks)";
	Device->setWindowCaption(tmp.c_str());
	//end init
	
	Device->setResizable(true);

	Device->setWindowCaption(L"Irrlicht Engine - Loading...");

	video::IVideoDriver* driver = Device->getVideoDriver();
	IGUIEnvironment* env = Device->getGUIEnvironment();
	scene::ISceneManager* smgr = Device->getSceneManager();
	smgr->getParameters()->setAttribute(scene::COLLADA_CREATE_SCENE_INSTANCES, true);

	driver->setTextureCreationFlag(video::ETCF_ALWAYS_32_BIT, true);

	smgr->addLightSceneNode(0, core::vector3df(200,200,200),
		video::SColorf(1.0f,1.0f,1.0f),2000);
	smgr->setAmbientLight(video::SColorf(0.3f,0.3f,0.3f));
	// add our media directory as "search path"
	Device->getFileSystem()->addFolderFileArchive("media/");

	// disable alpha

	for (s32 i=0; i<gui::EGDC_COUNT ; ++i)
	{
		video::SColor col = env->getSkin()->getColor((gui::EGUI_DEFAULT_COLOR)i);
		col.setAlpha(255);
		env->getSkin()->setColor((gui::EGUI_DEFAULT_COLOR)i, col);
	}

	// add a tabcontrol

  	createToolBox();

	// add skybox

	SkyBox = smgr->addSkyBoxSceneNode(
		driver->getTexture("irrlicht2_up.jpg"),
		driver->getTexture("irrlicht2_dn.jpg"),
		driver->getTexture("irrlicht2_lf.jpg"),
		driver->getTexture("irrlicht2_rt.jpg"),
		driver->getTexture("irrlicht2_ft.jpg"),
		driver->getTexture("irrlicht2_bk.jpg"));

	loadModel();
	
	// add a camera scene node
	Camera[0] = smgr->addCameraSceneNodeMaya();
	Camera[0]->setFarValue(20000.f);
	// Maya cameras reposition themselves relative to their target, so target the location
	// where the mesh scene node is placed.
	Camera[0]->setTarget(core::vector3df(0,30,0));

	Camera[1] = smgr->addCameraSceneNodeFPS();
	Camera[1]->setFarValue(20000.f);
	Camera[1]->setPosition(core::vector3df(0,1000,-2000));
	Camera[1]->setTarget(core::vector3df(0,30,0));

	setActiveCamera(Camera[0]);

	// draw everything
	// As in example 04, we'll use framerate independent movement.
	u32 then = Device->getTimer()->getTime();
	const f32 MOVEMENT_SPEED = 100.f;//5.f;
	while(Device->run() && driver)
	{
// 		if (Device->isWindowActive())
// 		{
			// Work out a frame delta time.
			const u32 now = Device->getTimer()->getTime();
			const f32 frameDeltaTime = (f32)(now - then) / 1000.f; // Time in seconds
			then = now;
			core::vector3df nodePosition = animModel2->getPosition();
			if(joystickInfo.size() > 0)
			{
				f32 moveHorizontal = 0.f; // Range is -1.f for full left to +1.f for full right
				f32 moveVertical = 0.f; // -1.f for full down to +1.f for full up.

				const SEvent::SJoystickEvent & joystickData = receiver.GetJoystickState();

				// We receive the full analog range of the axes, and so have to implement our
				// own dead zone.  This is an empirical value, since some joysticks have more
				// jitter or creep around the center point than others.  We'll use 5% of the
				// range as the dead zone, but generally you would want to give the user the
				// option to change this.
				const f32 DEAD_ZONE = 0.05f;
				std::cout << "Start Arrow X=" << nodePosition.X << "Arrow Y=" << nodePosition.Y << std::endl;
				
// 				std::cout << "Joystick X=" << (f32)joystickData.Axis[SEvent::SJoystickEvent::AXIS_X] << " Y=" << (f32)joystickData.Axis[SEvent::SJoystickEvent::AXIS_Y] << std::endl;
				moveHorizontal =
					(f32)joystickData.Axis[SEvent::SJoystickEvent::AXIS_X] / 32767.f;
				if(fabs(moveHorizontal) < DEAD_ZONE)
					moveHorizontal = 0.f;

				moveVertical =
					(f32)joystickData.Axis[SEvent::SJoystickEvent::AXIS_Y] / -32767.f;
				if(fabs(moveVertical) < DEAD_ZONE)
					moveVertical = 0.f;

				if(!core::equals(moveHorizontal, 0.f) || !core::equals(moveVertical, 0.f))
				{
// 					std::cout << "moveHorizontal2=" << moveHorizontal << "moveVertical2=" << moveVertical << std::endl;
					nodePosition.Z += MOVEMENT_SPEED * frameDeltaTime * moveHorizontal;
	// 				if(nodePosition.Y < 5.5 ) 	
					  nodePosition.X += MOVEMENT_SPEED * frameDeltaTime * moveVertical;
// 					movedWithJoystick = true;
				}
				std::cout << "End Arrow X=" << nodePosition.X << "Arrow Y=" << nodePosition.Y << std::endl;
				animModel2->setPosition(nodePosition);
			}

  
  
			driver->beginScene(true, true, video::SColor(150,50,50,50));

			smgr->drawAll();
			env->drawAll();

			driver->endScene();
	}

	Device->drop();
#endif
	return 0;
}

