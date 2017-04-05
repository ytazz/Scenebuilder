#include <Springhead.h>
using namespace Spr;

#include <Scenebuilder.h>
using namespace Scenebuilder;

class Robotarm : public Builder{
public:
	bool	fromFile;

	void BuildFromFile(){
		Parse("robotarm.xml");
	}

	void BuildManually(){
		Mass(1.0);
		Inertia(1.0);

		BoxSize(1.0, 1.0, 1.0);
		Box("box_base");
		BoxSize(0.3, 1.0, 0.3);
		Box("box_link");
		
		Push();
		Dynamical(false);
		Body("base");
		{
			Shape("../box_base");

			MoveY(0.5);
			RotX(Rad(-90.0));
			Connector();
		}
		End();
		Pop();
		
		MoveY(0.5);
		MoveY(0.9);

		Push();
		Body("link");
		{
			Shape("../box_link");
			RotX(Rad(-90.0));
			Connector();
			//JointPos(0.0);
			//Joint();
			//RotX(Rad(90.0));
		}
		End();
		Pop();

		Hinge("base/1", "link/1");
	}

	void Build(){
		if(fromFile)
			 BuildFromFile();
		else BuildManually();
	}
	
	void Loop(){
		
	}

} robot;

ServerService*		server;
ViewerService*		viewer;
SimulatorService*	simulator;

SceneLocal		scene;
MMTimer			mmtimer;
uint			updatePeriod;

/// タイマコールバック関数
void TimeProc(){
	robot.Loop();
}

const char* SMNAME = "RobotArm example";

int main(int argc, char* argv[]){
	// サービス起動
	/*Service::SetServicePath("..\\..\\bin\\");
	server = ServerService::Start();
	viewer = ViewerService::Start();
	simulator = SimulatorService::Start();
	if(!server || !viewer || !simulator){
		Message::Out("failed to start services");
		return 0;
	}
	server->Create(SMNAME);
	viewer->Open(SMNAME);*/

	// シーンを開く
	scene.Open(SMNAME);
	if(!scene.IsOpen()){
		Message::Out("failed to open shared memory");
		return 0;
	}

	// 構築
	robot.fromFile = false;
	robot.SetScene(&scene, -1);
	robot.Build();

	// マルチメディアタイマの設定
	mmtimer.Init(&TimeProc);
	mmtimer.Start(updatePeriod);
	Message::Out("timer started: period %d [ms]", updatePeriod);

	while(true){
		Message::Out("input command: ");
		char ch = getchar();
		
		switch(ch){
		case 'q':
			Message::Out("received quit command. exiting...");
			return 0;
		}
	};

	return 0;
}
