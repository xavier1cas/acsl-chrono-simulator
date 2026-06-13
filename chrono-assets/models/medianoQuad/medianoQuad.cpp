// C++ Chrono::Engine model automatically generated using Chrono::SolidWorks add-in
// Assembly: D:\Dropbox\Projects\Test_drone\CAD\ACSL_Test_Drone\Export Files\S3.SLDASM


#include <string>
#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChContactMaterialNSC.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"
#include "medianoQuad.h"


/// Function to import Solidworks assembly directly into Chrono ChSystem.
void ImportSolidworksSystemCpp(chrono::ChSystem& system, std::unordered_map<std::string, std::shared_ptr<chrono::ChFunction>>* motfun_map) {
std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> bodylist;
std::vector<std::shared_ptr<chrono::ChLinkBase>> linklist;
ImportSolidworksSystemCpp(bodylist, linklist, motfun_map);
for (auto& body : bodylist)
    system.Add(body);
for (auto& link : linklist)
    system.Add(link);
}


/// Function to import Solidworks bodies and mates into dedicated containers.
void ImportSolidworksSystemCpp(std::vector<std::shared_ptr<chrono::ChBodyAuxRef>>& bodylist, std::vector<std::shared_ptr<chrono::ChLinkBase>>& linklist, std::unordered_map<std::string, std::shared_ptr<chrono::ChFunction>>* motfun_map) {

// Some global settings
double sphereswept_r = 0.001;
chrono::ChCollisionModel::SetDefaultSuggestedEnvelope(0.003);
chrono::ChCollisionModel::SetDefaultSuggestedMargin(0.003);
chrono::ChCollisionSystemBullet::SetContactBreakingThreshold(0.002);

std::string shapes_dir = "medianoQuad_shapes/";

// Prepare some data for later use
std::shared_ptr<chrono::ChVisualShapeModelFile> body_shape;
chrono::ChMatrix33<> mr;
std::shared_ptr<chrono::ChLinkBase> link;
chrono::ChVector3d cA;
chrono::ChVector3d cB;
chrono::ChVector3d dA;
chrono::ChVector3d dB;

// Assembly ground body
auto body_0 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_0->SetName("SLDW_GROUND");
body_0->SetFixed(true);
bodylist.push_back(body_0);

// Rigid body part
auto body_1 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_1->SetName("S2-2");
body_1->SetPos(chrono::ChVector3d(-0.00028213672964037,0.000541686303864703,0.000193471194123856));
body_1->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_1->SetMass(1.04291239130921);
body_1->SetInertiaXX(chrono::ChVector3d(0.00356507620334993,0.00408659486216925,0.00606507369923718));
body_1->SetInertiaXY(chrono::ChVector3d(-4.08770048558901e-06,-1.15603516031003e-05,-8.65500570931156e-07));
body_1->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-0.00149597132207608,-0.0145247411685757,-0.0104019824317649),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_1->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// Collision Model
body_1->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

// Collision material
auto mat_1 = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

// Collision shape
std::shared_ptr<chrono::ChCollisionShape> collshape_1;
mr(0,0)=-1; mr(1,0)=0; mr(2,0)=-4.15528994464277E-15;
mr(0,1)=0; mr(1,1)=-1; mr(2,1)=5.57299002691835E-15;
mr(0,2)=-4.15528994464277E-15; mr(1,2)=5.57299002691835E-15; mr(2,2)=1;
collshape_1 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_1,0.243804529519431,0.27143039261605,0.116);
body_1->GetCollisionModel()->AddShape(collshape_1,chrono::ChFramed(chrono::ChVector3d(1.23003869083905E-15,-0.0145000000000017,-0.0134999999999997), mr));
body_1->EnableCollision(true);

bodylist.push_back(body_1);



// Rigid body part
auto body_2 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_2->SetName("Propeller CCW-2");
body_2->SetPos(chrono::ChVector3d(-0.0971844014893548,-0.124673510004162,-0.0307815288058759));
body_2->SetRot(chrono::ChQuaternion<>(0.707106781186549,0.707106781186546,0,2.39756158981543e-15));
body_2->SetMass(0.00327419527150974);
body_2->SetInertiaXX(chrono::ChVector3d(1.05208970154214e-06,1.0522950943804e-06,2.08055101948593e-06));
body_2->SetInertiaXY(chrono::ChVector3d(-2.58085779370827e-10,2.4109618758717e-11,-6.37155034569005e-11));
body_2->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(7.33156025842792e-06,-0.0084495854883779,-7.75309227200287e-06),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_2_1.obj");
body_2->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_2);



// Rigid body part
auto body_3 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_3->SetName("Propeller CW-2");
body_3->SetPos(chrono::ChVector3d(-0.064142539183858,0.147702698725156,-0.0378315288058766));
body_3->SetRot(chrono::ChQuaternion<>(0.707106781186546,-0.707106781186549,0,-2.46885013108236e-15));
body_3->SetMass(0.00327626318825968);
body_3->SetInertiaXX(chrono::ChVector3d(1.05254980157665e-06,1.0529278562727e-06,2.08159752619661e-06));
body_3->SetInertiaXY(chrono::ChVector3d(-7.80398918066972e-11,-1.13859879865916e-11,-2.32585720829613e-12));
body_3->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-0.033036952915471,0.0013976318628222,-0.0509441435522827),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_3_1.obj");
body_3->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_3);



// Rigid body part
auto body_4 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_4->SetName("Propeller CW-1");
body_4->SetPos(chrono::ChVector3d(0.129661990335559,-0.0737276938909061,-0.0378315288058753));
body_4->SetRot(chrono::ChQuaternion<>(0.707106781186546,-0.707106781186549,0,-2.46885013108232e-15));
body_4->SetMass(0.00327626318825968);
body_4->SetInertiaXX(chrono::ChVector3d(1.05254980157665e-06,1.0529278562727e-06,2.08159752619661e-06));
body_4->SetInertiaXY(chrono::ChVector3d(-7.80398918066972e-11,-1.13859879865916e-11,-2.32585720829613e-12));
body_4->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-0.033036952915471,0.0013976318628222,-0.0509441435522827),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_3_1.obj");
body_4->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_4);



// Rigid body part
auto body_5 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_5->SetName("Propeller CCW-1");
body_5->SetPos(chrono::ChVector3d(0.0966201280300766,0.0967568826118878,-0.0307815288058759));
body_5->SetRot(chrono::ChQuaternion<>(0.707106781186549,0.707106781186546,0,2.54517387617414e-15));
body_5->SetMass(0.00327419527150974);
body_5->SetInertiaXX(chrono::ChVector3d(1.05208970154214e-06,1.0522950943804e-06,2.08055101948593e-06));
body_5->SetInertiaXY(chrono::ChVector3d(-2.58085779370826e-10,2.41096187585024e-11,-6.37155034566722e-11));
body_5->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(7.33156025842792e-06,-0.0084495854883779,-7.75309227200287e-06),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_2_1.obj");
body_5->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_5);




// Mate constraint: Concentric1 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: S2-2 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_5 , SW name: Propeller CCW-1 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.0966201280300767,0.0967568826118877,-0.0463065288058759);
dA = chrono::ChVector3d(-3.59941941428318e-15,3.43362833808333e-15,1);
cB = chrono::ChVector3d(0.0966201280300767,0.0967568826118877,-0.0343065288058759);
dB = chrono::ChVector3d(3.59941941428318e-15,-3.39190141383284e-15,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_5,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric1");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(0.0966201280300767,0.0967568826118877,-0.0463065288058759);
cB = chrono::ChVector3d(0.0966201280300767,0.0967568826118877,-0.0343065288058759);
dA = chrono::ChVector3d(-3.59941941428318e-15,3.43362833808333e-15,1);
dB = chrono::ChVector3d(3.59941941428318e-15,-3.39190141383284e-15,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_1,body_5,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric1");
linklist.push_back(link);


// Mate constraint: Coincident1 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: S2-2 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_5 , SW name: Propeller CCW-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(0.0961849811274249,0.0992187207814498,-0.0343065288058759);
cB = chrono::ChVector3d(0.0966201280300767,0.0967568826118877,-0.0343065288058759);
dA = chrono::ChVector3d(3.59941941428318e-15,-3.43362833808333e-15,-1);
dB = chrono::ChVector3d(-3.59941941428318e-15,3.39190141383284e-15,1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_5,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident1");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.0961849811274249,0.0992187207814498,-0.0343065288058759);
dA = chrono::ChVector3d(3.59941941428318e-15,-3.43362833808333e-15,-1);
cB = chrono::ChVector3d(0.0966201280300767,0.0967568826118877,-0.0343065288058759);
dB = chrono::ChVector3d(-3.59941941428318e-15,3.39190141383284e-15,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_5,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident1");
linklist.push_back(link);


// Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: S2-2 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_2 , SW name: Propeller CCW-2 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.0971844014893547,-0.124673510004162,-0.0463065288058759);
dA = chrono::ChVector3d(-3.39066411694179e-15,3.56100461260611e-15,1);
cB = chrono::ChVector3d(-0.0971844014893547,-0.124673510004162,-0.0343065288058759);
dB = chrono::ChVector3d(3.39066411694179e-15,-3.61394601875787e-15,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric2");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-0.0971844014893547,-0.124673510004162,-0.0463065288058759);
cB = chrono::ChVector3d(-0.0971844014893547,-0.124673510004162,-0.0343065288058759);
dA = chrono::ChVector3d(-3.39066411694179e-15,3.56100461260611e-15,1);
dB = chrono::ChVector3d(3.39066411694179e-15,-3.61394601875787e-15,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_1,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric2");
linklist.push_back(link);


// Mate constraint: Coincident2 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: S2-2 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_2 , SW name: Propeller CCW-2 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-0.0967492545867029,-0.127135348173724,-0.0343065288058759);
cB = chrono::ChVector3d(-0.0971844014893547,-0.124673510004162,-0.0343065288058759);
dA = chrono::ChVector3d(3.39066411694179e-15,-3.56100461260611e-15,-1);
dB = chrono::ChVector3d(-3.39066411694179e-15,3.61394601875787e-15,1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_2,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident2");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.0967492545867029,-0.127135348173724,-0.0343065288058759);
dA = chrono::ChVector3d(3.39066411694179e-15,-3.56100461260611e-15,-1);
cB = chrono::ChVector3d(-0.0971844014893547,-0.124673510004162,-0.0343065288058759);
dB = chrono::ChVector3d(-3.39066411694179e-15,3.61394601875787e-15,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident2");
linklist.push_back(link);


// Mate constraint: Concentric3 [MateConcentric] type:1 align:0 flip:False
//   Entity 0: C::E name: body_1 , SW name: S2-2 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_4 , SW name: Propeller CW-1 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.0966201280300686,-0.124673510004166,-0.0463065288058752);
dA = chrono::ChVector3d(-3.49148133884321e-15,3.49148133884312e-15,1);
cB = chrono::ChVector3d(0.0966201280300686,-0.124673510004166,-0.0343065288058752);
dB = chrono::ChVector3d(-3.49148133884321e-15,3.49148133884313e-15,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_4,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric3");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(0.0966201280300686,-0.124673510004166,-0.0463065288058752);
cB = chrono::ChVector3d(0.0966201280300686,-0.124673510004166,-0.0343065288058752);
dA = chrono::ChVector3d(-3.49148133884321e-15,3.49148133884312e-15,1);
dB = chrono::ChVector3d(-3.49148133884321e-15,3.49148133884313e-15,1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_1,body_4,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric3");
linklist.push_back(link);


// Mate constraint: Coincident3 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: S2-2 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_4 , SW name: Propeller CW-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(0.0990819661996306,-0.125108656906818,-0.0343065288058752);
cB = chrono::ChVector3d(0.0966201280300686,-0.124673510004166,-0.0343065288058752);
dA = chrono::ChVector3d(3.49148133884321e-15,-3.49148133884312e-15,-1);
dB = chrono::ChVector3d(-3.49148133884321e-15,3.49148133884313e-15,1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_4,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident3");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.0990819661996306,-0.125108656906818,-0.0343065288058752);
dA = chrono::ChVector3d(3.49148133884321e-15,-3.49148133884312e-15,-1);
cB = chrono::ChVector3d(0.0966201280300686,-0.124673510004166,-0.0343065288058752);
dB = chrono::ChVector3d(-3.49148133884321e-15,3.49148133884313e-15,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_4,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident3");
linklist.push_back(link);


// Mate constraint: Concentric4 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: S2-2 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_3 , SW name: Propeller CW-2 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.0971844014893485,0.0967568826118956,-0.0468065288058766);
dA = chrono::ChVector3d(3.49148133884326e-15,-3.4914813388431e-15,-1);
cB = chrono::ChVector3d(-0.0971844014893485,0.0967568826118956,-0.0343065288058766);
dB = chrono::ChVector3d(-3.49148133884326e-15,3.49148133884313e-15,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric4");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-0.0971844014893485,0.0967568826118956,-0.0468065288058766);
cB = chrono::ChVector3d(-0.0971844014893485,0.0967568826118956,-0.0343065288058766);
dA = chrono::ChVector3d(3.49148133884326e-15,-3.4914813388431e-15,-1);
dB = chrono::ChVector3d(-3.49148133884326e-15,3.49148133884313e-15,1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_1,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric4");
linklist.push_back(link);


// Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: S2-2 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_3 , SW name: Propeller CW-2 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-0.0996462396589105,0.0971920295145476,-0.0343065288058766);
cB = chrono::ChVector3d(-0.0971844014893485,0.0967568826118956,-0.0343065288058766);
dA = chrono::ChVector3d(3.49148133884326e-15,-3.4914813388431e-15,-1);
dB = chrono::ChVector3d(-3.49148133884326e-15,3.49148133884313e-15,1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_3,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident4");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.0996462396589105,0.0971920295145476,-0.0343065288058766);
dA = chrono::ChVector3d(3.49148133884326e-15,-3.4914813388431e-15,-1);
cB = chrono::ChVector3d(-0.0971844014893485,0.0967568826118956,-0.0343065288058766);
dB = chrono::ChVector3d(-3.49148133884326e-15,3.49148133884313e-15,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident4");
linklist.push_back(link);


// Auxiliary marker (coordinate system feature)
auto marker_0_1 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_1->SetName("motor_1");
body_0->AddMarker(marker_0_1);
marker_0_1->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(0.0966201280300767,0.0967568826118877,-0.0468065288058759),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_2 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_2->SetName("motor_2");
body_0->AddMarker(marker_0_2);
marker_0_2->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.0971844014893547,-0.124673510004162,-0.0468065288058758),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_3 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_3->SetName("motor_3");
body_0->AddMarker(marker_0_3);
marker_0_3->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(0.0966201280300686,-0.124673510004166,-0.0468065288058751),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_4 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_4->SetName("motor_4");
body_0->AddMarker(marker_0_4);
marker_0_4->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.0971844014893485,0.0967568826118956,-0.0468065288058766),chrono::ChQuaternion<>(1,0,0,0)));


} // end function
