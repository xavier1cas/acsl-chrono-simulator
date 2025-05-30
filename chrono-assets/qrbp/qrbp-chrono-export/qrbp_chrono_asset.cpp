// C++ Chrono::Engine model automatically generated using Chrono::SolidWorks add-in
// Assembly: D:\Dropbox\Projects\ROS2\acsl-physics-sim\chrono-assets\qrbp\qrbp-solidworks\qrbp_assembly.SLDASM


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
#include "qrbp_chrono_asset.h"


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

std::string shapes_dir = "qrbp_chrono_asset_shapes/";

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
body_1->SetName("prop_4-1");
body_1->SetPos(chrono::ChVector3d(-0.00942816685338454,0.308068237403483,0.154498963796847));
body_1->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_1->SetMass(0.00395402309271749);
body_1->SetInertiaXX(chrono::ChVector3d(2.83955219476957e-06,2.84026926028763e-06,5.66580058802151e-06));
body_1->SetInertiaXY(chrono::ChVector3d(7.53921615234973e-10,-1.55407912210431e-11,-5.50513541652702e-11));
body_1->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-5.52286503295572e-10,3.32677464985852e-06,-0.000645546072066471),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_1->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_1);



// Rigid body part
auto body_2 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_2->SetName("prop_2-1");
body_2->SetPos(chrono::ChVector3d(-0.00942816685338318,0.086637844787446,0.154498963796845));
body_2->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_2->SetMass(0.00395402309271749);
body_2->SetInertiaXX(chrono::ChVector3d(2.83955219476957e-06,2.84026926028763e-06,5.66580058802151e-06));
body_2->SetInertiaXY(chrono::ChVector3d(7.53921615234973e-10,-1.55407912210431e-11,-5.50513541652702e-11));
body_2->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-5.52286503295572e-10,3.32677464985852e-06,-0.000645546072066471),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_2_1.obj");
body_2->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_2);



// Rigid body part
auto body_3 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_3->SetName("prop_3-1");
body_3->SetPos(chrono::ChVector3d(0.184376362665988,0.0866378447874975,0.154498963796845));
body_3->SetRot(chrono::ChQuaternion<>(0.642220241731439,-3.53271206234227e-15,1.51678818982111e-15,0.766520163538058));
body_3->SetMass(0.00395402309271749);
body_3->SetInertiaXX(chrono::ChVector3d(2.83955219476957e-06,2.84026926028763e-06,5.66580058802151e-06));
body_3->SetInertiaXY(chrono::ChVector3d(7.53921615234973e-10,-1.55407912210431e-11,-5.50513541652702e-11));
body_3->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-5.52286503295572e-10,3.32677464985852e-06,-0.000645546072066471),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_3_1.obj");
body_3->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_3);



// Rigid body part
auto body_4 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_4->SetName("sqrbp-2");
body_4->SetPos(chrono::ChVector3d(0.0984471609373536,0.187071795527612,0.17256855449029));
body_4->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_4->SetMass(1.67236138725861);
body_4->SetInertiaXX(chrono::ChVector3d(0.0233070324786724,0.0160580843956144,0.0271717864469372));
body_4->SetInertiaXY(chrono::ChVector3d(1.10006555036583e-07,-0.000482088295393743,1.23260346821255e-06));
body_4->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-0.0163151142227842,0.0102080920244478,0.017016814810521),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_4_1.obj");
body_4->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// Collision Model
body_4->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

// Collision material
auto mat_4 = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

// Collision shape
std::shared_ptr<chrono::ChCollisionShape> collshape_4;
mr(0,0)=-1; mr(1,0)=-3.38384126574542E-15; mr(2,0)=0;
mr(0,1)=-1.13691526083878E-16; mr(1,1)=6.82149156503269E-15; mr(2,1)=-1;
mr(0,2)=3.38384126574542E-15; mr(1,2)=-1; mr(2,2)=-6.82149156503269E-15;
collshape_4 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_4,0.0164047740043827,0.122065278616947,0.5);
body_4->GetCollisionModel()->AddShape(collshape_4,chrono::ChFramed(chrono::ChVector3d(0.0859073517911931,0.0102797625509052,0.0890880486150245), mr));
mr(0,0)=-1; mr(1,0)=1.85656158318431E-16; mr(2,0)=-3.37275354278483E-15;
mr(0,1)=0; mr(1,1)=1; mr(2,1)=5.73186685091805E-15;
mr(0,2)=3.37275354278483E-15; mr(1,2)=5.73186685091805E-15; mr(2,2)=-1;
collshape_4 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_4,0.224249837982941,0.265117422382603,0.114399999999996);
body_4->GetCollisionModel()->AddShape(collshape_4,chrono::ChFramed(chrono::ChVector3d(-0.0109730630310874,0.0102812455678535,-0.0391445906934458), mr));
mr(0,0)=-1; mr(1,0)=-3.38384126574542E-15; mr(2,0)=0;
mr(0,1)=1.13691526083878E-16; mr(1,1)=-7.04887461720045E-15; mr(2,1)=1;
mr(0,2)=-3.38384126574542E-15; mr(1,2)=1; mr(2,2)=7.04887461720045E-15;
collshape_4 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_4,0.0164047740043827,0.122065278616947,0.5);
body_4->GetCollisionModel()->AddShape(collshape_4,chrono::ChFramed(chrono::ChVector3d(-0.107892648208808,0.0102797625509045,0.0890880486150274), mr));
body_4->EnableCollision(true);

bodylist.push_back(body_4);



// Rigid body part
auto body_5 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_5->SetName("prop_1-2");
body_5->SetPos(chrono::ChVector3d(0.184376362665965,0.308068237403556,0.154498963796847));
body_5->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_5->SetMass(0.00395402309271749);
body_5->SetInertiaXX(chrono::ChVector3d(2.83955219476957e-06,2.84026926028763e-06,5.66580058802151e-06));
body_5->SetInertiaXY(chrono::ChVector3d(7.53921615234973e-10,-1.55407912210431e-11,-5.50513541652702e-11));
body_5->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-5.52286503295572e-10,3.32677464985852e-06,-0.000645546072066471),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_5_1.obj");
body_5->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_5);




// Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_4 , SW name: sqrbp-2 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_5 , SW name: prop_1-2 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.184376362665965,0.308068237403556,0.152623963796844);
dA = chrono::ChVector3d(-3.49148133884316e-15,6.98296267768625e-15,1);
cB = chrono::ChVector3d(0.184376362665965,0.308068237403556,0.150873963796847);
dB = chrono::ChVector3d(0,0,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_4,body_5,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric2");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(0.184376362665965,0.308068237403556,0.152623963796844);
cB = chrono::ChVector3d(0.184376362665965,0.308068237403556,0.150873963796847);
dA = chrono::ChVector3d(-3.49148133884316e-15,6.98296267768625e-15,1);
dB = chrono::ChVector3d(0,0,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_4,body_5,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric2");
linklist.push_back(link);


// Mate constraint: Coincident3 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_4 , SW name: sqrbp-2 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_5 , SW name: prop_1-2 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(0.184871898993746,0.309085313481266,0.158123963796845);
cB = chrono::ChVector3d(0.184376362665965,0.308068237403556,0.158123963796847);
dA = chrono::ChVector3d(1.20533813419051e-17,1.72292970257481e-17,-1);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_4,body_5,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident3");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.184871898993746,0.309085313481266,0.158123963796845);
dA = chrono::ChVector3d(1.20533813419051e-17,1.72292970257481e-17,-1);
cB = chrono::ChVector3d(0.184376362665965,0.308068237403556,0.158123963796847);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_4,body_5,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident3");
linklist.push_back(link);


// Mate constraint: Concentric3 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_4 , SW name: sqrbp-2 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_2 , SW name: prop_2-1 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.00942816685338319,0.086637844787446,0.152623963796845);
dA = chrono::ChVector3d(-3.49148133884316e-15,6.98296267768625e-15,1);
cB = chrono::ChVector3d(-0.00942816685338318,0.086637844787446,0.150873963796845);
dB = chrono::ChVector3d(0,0,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_4,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric3");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-0.00942816685338319,0.086637844787446,0.152623963796845);
cB = chrono::ChVector3d(-0.00942816685338318,0.086637844787446,0.150873963796845);
dA = chrono::ChVector3d(-3.49148133884316e-15,6.98296267768625e-15,1);
dB = chrono::ChVector3d(0,0,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_4,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric3");
linklist.push_back(link);


// Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_4 , SW name: sqrbp-2 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_2 , SW name: prop_2-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-0.00851099108493078,0.0879488705727621,0.158123963796845);
cB = chrono::ChVector3d(-0.00942816685338318,0.086637844787446,0.158123963796845);
dA = chrono::ChVector3d(-1.72292970257482e-17,1.20533813419047e-17,-1);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_4,body_2,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident4");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.00851099108493078,0.0879488705727621,0.158123963796845);
dA = chrono::ChVector3d(-1.72292970257482e-17,1.20533813419047e-17,-1);
cB = chrono::ChVector3d(-0.00942816685338318,0.086637844787446,0.158123963796845);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_4,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident4");
linklist.push_back(link);


// Mate constraint: Concentric4 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_4 , SW name: sqrbp-2 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_3 , SW name: prop_3-1 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.184376362665988,0.0866378447874975,0.152623963796846);
dA = chrono::ChVector3d(-3.49148133884316e-15,6.98296267768625e-15,1);
cB = chrono::ChVector3d(0.184376362665988,0.0866378447874975,0.150873963796845);
dB = chrono::ChVector3d(3.46756589967432e-15,-6.8628558519186e-15,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_4,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric4");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(0.184376362665988,0.0866378447874975,0.152623963796846);
cB = chrono::ChVector3d(0.184376362665988,0.0866378447874975,0.150873963796845);
dA = chrono::ChVector3d(-3.49148133884316e-15,6.98296267768625e-15,1);
dB = chrono::ChVector3d(3.46756589967432e-15,-6.8628558519186e-15,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_4,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric4");
linklist.push_back(link);


// Mate constraint: Coincident5 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_4 , SW name: sqrbp-2 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_3 , SW name: prop_3-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(0.186801489228191,0.0873580955475104,0.158123963796845);
cB = chrono::ChVector3d(0.184376362665988,0.0866378447874975,0.158123963796845);
dA = chrono::ChVector3d(-1.20533813418501e-17,1.72292970257864e-17,-1);
dB = chrono::ChVector3d(-3.46756589967432e-15,6.8628558519186e-15,1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_4,body_3,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident5");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.186801489228191,0.0873580955475104,0.158123963796845);
dA = chrono::ChVector3d(-1.20533813418501e-17,1.72292970257864e-17,-1);
cB = chrono::ChVector3d(0.184376362665988,0.0866378447874975,0.158123963796845);
dB = chrono::ChVector3d(-3.46756589967432e-15,6.8628558519186e-15,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_4,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident5");
linklist.push_back(link);


// Mate constraint: Concentric5 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_4 , SW name: sqrbp-2 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_1 , SW name: prop_4-1 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.00942816685338441,0.308068237403485,0.152623963796843);
dA = chrono::ChVector3d(-3.49148133884316e-15,6.98296267768625e-15,1);
cB = chrono::ChVector3d(-0.00942816685338454,0.308068237403483,0.150873963796847);
dB = chrono::ChVector3d(0,0,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_4,body_1,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric5");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-0.00942816685338441,0.308068237403485,0.152623963796843);
cB = chrono::ChVector3d(-0.00942816685338454,0.308068237403483,0.150873963796847);
dA = chrono::ChVector3d(-3.49148133884316e-15,6.98296267768625e-15,1);
dB = chrono::ChVector3d(0,0,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_4,body_1,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric5");
linklist.push_back(link);


// Mate constraint: Coincident6 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_4 , SW name: sqrbp-2 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_1 , SW name: prop_4-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-0.00968893672835089,0.30731193120074,0.158123963796845);
cB = chrono::ChVector3d(-0.00942816685338454,0.308068237403483,0.158123963796847);
dA = chrono::ChVector3d(-1.20533813418502e-17,1.72292970257864e-17,-1);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_4,body_1,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident6");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.00968893672835089,0.30731193120074,0.158123963796845);
dA = chrono::ChVector3d(-1.20533813418502e-17,1.72292970257864e-17,-1);
cB = chrono::ChVector3d(-0.00942816685338454,0.308068237403483,0.158123963796847);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_4,body_1,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident6");
linklist.push_back(link);


// Auxiliary marker (coordinate system feature)
auto marker_0_1 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_1->SetName("actuator_1");
body_0->AddMarker(marker_0_1);
marker_0_1->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(0.184376362665965,0.308068237403556,0.152623963796844),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_2 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_2->SetName("actuator_2");
body_0->AddMarker(marker_0_2);
marker_0_2->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.00942816685338317,0.086637844787446,0.152623963796845),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_3 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_3->SetName("actuator_3");
body_0->AddMarker(marker_0_3);
marker_0_3->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(0.184376362665988,0.0866378447874975,0.152623963796846),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_4 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_4->SetName("actuator_4");
body_0->AddMarker(marker_0_4);
marker_0_4->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.00942816685338455,0.308068237403483,0.152623963796843),chrono::ChQuaternion<>(1,0,0,0)));


} // end function
