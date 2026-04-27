// C++ Chrono::Engine model automatically generated using Chrono::SolidWorks add-in
// Assembly: D:\Dropbox\Projects\QRBP\SolidWorks_Files\TAILSITTER\TAILSITTER_V1.0\chrono_assembly.SLDASM


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
#include "tailsitter.h"


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

std::string shapes_dir = "tailsitter_shapes/";

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
body_1->SetName("16x5.4-4");
body_1->SetPos(chrono::ChVector3d(-0.195536532075182,0.622423734499918,0.389590456554933));
body_1->SetRot(chrono::ChQuaternion<>(0.709933147284872,-0.704269072433396,1.37055504022756e-17,1.38932008928742e-17));
body_1->SetMass(0.02311480613903);
body_1->SetInertiaXX(chrono::ChVector3d(0.000243481959493161,2.02448155763798e-06,0.000245240519319778));
body_1->SetInertiaXY(chrono::ChVector3d(-1.27122136125483e-05,1.01829982031449e-07,1.94838378671284e-06));
body_1->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-3.39798186774076e-09,-0.00151682657378196,1.12688645520965e-09),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_1->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_1);



// Rigid body part
auto body_2 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_2->SetName("full_frame_single_part_w_col_zones_for_chorno-1");
body_2->SetPos(chrono::ChVector3d(0.0626960034167829,0.0398098665855967,0.291715918886598));
body_2->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_2->SetMass(1.54005578358517);
body_2->SetInertiaXX(chrono::ChVector3d(0.144934841382282,0.0467202293377085,0.178211716360287));
body_2->SetInertiaXY(chrono::ChVector3d(1.50847693993492e-06,7.33299121677796e-06,-0.00105481465615566));
body_2->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-0.0352349286341472,0.322872417873955,0.186003857246375),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_2_1.obj");
body_2->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// Collision Model
body_2->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

// Collision material
auto mat_2 = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

// Collision shape
std::shared_ptr<chrono::ChCollisionShape> collshape_2;
mr(0,0)=1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=0; mr(1,1)=-0.00801014722760637; mr(2,1)=0.999967918256077;
mr(0,2)=0; mr(1,2)=-0.999967918256077; mr(2,2)=-0.00801014722760637;
collshape_2 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_2,0.3655,0.0509962779529972,0.928);
body_2->GetCollisionModel()->AddShape(collshape_2,chrono::ChFramed(chrono::ChVector3d(-0.0354825354921314,0.32198645845824,0.299999171348472), mr));
mr(0,0)=-2.19672588431727E-16; mr(1,0)=0.00801014722760712; mr(2,0)=-0.999967918256077;
mr(0,1)=1; mr(1,1)=0; mr(2,1)=0;
mr(0,2)=0; mr(1,2)=-0.999967918256077; mr(2,2)=-0.00801014722760712;
collshape_2 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_2,0.252699490762865,0.06,1.5);
body_2->GetCollisionModel()->AddShape(collshape_2,chrono::ChFramed(chrono::ChVector3d(-0.258232535492131,0.322598081802313,0.223645553275023), mr));
std::vector<chrono::ChVector3d> pt_vect_body_2;
pt_vect_body_2.push_back(chrono::ChVector3d(0.0646743414384874,0.601841448593895,0.194381649536858));
pt_vect_body_2.push_back(chrono::ChVector3d(-0.135262807604944,0.602458229930421,0.117384119831141));
pt_vect_body_2.push_back(chrono::ChVector3d(-0.135262807604944,0.601841448593895,0.194381649536858));
pt_vect_body_2.push_back(chrono::ChVector3d(0.0646743414384874,0.602458229930421,0.117384119831141));
pt_vect_body_2.push_back(chrono::ChVector3d(0.0641718443387435,0.0444761315435296,0.112914457678136));
pt_vect_body_2.push_back(chrono::ChVector3d(-0.135765304704688,0.0444761315435302,0.112914457678136));
pt_vect_body_2.push_back(chrono::ChVector3d(-0.135765304704688,0.0438593502070044,0.189911987383853));
pt_vect_body_2.push_back(chrono::ChVector3d(0.0641718443387435,0.0438593502070038,0.189911987383853));
collshape_2 = chrono_types::make_shared<chrono::ChCollisionShapeConvexHull>(mat_2,pt_vect_body_2);
body_2->GetCollisionModel()->AddShape(collshape_2);
mr(0,0)=-1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=2.19672588431726E-16; mr(1,1)=-0.00801014722760665; mr(2,1)=0.999967918256077;
mr(0,2)=0; mr(1,2)=0.999967918256077; mr(2,2)=0.00801014722760665;
collshape_2 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_2,0.06,0.252699490762866,1.5);
body_2->GetCollisionModel()->AddShape(collshape_2,chrono::ChFramed(chrono::ChVector3d(0.187267464507868,0.322598081802313,0.223645553275024), mr));
body_2->EnableCollision(true);

bodylist.push_back(body_2);



// Rigid body part
auto body_3 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_3->SetName("16x5.4-1");
body_3->SetPos(chrono::ChVector3d(0.249963467924819,0.622423734499919,0.389590456554933));
body_3->SetRot(chrono::ChQuaternion<>(0.709933147284872,-0.704269072433396,1.37055504022756e-17,1.38932008928742e-17));
body_3->SetMass(0.02311480613903);
body_3->SetInertiaXX(chrono::ChVector3d(0.000243481959493161,2.02448155763798e-06,0.000245240519319778));
body_3->SetInertiaXY(chrono::ChVector3d(-1.27122136125483e-05,1.01829982031449e-07,1.94838378671284e-06));
body_3->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-3.39798186774076e-09,-0.00151682657378196,1.12688645520965e-09),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_3->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_3);



// Rigid body part
auto body_4 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_4->SetName("16x5.4-3");
body_4->SetPos(chrono::ChVector3d(0.249963467924483,0.104440352842935,0.385441200291029));
body_4->SetRot(chrono::ChQuaternion<>(0.709933147284872,-0.704269072433396,-6.67822392031798e-17,-6.63105858532401e-17));
body_4->SetMass(0.02311480613903);
body_4->SetInertiaXX(chrono::ChVector3d(0.000243481959493161,2.02448155763798e-06,0.000245240519319778));
body_4->SetInertiaXY(chrono::ChVector3d(-1.27122136125483e-05,1.01829982031452e-07,1.9483837867129e-06));
body_4->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-3.39798186774076e-09,-0.00151682657378196,1.12688645520965e-09),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_4->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_4);



// Rigid body part
auto body_5 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_5->SetName("16x5.4-2");
body_5->SetPos(chrono::ChVector3d(-0.195536532075518,0.104440352842934,0.38544120029103));
body_5->SetRot(chrono::ChQuaternion<>(0.709933147284872,-0.704269072433396,1.37055504022756e-17,1.38932008928742e-17));
body_5->SetMass(0.02311480613903);
body_5->SetInertiaXX(chrono::ChVector3d(0.000243481959493161,2.02448155763798e-06,0.000245240519319778));
body_5->SetInertiaXY(chrono::ChVector3d(-1.27122136125483e-05,1.01829982031449e-07,1.94838378671284e-06));
body_5->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-3.39798186774076e-09,-0.00151682657378196,1.12688645520965e-09),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_5->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_5);




// Mate constraint: Concentric1 [MateConcentric] type:1 align:0 flip:False
//   Entity 0: C::E name: body_2 , SW name: full_frame_single_part_w_col_zones_for_chorno-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_3 , SW name: 16x5.4-1 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.249963467924819,0.622411719279077,0.391090408432317);
dA = chrono::ChVector3d(-3.90312782094782e-17,0.00801014722760722,-0.999967918256077);
cB = chrono::ChVector3d(0.249963467924819,0.62243574972076,0.388090504677549);
dB = chrono::ChVector3d(-3.90312782094781e-17,0.00801014722760722,-0.999967918256077);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_2,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric1");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(0.249963467924819,0.622411719279077,0.391090408432317);
cB = chrono::ChVector3d(0.249963467924819,0.62243574972076,0.388090504677549);
dA = chrono::ChVector3d(-3.90312782094782e-17,0.00801014722760722,-0.999967918256077);
dB = chrono::ChVector3d(-3.90312782094781e-17,0.00801014722760722,-0.999967918256077);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_2,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric1");
linklist.push_back(link);


// Mate constraint: Coincident1 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_2 , SW name: full_frame_single_part_w_col_zones_for_chorno-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_3 , SW name: 16x5.4-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(0.249963467924819,0.622411719279077,0.391090408432317);
cB = chrono::ChVector3d(0.249963467924819,0.622411719279077,0.391090408432317);
dA = chrono::ChVector3d(-3.90312782094782e-17,0.00801014722760722,-0.999967918256077);
dB = chrono::ChVector3d(3.90312782094781e-17,-0.00801014722760722,0.999967918256077);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_2,body_3,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident1");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.249963467924819,0.622411719279077,0.391090408432317);
dA = chrono::ChVector3d(-3.90312782094782e-17,0.00801014722760722,-0.999967918256077);
cB = chrono::ChVector3d(0.249963467924819,0.622411719279077,0.391090408432317);
dB = chrono::ChVector3d(3.90312782094781e-17,-0.00801014722760722,0.999967918256077);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_2,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident1");
linklist.push_back(link);


// Mate constraint: Concentric3 [MateConcentric] type:1 align:0 flip:False
//   Entity 0: C::E name: body_2 , SW name: full_frame_single_part_w_col_zones_for_chorno-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_5 , SW name: 16x5.4-2 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.195536532075518,0.104428337622092,0.386941152168414);
dA = chrono::ChVector3d(-3.90312782094782e-17,0.00801014722760722,-0.999967918256077);
cB = chrono::ChVector3d(-0.195536532075518,0.104452368063775,0.383941248413645);
dB = chrono::ChVector3d(-3.90312782094781e-17,0.00801014722760722,-0.999967918256077);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_2,body_5,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric3");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-0.195536532075518,0.104428337622092,0.386941152168414);
cB = chrono::ChVector3d(-0.195536532075518,0.104452368063775,0.383941248413645);
dA = chrono::ChVector3d(-3.90312782094782e-17,0.00801014722760722,-0.999967918256077);
dB = chrono::ChVector3d(-3.90312782094781e-17,0.00801014722760722,-0.999967918256077);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_2,body_5,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric3");
linklist.push_back(link);


// Mate constraint: Coincident2 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_2 , SW name: full_frame_single_part_w_col_zones_for_chorno-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_5 , SW name: 16x5.4-2 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-0.195536532075518,0.104428337622092,0.386941152168414);
cB = chrono::ChVector3d(-0.195536532075518,0.104428337622092,0.386941152168414);
dA = chrono::ChVector3d(-3.90312782094782e-17,0.00801014722760722,-0.999967918256077);
dB = chrono::ChVector3d(3.90312782094781e-17,-0.00801014722760722,0.999967918256077);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_2,body_5,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident2");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.195536532075518,0.104428337622092,0.386941152168414);
dA = chrono::ChVector3d(-3.90312782094782e-17,0.00801014722760722,-0.999967918256077);
cB = chrono::ChVector3d(-0.195536532075518,0.104428337622092,0.386941152168414);
dB = chrono::ChVector3d(3.90312782094781e-17,-0.00801014722760722,0.999967918256077);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_2,body_5,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident2");
linklist.push_back(link);


// Mate constraint: Concentric4 [MateConcentric] type:1 align:0 flip:False
//   Entity 0: C::E name: body_2 , SW name: full_frame_single_part_w_col_zones_for_chorno-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_4 , SW name: 16x5.4-3 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.249963467924483,0.104428337622093,0.386941152168414);
dA = chrono::ChVector3d(1.8821749714349e-16,0.00801014722760746,-0.999967918256077);
cB = chrono::ChVector3d(0.249963467924483,0.104452368063776,0.383941248413645);
dB = chrono::ChVector3d(1.8821749714349e-16,0.00801014722760746,-0.999967918256077);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_2,body_4,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric4");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(0.249963467924483,0.104428337622093,0.386941152168414);
cB = chrono::ChVector3d(0.249963467924483,0.104452368063776,0.383941248413645);
dA = chrono::ChVector3d(1.8821749714349e-16,0.00801014722760746,-0.999967918256077);
dB = chrono::ChVector3d(1.8821749714349e-16,0.00801014722760746,-0.999967918256077);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_2,body_4,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric4");
linklist.push_back(link);


// Mate constraint: Coincident3 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_2 , SW name: full_frame_single_part_w_col_zones_for_chorno-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_4 , SW name: 16x5.4-3 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(0.249963467924483,0.104428337622093,0.386941152168414);
cB = chrono::ChVector3d(0.249963467924483,0.104428337622093,0.386941152168414);
dA = chrono::ChVector3d(1.8821749714349e-16,0.00801014722760746,-0.999967918256077);
dB = chrono::ChVector3d(-1.8821749714349e-16,-0.00801014722760746,0.999967918256077);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_2,body_4,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident3");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.249963467924483,0.104428337622093,0.386941152168414);
dA = chrono::ChVector3d(1.8821749714349e-16,0.00801014722760746,-0.999967918256077);
cB = chrono::ChVector3d(0.249963467924483,0.104428337622093,0.386941152168414);
dB = chrono::ChVector3d(-1.8821749714349e-16,-0.00801014722760746,0.999967918256077);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_2,body_4,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident3");
linklist.push_back(link);


// Mate constraint: Concentric5 [MateConcentric] type:1 align:0 flip:False
//   Entity 0: C::E name: body_2 , SW name: full_frame_single_part_w_col_zones_for_chorno-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_1 , SW name: 16x5.4-4 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.195536532075182,0.622411719279077,0.391090408432317);
dA = chrono::ChVector3d(-3.90312782094782e-17,0.00801014722760722,-0.999967918256077);
cB = chrono::ChVector3d(-0.195536532075182,0.62243574972076,0.388090504677549);
dB = chrono::ChVector3d(-3.90312782094781e-17,0.00801014722760722,-0.999967918256077);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_2,body_1,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric5");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-0.195536532075182,0.622411719279077,0.391090408432317);
cB = chrono::ChVector3d(-0.195536532075182,0.62243574972076,0.388090504677549);
dA = chrono::ChVector3d(-3.90312782094782e-17,0.00801014722760722,-0.999967918256077);
dB = chrono::ChVector3d(-3.90312782094781e-17,0.00801014722760722,-0.999967918256077);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_2,body_1,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric5");
linklist.push_back(link);


// Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_2 , SW name: full_frame_single_part_w_col_zones_for_chorno-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_1 , SW name: 16x5.4-4 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-0.195536532075182,0.622411719279077,0.391090408432317);
cB = chrono::ChVector3d(-0.195536532075182,0.622411719279077,0.391090408432317);
dA = chrono::ChVector3d(-3.90312782094782e-17,0.00801014722760722,-0.999967918256077);
dB = chrono::ChVector3d(3.90312782094781e-17,-0.00801014722760722,0.999967918256077);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_2,body_1,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident4");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.195536532075182,0.622411719279077,0.391090408432317);
dA = chrono::ChVector3d(-3.90312782094782e-17,0.00801014722760722,-0.999967918256077);
cB = chrono::ChVector3d(-0.195536532075182,0.622411719279077,0.391090408432317);
dB = chrono::ChVector3d(3.90312782094781e-17,-0.00801014722760722,0.999967918256077);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_2,body_1,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident4");
linklist.push_back(link);


// Auxiliary marker (coordinate system feature)
auto marker_0_1 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_1->SetName("motor_1");
body_0->AddMarker(marker_0_1);
marker_0_1->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(0.249963467924819,0.622411719279078,0.391090408432317),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_2 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_2->SetName("motor_2");
body_0->AddMarker(marker_0_2);
marker_0_2->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.195536532075518,0.104428337622092,0.386941152168414),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_3 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_3->SetName("motor_3");
body_0->AddMarker(marker_0_3);
marker_0_3->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(0.249963467924819,0.104428337622092,0.386941152168414),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_4 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_4->SetName("motor_4");
body_0->AddMarker(marker_0_4);
marker_0_4->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.195536532075518,0.622411719279078,0.391090408432317),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_5 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_5->SetName("lower_wing_p1");
body_0->AddMarker(marker_0_5);
marker_0_5->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(0.249963469370506,-0.38737054221452,0.484704907166303),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_6 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_6->SetName("lower_wing_p2");
body_0->AddMarker(marker_0_6);
marker_0_6->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(0.249963469370506,1.1125813351696,0.496720128007714),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_7 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_7->SetName("upper_wing_p1");
body_0->AddMarker(marker_0_7);
marker_0_7->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.195536532075349,-0.38737054221452,0.484704907166303),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_8 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_8->SetName("upper_wing_p2");
body_0->AddMarker(marker_0_8);
marker_0_8->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.195536532075349,1.1125813351696,0.496720128007713),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_9 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_9->SetName("right_stab_p1");
body_0->AddMarker(marker_0_9);
marker_0_9->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.183167605142544,0.820861751638429,0.582400787546324),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_10 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_10->SetName("right_stab_p2");
body_0->AddMarker(marker_0_10);
marker_0_10->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(0.237594540987456,0.820861751638429,0.582400787546324),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_11 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_11->SetName("left_stab_p1");
body_0->AddMarker(marker_0_11);
marker_0_11->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.183167605142544,-0.0971087973206494,0.575045854598506),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_12 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_12->SetName("left_stab_p2");
body_0->AddMarker(marker_0_12);
marker_0_12->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(0.237594540987456,-0.0971087973206494,0.575045854598506),chrono::ChQuaternion<>(1,0,0,0)));


} // end function
