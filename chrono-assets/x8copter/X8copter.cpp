// C++ Chrono::Engine model automatically generated using Chrono::SolidWorks add-in

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
#include "X8copter.h"


/// Function to import Solidworks assembly directly into Chrono ChSystem.
void ImportSolidworksSystemCpp(
  chrono::ChSystem& system,
  std::unordered_map<std::string, std::shared_ptr<chrono::ChFunction>>* motfun_map)
{
  std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> bodylist;
  std::vector<std::shared_ptr<chrono::ChLinkBase>> linklist;
  ImportSolidworksSystemCpp(bodylist, linklist, motfun_map);
  for (auto& body : bodylist)
    system.Add(body);
  for (auto& link : linklist)
    system.Add(link);
}


/// Function to import Solidworks bodies and mates into dedicated containers.
void ImportSolidworksSystemCpp(
  std::vector<std::shared_ptr<chrono::ChBodyAuxRef>>& bodylist,
  std::vector<std::shared_ptr<chrono::ChLinkBase>>& linklist,
  std::unordered_map<std::string,
  std::shared_ptr<chrono::ChFunction>>* motfun_map)
{

  // Some global settings
  double sphereswept_r = 0.001;
  chrono::ChCollisionModel::SetDefaultSuggestedEnvelope(0.003);
  chrono::ChCollisionModel::SetDefaultSuggestedMargin(0.003);
  chrono::ChCollisionSystemBullet::SetContactBreakingThreshold(0.002);

  std::string shapes_dir = "../chrono-assets/x8copter/assets/X8copter_shapes";

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
  body_1->SetName("Combine1-1");
  body_1->SetPos(chrono::ChVector3d(0.0371890593246787,-0.328255914279035,-0.00409371708189708));
  body_1->SetRot(chrono::ChQuaternion<>(1,0,0,0));
  body_1->SetMass(12.8583783293344);
  body_1->SetInertiaXX(chrono::ChVector3d(0.246042580268511,0.207285984125684,0.21961669646275));
  body_1->SetInertiaXY(chrono::ChVector3d(2.91302386743778e-05,-0.00235840241095761,0.0011780291169613));
  body_1->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-0.0338805823922144,-0.000461155790933507,0.0883050254534988),chrono::ChQuaternion<>(1,0,0,0)));

  // Visualization shape
  body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
  body_shape->SetFilename(shapes_dir + "body_1_1.obj");
  body_1->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

  bodylist.push_back(body_1);



  // Rigid body part
  auto body_2 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
  body_2->SetName("3_blade_prop-3-solid1-1");
  body_2->SetPos(chrono::ChVector3d(-0.218467842556604,-0.139784773508918,-0.00409371708189528));
  body_2->SetRot(chrono::ChQuaternion<>(-0.109711708248359,3.49699540176307e-15,2.03706924786554e-15,0.993963450572116));
  body_2->SetMass(0.00402346188136127);
  body_2->SetInertiaXX(chrono::ChVector3d(1.56978619628154e-06,1.56998634466397e-06,3.11347365635612e-06));
  body_2->SetInertiaXY(chrono::ChVector3d(5.38976587365754e-11,-7.50116610465268e-13,2.44431697741329e-12));
  body_2->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-0.117426910402035,0.108345058674287,0.0813900042238387),chrono::ChQuaternion<>(1,0,0,0)));

  // Visualization shape
  body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
  body_shape->SetFilename(shapes_dir + "body_2_1.obj");
  body_2->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

  bodylist.push_back(body_2);



  // Rigid body part
  auto body_3 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
  body_3->SetName("3_blade_prop-5-solid1-1");
  body_3->SetPos(chrono::ChVector3d(-0.0737342696756951,-0.0602693344784887,-0.00409371708189701));
  body_3->SetRot(chrono::ChQuaternion<>(0.419740426215534,6.2512186617931e-15,4.2106669583064e-15,0.907644189426893));
  body_3->SetMass(0.00402328325805272);
  body_3->SetInertiaXX(chrono::ChVector3d(1.56978800995868e-06,1.56981587958553e-06,3.11330611692511e-06));
  body_3->SetInertiaXY(chrono::ChVector3d(-8.38274349261104e-11,1.06750881128227e-12,6.46050276289916e-13));
  body_3->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-0.117427162658465,0.108344732611071,0.0241100368042769),chrono::ChQuaternion<>(1,0,0,0)));

  // Visualization shape
  body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
  body_shape->SetFilename(shapes_dir + "body_3_1.obj");
  body_3->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

  bodylist.push_back(body_3);



  // Rigid body part
  auto body_4 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
  body_4->SetName("3_blade_prop-6-solid1-1");
  body_4->SetPos(chrono::ChVector3d(0.174164353040732,-0.314547761477182,-0.00409371708190509));
  body_4->SetRot(chrono::ChQuaternion<>(0.829547966297853,-2.57119971660034e-16,4.09826156080216e-15,0.558435467723081));
  body_4->SetMass(0.00402345293701421);
  body_4->SetInertiaXX(chrono::ChVector3d(1.56995407895125e-06,1.56980599023786e-06,3.11346123424313e-06));
  body_4->SetInertiaXY(chrono::ChVector3d(7.98094621110866e-11,2.75954187812825e-12,-1.05233138430339e-12));
  body_4->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(0.058306027521321,0.108591567855248,0.0241099748285845),chrono::ChQuaternion<>(1,0,0,0)));

  // Visualization shape
  body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
  body_shape->SetFilename(shapes_dir + "body_4_1.obj");
  body_4->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

  bodylist.push_back(body_4);



  // Rigid body part
  auto body_5 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
  body_5->SetName("3_blade_prop-4-solid1-1");
  body_5->SetPos(chrono::ChVector3d(0.172634345749969,-0.12353304862032,-0.00409371708190524));
  body_5->SetRot(chrono::ChQuaternion<>(0.0916071082234274,3.0180900429845e-15,2.69624933062376e-15,0.995795228811095));
  body_5->SetMass(0.00402343266532123);
  body_5->SetInertiaXX(chrono::ChVector3d(1.56973536857199e-06,1.56998906167492e-06,3.11342569982005e-06));
  body_5->SetInertiaXY(chrono::ChVector3d(-4.63415331088178e-11,-1.26495657462373e-12,-3.59163167296094e-13));
  body_5->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(0.0583060612939889,0.108591272967805,0.0813899921167598),chrono::ChQuaternion<>(1,0,0,0)));

  // Visualization shape
  body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
  body_shape->SetFilename(shapes_dir + "body_5_1.obj");
  body_5->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

  bodylist.push_back(body_5);



  // Rigid body part
  auto body_6 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
  body_6->SetName("3_blade_prop-7-solid1-1");
  body_6->SetPos(chrono::ChVector3d(-0.000494567163137741,-0.359887411266736,-0.00409371708189639));
  body_6->SetRot(chrono::ChQuaternion<>(0.979890511258246,-1.38443351002022e-15,3.64986733662816e-15,0.199535926454492));
  body_6->SetMass(0.00402343459612549);
  body_6->SetInertiaXX(chrono::ChVector3d(1.56977011443042e-06,1.56995679651252e-06,3.11342816883423e-06));
  body_6->SetInertiaXY(chrono::ChVector3d(9.90120523983286e-11,1.03776295625586e-12,-4.01579459746519e-13));
  body_6->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(0.0588269483869425,-0.108345256131597,0.0241099907916463),chrono::ChQuaternion<>(1,0,0,0)));

  // Visualization shape
  body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
  body_shape->SetFilename(shapes_dir + "body_6_1.obj");
  body_6->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

  bodylist.push_back(body_6);



  // Rigid body part
  auto body_7 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
  body_7->SetName("3_blade_prop-8-solid1-1");
  body_7->SetPos(chrono::ChVector3d(0.0790441026604105,-0.452786604223404,-0.00409371708189703));
  body_7->SetRot(chrono::ChQuaternion<>(0.911326095865913,-2.86273640869704e-15,-1.29932513858465e-15,-0.411685252339445));
  body_7->SetMass(0.00402332710748505);
  body_7->SetInertiaXX(chrono::ChVector3d(1.56976612197756e-06,1.56975267243666e-06,3.11321992122578e-06));
  body_7->SetInertiaXY(chrono::ChVector3d(-3.19596269449819e-12,-2.41620590549228e-13,-1.06208213746015e-13));
  body_7->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-0.116906352418646,-0.108591080205471,0.0241099989445268),chrono::ChQuaternion<>(1,0,0,0)));

  // Visualization shape
  body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
  body_shape->SetFilename(shapes_dir + "body_7_1.obj");
  body_7->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

  bodylist.push_back(body_7);



  // Rigid body part
  auto body_8 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
  body_8->SetName("3_blade_prop-2-solid1-1");
  body_8->SetPos(chrono::ChVector3d(0.150383124705048,-0.325950753500095,-0.00409371708189769));
  body_8->SetRot(chrono::ChQuaternion<>(0.888348898048883,-3.31959856591705e-15,2.01264039718115e-15,-0.459169070534302));
  body_8->SetMass(0.00402346300513412);
  body_8->SetInertiaXX(chrono::ChVector3d(1.56992717541637e-06,1.56984699092301e-06,3.11347527263434e-06));
  body_8->SetInertiaXY(chrono::ChVector3d(-1.05488607253123e-10,-1.09497053143355e-12,2.32494369655977e-12));
  body_8->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(0.058826974864954,-0.108344973454927,0.0813899776885088),chrono::ChQuaternion<>(1,0,0,0)));

  // Visualization shape
  body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
  body_shape->SetFilename(shapes_dir + "body_8_1.obj");
  body_8->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

  bodylist.push_back(body_8);



  // Rigid body part
  auto body_9 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
  body_9->SetName("box_big_200x200x100-1-solid1-1");
  body_9->SetPos(chrono::ChVector3d(0.0371890593246787,-0.328255914279036,-0.00409371708189819));
  body_9->SetRot(chrono::ChQuaternion<>(1,0,0,0));
  body_9->SetMass(0.349308000000001);
  body_9->SetInertiaXX(chrono::ChVector3d(0.0022603719960028,0.00226037199600279,0.00376709264800002));
  body_9->SetInertiaXY(chrono::ChVector3d(-1.05048426581146e-18,1.04620089376833e-17,1.54326919600813e-18));
  body_9->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-0.0292999999999985,-8.10581988149352e-16,0.243240863342385),chrono::ChQuaternion<>(1,0,0,0)));

  // Visualization shape
  body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
  body_shape->SetFilename(shapes_dir + "body_9_1.obj");
  body_9->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

  bodylist.push_back(body_9);



  // Rigid body part
  auto body_10 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
  body_10->SetName("3_blade_prop-1-solid1-1");
  body_10->SetPos(chrono::ChVector3d(0.0718947090992488,-0.486577031195915,-0.004093717081892));
  body_10->SetRot(chrono::ChQuaternion<>(0.86141567744605,-3.26341720842149e-15,2.12501391112959e-15,-0.507900610996051));
  body_10->SetMass(0.00402345930601188);
  body_10->SetInertiaXX(chrono::ChVector3d(1.56994580377923e-06,1.56982342302243e-06,3.11347035310835e-06));
  body_10->SetInertiaXY(chrono::ChVector3d(-9.23509723024473e-11,-1.15107134381449e-12,2.52757352501226e-12));
  body_10->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-0.11690595291172,-0.108591448813475,0.0813899775028758),chrono::ChQuaternion<>(1,0,0,0)));

  // Visualization shape
  body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
  body_shape->SetFilename(shapes_dir + "body_10_1.obj");
  body_10->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

  bodylist.push_back(body_10);




  // Mate constraint: Coincident2 [MateCoincident] type:0 align:1 flip:False
  //   Entity 0: C::E name: body_1 , SW name: Combine1-1 ,  SW ref.type:2 (2)
  //   Entity 1: C::E name: body_9 , SW name: box_big_200x200x100-1-solid1-1 ,  SW ref.type:2 (2)
  link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
  cA = chrono::ChVector3d(0.0928890593246806,-0.148255914279039,0.273470328094284);
  cB = chrono::ChVector3d(0.00788905932468064,-0.328255914279037,0.273470328094284);
  dA = chrono::ChVector3d(-7.13778253276993e-15,-3.49148133835958e-15,-1);
  dB = chrono::ChVector3d(6.98639662349993e-15,-2.77555756156147e-17,1);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_9,false,cA,cB,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident2");
  linklist.push_back(link);

  link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
  cA = chrono::ChVector3d(0.0928890593246806,-0.148255914279039,0.273470328094284);
  dA = chrono::ChVector3d(-7.13778253276993e-15,-3.49148133835958e-15,-1);
  cB = chrono::ChVector3d(0.00788905932468064,-0.328255914279037,0.273470328094284);
  dB = chrono::ChVector3d(6.98639662349993e-15,-2.77555756156147e-17,1);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_9,false,cA,cB,dA,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident2");
  linklist.push_back(link);


  // Mate constraint: Concentric1 [MateConcentric] type:1 align:0 flip:False
  //   Entity 0: C::E name: body_1 , SW name: Combine1-1 ,  SW ref.type:1 (1)
  //   Entity 1: C::E name: body_6 , SW name: 3_blade_prop-7-solid1-1 ,  SW ref.type:1 (1)
  link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
  cA = chrono::ChVector3d(0.0960159754817679,-0.436600934772515,0.0121562829181011);
  dA = chrono::ChVector3d(6.60045229495362e-15,4.1697458407737e-15,1);
  cB = chrono::ChVector3d(0.096015975481768,-0.436600934772515,0.0241562829181011);
  dB = chrono::ChVector3d(6.60045229495363e-15,4.1697458407737e-15,1);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_6,false,cA,cB,dA,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric1");
  linklist.push_back(link);

  link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
  std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
  cA = chrono::ChVector3d(0.0960159754817679,-0.436600934772515,0.0121562829181011);
  cB = chrono::ChVector3d(0.096015975481768,-0.436600934772515,0.0241562829181011);
  dA = chrono::ChVector3d(6.60045229495362e-15,4.1697458407737e-15,1);
  dB = chrono::ChVector3d(6.60045229495363e-15,4.1697458407737e-15,1);
  std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_1,body_6,false,cA,cB,dA,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric1");
  linklist.push_back(link);


  // Mate constraint: Coincident6 [MateCoincident] type:0 align:1 flip:False
  //   Entity 0: C::E name: body_1 , SW name: Combine1-1 ,  SW ref.type:2 (2)
  //   Entity 1: C::E name: body_6 , SW name: 3_blade_prop-7-solid1-1 ,  SW ref.type:2 (2)
  link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
  cA = chrono::ChVector3d(0.0937596684175709,-0.437677539821758,0.0241562829181011);
  cB = chrono::ChVector3d(0.096015975481768,-0.436600934772515,0.0241562829181011);
  dA = chrono::ChVector3d(-6.60045229495362e-15,-4.1697458407737e-15,-1);
  dB = chrono::ChVector3d(6.60045229495363e-15,4.1697458407737e-15,1);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_6,false,cA,cB,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident6");
  linklist.push_back(link);

  link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
  cA = chrono::ChVector3d(0.0937596684175709,-0.437677539821758,0.0241562829181011);
  dA = chrono::ChVector3d(-6.60045229495362e-15,-4.1697458407737e-15,-1);
  cB = chrono::ChVector3d(0.096015975481768,-0.436600934772515,0.0241562829181011);
  dB = chrono::ChVector3d(6.60045229495363e-15,4.1697458407737e-15,1);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_6,false,cA,cB,dA,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident6");
  linklist.push_back(link);


  // Mate constraint: Concentric2 [MateConcentric] type:1 align:0 flip:False
  //   Entity 0: C::E name: body_1 , SW name: Combine1-1 ,  SW ref.type:1 (1)
  //   Entity 1: C::E name: body_7 , SW name: 3_blade_prop-8-solid1-1 ,  SW ref.type:1 (1)
  link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
  cA = chrono::ChVector3d(-0.0797169542599101,-0.436847418639606,0.0121562829181057);
  dA = chrono::ChVector3d(6.95370162624163e-15,3.1561129071352e-15,1);
  cB = chrono::ChVector3d(-0.0797169542599101,-0.436847418639606,0.0241562829181057);
  dB = chrono::ChVector3d(6.9537016262527e-15,3.15611290713255e-15,1);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_7,false,cA,cB,dA,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric2");
  linklist.push_back(link);

  link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
  std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
  cA = chrono::ChVector3d(-0.0797169542599101,-0.436847418639606,0.0121562829181057);
  cB = chrono::ChVector3d(-0.0797169542599101,-0.436847418639606,0.0241562829181057);
  dA = chrono::ChVector3d(6.95370162624163e-15,3.1561129071352e-15,1);
  dB = chrono::ChVector3d(6.9537016262527e-15,3.15611290713255e-15,1);
  std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_1,body_7,false,cA,cB,dA,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric2");
  linklist.push_back(link);


  // Mate constraint: Coincident8 [MateCoincident] type:0 align:1 flip:False
  //   Entity 0: C::E name: body_1 , SW name: Combine1-1 ,  SW ref.type:2 (2)
  //   Entity 1: C::E name: body_7 , SW name: 3_blade_prop-8-solid1-1 ,  SW ref.type:2 (2)
  link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
  cA = chrono::ChVector3d(-0.0807789309457907,-0.439110647642303,0.0241562829181058);
  cB = chrono::ChVector3d(-0.0797169542599101,-0.436847418639606,0.0241562829181057);
  dA = chrono::ChVector3d(-6.95370162624163e-15,-3.1561129071352e-15,-1);
  dB = chrono::ChVector3d(6.9537016262527e-15,3.15611290713255e-15,1);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_7,false,cA,cB,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident8");
  linklist.push_back(link);

  link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
  cA = chrono::ChVector3d(-0.0807789309457907,-0.439110647642303,0.0241562829181058);
  dA = chrono::ChVector3d(-6.95370162624163e-15,-3.1561129071352e-15,-1);
  cB = chrono::ChVector3d(-0.0797169542599101,-0.436847418639606,0.0241562829181057);
  dB = chrono::ChVector3d(6.9537016262527e-15,3.15611290713255e-15,1);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_7,false,cA,cB,dA,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident8");
  linklist.push_back(link);


  // Mate constraint: Concentric3 [MateConcentric] type:1 align:1 flip:False
  //   Entity 0: C::E name: body_1 , SW name: Combine1-1 ,  SW ref.type:1 (1)
  //   Entity 1: C::E name: body_8 , SW name: 3_blade_prop-2-solid1-1 ,  SW ref.type:1 (1)
  link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
  cA = chrono::ChVector3d(0.0960159754817684,-0.436600934772515,0.0851562829181011);
  dA = chrono::ChVector3d(-6.6004522949585e-15,-4.16974584077486e-15,-1);
  cB = chrono::ChVector3d(0.0960159754817682,-0.436600934772515,0.0731562829181011);
  dB = chrono::ChVector3d(6.62436773412735e-15,4.04963901500721e-15,1);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_8,false,cA,cB,dA,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric3");
  linklist.push_back(link);

  link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
  std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
  cA = chrono::ChVector3d(0.0960159754817684,-0.436600934772515,0.0851562829181011);
  cB = chrono::ChVector3d(0.0960159754817682,-0.436600934772515,0.0731562829181011);
  dA = chrono::ChVector3d(-6.6004522949585e-15,-4.16974584077486e-15,-1);
  dB = chrono::ChVector3d(6.62436773412735e-15,4.04963901500721e-15,1);
  std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_1,body_8,false,cA,cB,dA,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric3");
  linklist.push_back(link);


  // Mate constraint: Coincident10 [MateCoincident] type:0 align:1 flip:False
  //   Entity 0: C::E name: body_1 , SW name: Combine1-1 ,  SW ref.type:2 (2)
  //   Entity 1: C::E name: body_8 , SW name: 3_blade_prop-2-solid1-1 ,  SW ref.type:2 (2)
  link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
  cA = chrono::ChVector3d(0.0970925805310108,-0.438857241836712,0.0731562829181011);
  cB = chrono::ChVector3d(0.0960159754817682,-0.436600934772515,0.0731562829181011);
  dA = chrono::ChVector3d(6.6004522949585e-15,4.16974584077486e-15,1);
  dB = chrono::ChVector3d(-6.62436773412735e-15,-4.04963901500721e-15,-1);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_8,false,cA,cB,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident10");
  linklist.push_back(link);

  link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
  cA = chrono::ChVector3d(0.0970925805310108,-0.438857241836712,0.0731562829181011);
  dA = chrono::ChVector3d(6.6004522949585e-15,4.16974584077486e-15,1);
  cB = chrono::ChVector3d(0.0960159754817682,-0.436600934772515,0.0731562829181011);
  dB = chrono::ChVector3d(-6.62436773412735e-15,-4.04963901500721e-15,-1);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_8,false,cA,cB,dA,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident10");
  linklist.push_back(link);


  // Mate constraint: Concentric4 [MateConcentric] type:1 align:1 flip:False
  //   Entity 0: C::E name: body_1 , SW name: Combine1-1 ,  SW ref.type:1 (1)
  //   Entity 1: C::E name: body_10 , SW name: 3_blade_prop-1-solid1-1 ,  SW ref.type:1 (1)
  link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
  cA = chrono::ChVector3d(-0.0797169542599089,-0.4368474186396,0.0851562829181058);
  dA = chrono::ChVector3d(-6.95210834469172e-15,-3.583832588855e-15,-1);
  cB = chrono::ChVector3d(-0.079716954259909,-0.4368474186396,0.0731562829181023);
  dB = chrono::ChVector3d(6.97602378386057e-15,3.46372576308735e-15,1);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_10,false,cA,cB,dA,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric4");
  linklist.push_back(link);

  link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
  std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
  cA = chrono::ChVector3d(-0.0797169542599089,-0.4368474186396,0.0851562829181058);
  cB = chrono::ChVector3d(-0.079716954259909,-0.4368474186396,0.0731562829181023);
  dA = chrono::ChVector3d(-6.95210834469172e-15,-3.583832588855e-15,-1);
  dB = chrono::ChVector3d(6.97602378386057e-15,3.46372576308735e-15,1);
  std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_1,body_10,false,cA,cB,dA,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric4");
  linklist.push_back(link);


  // Mate constraint: Coincident12 [MateCoincident] type:0 align:1 flip:False
  //   Entity 0: C::E name: body_1 , SW name: Combine1-1 ,  SW ref.type:2 (2)
  //   Entity 1: C::E name: body_10 , SW name: 3_blade_prop-1-solid1-1 ,  SW ref.type:2 (2)
  link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
  cA = chrono::ChVector3d(-0.0774537252572126,-0.437909395325481,0.0731562829181058);
  cB = chrono::ChVector3d(-0.079716954259909,-0.4368474186396,0.0731562829181023);
  dA = chrono::ChVector3d(6.95210834469172e-15,3.583832588855e-15,1);
  dB = chrono::ChVector3d(-6.97602378386057e-15,-3.46372576308735e-15,-1);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_10,false,cA,cB,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident12");
  linklist.push_back(link);

  link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
  cA = chrono::ChVector3d(-0.0774537252572126,-0.437909395325481,0.0731562829181058);
  dA = chrono::ChVector3d(6.95210834469172e-15,3.583832588855e-15,1);
  cB = chrono::ChVector3d(-0.079716954259909,-0.4368474186396,0.0731562829181023);
  dB = chrono::ChVector3d(-6.97602378386057e-15,-3.46372576308735e-15,-1);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_10,false,cA,cB,dA,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident12");
  linklist.push_back(link);


  // Mate constraint: Concentric5 [MateConcentric] type:1 align:0 flip:False
  //   Entity 0: C::E name: body_1 , SW name: Combine1-1 ,  SW ref.type:1 (1)
  //   Entity 1: C::E name: body_3 , SW name: 3_blade_prop-5-solid1-1 ,  SW ref.type:1 (1)
  link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
  cA = chrono::ChVector3d(-0.0802378568324105,-0.219910893785556,0.012156282918102);
  dA = chrono::ChVector3d(6.48087509912429e-15,4.93697426161396e-15,1);
  cB = chrono::ChVector3d(-0.0802378568324104,-0.219910893785555,0.024156282918102);
  dB = chrono::ChVector3d(6.88730092101428e-15,4.63911630499735e-15,1);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_3,false,cA,cB,dA,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric5");
  linklist.push_back(link);

  link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
  std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
  cA = chrono::ChVector3d(-0.0802378568324105,-0.219910893785556,0.012156282918102);
  cB = chrono::ChVector3d(-0.0802378568324104,-0.219910893785555,0.024156282918102);
  dA = chrono::ChVector3d(6.48087509912429e-15,4.93697426161396e-15,1);
  dB = chrono::ChVector3d(6.88730092101428e-15,4.63911630499735e-15,1);
  std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_1,body_3,false,cA,cB,dA,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric5");
  linklist.push_back(link);


  // Mate constraint: Coincident14 [MateCoincident] type:0 align:1 flip:False
  //   Entity 0: C::E name: body_1 , SW name: Combine1-1 ,  SW ref.type:2 (2)
  //   Entity 1: C::E name: body_3 , SW name: 3_blade_prop-5-solid1-1 ,  SW ref.type:2 (2)
  link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
  cA = chrono::ChVector3d(-0.0779815497682133,-0.218834288736313,0.024156282918102);
  cB = chrono::ChVector3d(-0.0802378568324104,-0.219910893785555,0.024156282918102);
  dA = chrono::ChVector3d(-6.48087509912429e-15,-4.93697426161396e-15,-1);
  dB = chrono::ChVector3d(6.88730092101428e-15,4.63911630499735e-15,1);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_3,false,cA,cB,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident14");
  linklist.push_back(link);

  link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
  cA = chrono::ChVector3d(-0.0779815497682133,-0.218834288736313,0.024156282918102);
  dA = chrono::ChVector3d(-6.48087509912429e-15,-4.93697426161396e-15,-1);
  cB = chrono::ChVector3d(-0.0802378568324104,-0.219910893785555,0.024156282918102);
  dB = chrono::ChVector3d(6.88730092101428e-15,4.63911630499735e-15,1);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_3,false,cA,cB,dA,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident14");
  linklist.push_back(link);


  // Mate constraint: Concentric6 [MateConcentric] type:1 align:1 flip:False
  //   Entity 0: C::E name: body_1 , SW name: Combine1-1 ,  SW ref.type:1 (1)
  //   Entity 1: C::E name: body_2 , SW name: 3_blade_prop-3-solid1-1 ,  SW ref.type:1 (1)
  link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
  cA = chrono::ChVector3d(-0.0802378568324102,-0.219910893785555,0.085156282918102);
  dA = chrono::ChVector3d(-6.48087509916658e-15,-4.93697426162138e-15,-1);
  cB = chrono::ChVector3d(-0.0802378568324101,-0.219910893785555,0.0731562829181011);
  dB = chrono::ChVector3d(6.50479053833543e-15,4.81686743585372e-15,1);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_2,false,cA,cB,dA,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric6");
  linklist.push_back(link);

  link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
  std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
  cA = chrono::ChVector3d(-0.0802378568324102,-0.219910893785555,0.085156282918102);
  cB = chrono::ChVector3d(-0.0802378568324101,-0.219910893785555,0.0731562829181011);
  dA = chrono::ChVector3d(-6.48087509916658e-15,-4.93697426162138e-15,-1);
  dB = chrono::ChVector3d(6.50479053833543e-15,4.81686743585372e-15,1);
  std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_1,body_2,false,cA,cB,dA,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric6");
  linklist.push_back(link);


  // Mate constraint: Coincident16 [MateCoincident] type:0 align:1 flip:False
  //   Entity 0: C::E name: body_1 , SW name: Combine1-1 ,  SW ref.type:2 (2)
  //   Entity 1: C::E name: body_2 , SW name: 3_blade_prop-3-solid1-1 ,  SW ref.type:2 (2)
  link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
  cA = chrono::ChVector3d(-0.0791612517831678,-0.222167200849752,0.073156282918102);
  cB = chrono::ChVector3d(-0.0802378568324101,-0.219910893785555,0.0731562829181011);
  dA = chrono::ChVector3d(6.48087509916658e-15,4.93697426162138e-15,1);
  dB = chrono::ChVector3d(-6.50479053833543e-15,-4.81686743585372e-15,-1);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_2,false,cA,cB,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident16");
  linklist.push_back(link);

  link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
  cA = chrono::ChVector3d(-0.0791612517831678,-0.222167200849752,0.073156282918102);
  dA = chrono::ChVector3d(6.48087509916658e-15,4.93697426162138e-15,1);
  cB = chrono::ChVector3d(-0.0802378568324101,-0.219910893785555,0.0731562829181011);
  dB = chrono::ChVector3d(-6.50479053833543e-15,-4.81686743585372e-15,-1);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_2,false,cA,cB,dA,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident16");
  linklist.push_back(link);


  // Mate constraint: Concentric7 [MateConcentric] type:1 align:0 flip:False
  //   Entity 0: C::E name: body_1 , SW name: Combine1-1 ,  SW ref.type:1 (1)
  //   Entity 1: C::E name: body_4 , SW name: 3_blade_prop-6-solid1-1 ,  SW ref.type:1 (1)
  link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
  cA = chrono::ChVector3d(0.095495072909266,-0.21966440991844,0.0121562829181008);
  dA = chrono::ChVector3d(6.51223926297036e-15,5.00381592228644e-15,1);
  cB = chrono::ChVector3d(0.0954950729092661,-0.21966440991844,0.0241562829181001);
  dB = chrono::ChVector3d(6.51223926297036e-15,5.00381592228644e-15,1);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_4,false,cA,cB,dA,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric7");
  linklist.push_back(link);

  link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
  std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
  cA = chrono::ChVector3d(0.095495072909266,-0.21966440991844,0.0121562829181008);
  cB = chrono::ChVector3d(0.0954950729092661,-0.21966440991844,0.0241562829181001);
  dA = chrono::ChVector3d(6.51223926297036e-15,5.00381592228644e-15,1);
  dB = chrono::ChVector3d(6.51223926297036e-15,5.00381592228644e-15,1);
  std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_1,body_4,false,cA,cB,dA,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric7");
  linklist.push_back(link);


  // Mate constraint: Coincident18 [MateCoincident] type:0 align:1 flip:False
  //   Entity 0: C::E name: body_1 , SW name: Combine1-1 ,  SW ref.type:2 (2)
  //   Entity 1: C::E name: body_4 , SW name: 3_blade_prop-6-solid1-1 ,  SW ref.type:2 (2)
  link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
  cA = chrono::ChVector3d(0.0944330962233855,-0.221927638921136,0.0241562829181008);
  cB = chrono::ChVector3d(0.0954950729092661,-0.21966440991844,0.0241562829181001);
  dA = chrono::ChVector3d(-6.51223926297036e-15,-5.00381592228644e-15,-1);
  dB = chrono::ChVector3d(6.51223926297036e-15,5.00381592228644e-15,1);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_4,false,cA,cB,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident18");
  linklist.push_back(link);

  link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
  cA = chrono::ChVector3d(0.0944330962233855,-0.221927638921136,0.0241562829181008);
  dA = chrono::ChVector3d(-6.51223926297036e-15,-5.00381592228644e-15,-1);
  cB = chrono::ChVector3d(0.0954950729092661,-0.21966440991844,0.0241562829181001);
  dB = chrono::ChVector3d(6.51223926297036e-15,5.00381592228644e-15,1);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_4,false,cA,cB,dA,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident18");
  linklist.push_back(link);


  // Mate constraint: Concentric8 [MateConcentric] type:1 align:1 flip:False
  //   Entity 0: C::E name: body_1 , SW name: Combine1-1 ,  SW ref.type:1 (1)
  //   Entity 1: C::E name: body_5 , SW name: 3_blade_prop-4-solid1-1 ,  SW ref.type:1 (1)
  link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
  cA = chrono::ChVector3d(0.0954950729092661,-0.21966440991844,0.0851562829181002);
  dA = chrono::ChVector3d(-6.4808750991393e-15,-4.93697426161674e-15,-1);
  cB = chrono::ChVector3d(0.0954950729092661,-0.21966440991844,0.0731562829181002);
  dB = chrono::ChVector3d(6.50479053830807e-15,4.81686743584904e-15,1);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_5,false,cA,cB,dA,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric8");
  linklist.push_back(link);

  link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
  std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
  cA = chrono::ChVector3d(0.0954950729092661,-0.21966440991844,0.0851562829181002);
  cB = chrono::ChVector3d(0.0954950729092661,-0.21966440991844,0.0731562829181002);
  dA = chrono::ChVector3d(-6.4808750991393e-15,-4.93697426161674e-15,-1);
  dB = chrono::ChVector3d(6.50479053830807e-15,4.81686743584904e-15,1);
  std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_1,body_5,false,cA,cB,dA,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric8");
  linklist.push_back(link);


  // Mate constraint: Coincident20 [MateCoincident] type:0 align:1 flip:False
  //   Entity 0: C::E name: body_1 , SW name: Combine1-1 ,  SW ref.type:2 (2)
  //   Entity 1: C::E name: body_5 , SW name: 3_blade_prop-4-solid1-1 ,  SW ref.type:2 (2)
  link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
  cA = chrono::ChVector3d(0.0977583019119624,-0.22072638660432,0.0731562829181002);
  cB = chrono::ChVector3d(0.0954950729092661,-0.21966440991844,0.0731562829181002);
  dA = chrono::ChVector3d(6.4808750991393e-15,4.93697426161674e-15,1);
  dB = chrono::ChVector3d(-6.50479053830807e-15,-4.81686743584904e-15,-1);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_5,false,cA,cB,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident20");
  linklist.push_back(link);

  link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
  cA = chrono::ChVector3d(0.0977583019119624,-0.22072638660432,0.0731562829181002);
  dA = chrono::ChVector3d(6.4808750991393e-15,4.93697426161674e-15,1);
  cB = chrono::ChVector3d(0.0954950729092661,-0.21966440991844,0.0731562829181002);
  dB = chrono::ChVector3d(-6.50479053830807e-15,-4.81686743584904e-15,-1);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_5,false,cA,cB,dA,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident20");
  linklist.push_back(link);


  // Mate constraint: Coincident21 [MateCoincident] type:0 align:0 flip:False
  //   Entity 0: C::E name: body_0 , SW name: Combine1-1 ,  SW ref.type:4 (4)
  //   Entity 1: C::E name: body_0 , SW name: box_big_200x200x100-1-solid1-1 ,  SW ref.type:4 (4)
  link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
  cA = chrono::ChVector3d(0.0371890593246787,-0.328255914279035,-0.00409371708189708);
  cB = chrono::ChVector3d(0.0371890593246787,-0.328255914279036,-0.00409371708189819);
  dA = chrono::ChVector3d(0,1,0);
  dB = chrono::ChVector3d(0,1,0);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_9,false,cA,cB,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident21");
  linklist.push_back(link);

  link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
  cA = chrono::ChVector3d(0.0371890593246787,-0.328255914279035,-0.00409371708189708);
  dA = chrono::ChVector3d(0,1,0);
  cB = chrono::ChVector3d(0.0371890593246787,-0.328255914279036,-0.00409371708189819);
  dB = chrono::ChVector3d(0,1,0);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_9,false,cA,cB,dA,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident21");
  linklist.push_back(link);


  // Mate constraint: Coincident22 [MateCoincident] type:0 align:0 flip:False
  //   Entity 0: C::E name: body_0 , SW name: Combine1-1 ,  SW ref.type:4 (4)
  //   Entity 1: C::E name: body_0 , SW name: box_big_200x200x100-1-solid1-1 ,  SW ref.type:4 (4)
  link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
  cA = chrono::ChVector3d(0.0371890593246787,-0.328255914279035,-0.00409371708189708);
  cB = chrono::ChVector3d(0.0371890593246787,-0.328255914279036,-0.00409371708189819);
  dA = chrono::ChVector3d(1,0,0);
  dB = chrono::ChVector3d(1,0,0);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_9,false,cA,cB,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
  std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident22");
  linklist.push_back(link);

  link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
  cA = chrono::ChVector3d(0.0371890593246787,-0.328255914279035,-0.00409371708189708);
  dA = chrono::ChVector3d(1,0,0);
  cB = chrono::ChVector3d(0.0371890593246787,-0.328255914279036,-0.00409371708189819);
  dB = chrono::ChVector3d(1,0,0);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_9,false,cA,cB,dA,dB);
  std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident22");
  linklist.push_back(link);


  // Auxiliary marker (coordinate system feature)
  auto marker_0_1 = chrono_types::make_shared<chrono::ChMarker>();
  marker_0_1->SetName("Coordinate System1");
  body_0->AddMarker(marker_0_1);
  marker_0_1->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(0.095495072909266,-0.21966440991844,0.0121562829181008),chrono::ChQuaternion<>(1,0,0,0)));

  // Auxiliary marker (coordinate system feature)
  auto marker_0_2 = chrono_types::make_shared<chrono::ChMarker>();
  marker_0_2->SetName("Coordinate System2");
  body_0->AddMarker(marker_0_2);
  marker_0_2->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.0802378568324105,-0.219910893785556,0.012156282918102),chrono::ChQuaternion<>(1,0,0,0)));

  // Auxiliary marker (coordinate system feature)
  auto marker_0_3 = chrono_types::make_shared<chrono::ChMarker>();
  marker_0_3->SetName("Coordinate System3");
  body_0->AddMarker(marker_0_3);
  marker_0_3->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.0797169542599101,-0.436847418639606,0.0121562829181057),chrono::ChQuaternion<>(1,0,0,0)));

  // Auxiliary marker (coordinate system feature)
  auto marker_0_4 = chrono_types::make_shared<chrono::ChMarker>();
  marker_0_4->SetName("Coordinate System4");
  body_0->AddMarker(marker_0_4);
  marker_0_4->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(0.0960159754817679,-0.436600934772515,0.0121562829181011),chrono::ChQuaternion<>(1,0,0,0)));

  // Auxiliary marker (coordinate system feature)
  auto marker_0_5 = chrono_types::make_shared<chrono::ChMarker>();
  marker_0_5->SetName("Coordinate System5");
  body_0->AddMarker(marker_0_5);
  marker_0_5->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.0802378568324102,-0.219910893785555,0.085156282918102),chrono::ChQuaternion<>(1,0,0,0)));

  // Auxiliary marker (coordinate system feature)
  auto marker_0_6 = chrono_types::make_shared<chrono::ChMarker>();
  marker_0_6->SetName("Coordinate System6");
  body_0->AddMarker(marker_0_6);
  marker_0_6->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(0.0954950729092661,-0.21966440991844,0.0851562829181002),chrono::ChQuaternion<>(1,0,0,0)));

  // Auxiliary marker (coordinate system feature)
  auto marker_0_7 = chrono_types::make_shared<chrono::ChMarker>();
  marker_0_7->SetName("Coordinate System7");
  body_0->AddMarker(marker_0_7);
  marker_0_7->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.0797169542599089,-0.4368474186396,0.0851562829181058),chrono::ChQuaternion<>(1,0,0,0)));

  // Auxiliary marker (coordinate system feature)
  auto marker_0_8 = chrono_types::make_shared<chrono::ChMarker>();
  marker_0_8->SetName("Coordinate System8");
  body_0->AddMarker(marker_0_8);
  marker_0_8->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(0.0960159754817684,-0.436600934772515,0.0851562829181011),chrono::ChQuaternion<>(1,0,0,0)));


} // end function
