// C++ Chrono::Engine model automatically generated using Chrono::SolidWorks add-in

#ifndef CH_IMPORT_SLDW_CPP_QRBP_H
#define CH_IMPORT_SLDW_CPP_QRBP_H

#include <vector>
#include <unordered_map>
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/functions/ChFunction.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChLinkRSDA.h"

// Triangle meshes 
#include "chrono/geometry/ChTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"

/// Function to import Solidworks assembly directly into Chrono ChSystem.
void ImportSolidworksSystemCpp(chrono::ChSystem& system, std::unordered_map<std::string, std::shared_ptr<chrono::ChFunction>>* motfun_map = nullptr);

/// Function to import Solidworks bodies and mates into dedicated containers.
void ImportSolidworksSystemCpp(std::vector<std::shared_ptr<chrono::ChBodyAuxRef>>& bodylist, std::vector<std::shared_ptr<chrono::ChLinkBase>>& linklist, std::unordered_map<std::string, std::shared_ptr<chrono::ChFunction>>* motfun_map = nullptr);

#endif // end CH_IMPORT_SLDW_CPP_QRBP_H
