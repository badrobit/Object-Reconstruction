/*
 * OcclusionRepair.h
 *
 *  Created on: Aug 17, 2013
 *      Author: badrobot
 */

#ifndef OCCLUSIONREPAIR_H_
#define OCCLUSIONREPAIR_H_

#include "HelperFunctions.h"

#include <vtkSTLWriter.h>
#include <vtkSmartPointer.h>
#include <vtkFillHolesFilter.h>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkSmoothPolyDataFilter.h>

class OcclusionRepair
{
public:
  OcclusionRepair();
  virtual ~OcclusionRepair();
  bool DetectOcclusion( PCLMesh input_mesh, std::string output_dir );
};

#endif /* OCCLUSIONREPAIR_H_ */
