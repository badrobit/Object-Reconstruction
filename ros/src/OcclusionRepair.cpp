/*
 * OcclusionRepair.cpp
 *
 *  Created on: Aug 17, 2013
 *      Author: badrobot
 */

#include "OcclusionRepair.h"

OcclusionRepair::OcclusionRepair()
{
  // TODO Auto-generated constructor stub

}

OcclusionRepair::~OcclusionRepair()
{
  // TODO Auto-generated destructor stub
}

bool
OcclusionRepair::DetectOcclusion( PCLMesh input_mesh, std::string output_dir )
{
  vtkSmartPointer<vtkPolyData> vtk_mesh;
  pcl::VTKUtils::convertToVTK( input_mesh, vtk_mesh );

  //ROS_INFO_STREAM( "Number of polygons before hole filling: "<< vtk_mesh.polygons.size() );

  vtkSmartPointer<vtkFillHolesFilter> fillHolesFilter = vtkSmartPointer<vtkFillHolesFilter>::New();
  //fillHolesFilter->SetInputData( vtk_mesh );

#if VTK_MAJOR_VERSION >= 5
  fillHolesFilter->SetInput( vtk_mesh );
#else
  ROS_ERROR( "VTK Version to low, please upgrade to at least VTK 5.0" );
#endif

  fillHolesFilter->SetHoleSize( 1000.0 );
  fillHolesFilter->Update();

  std::string filename = output_dir + "05-fixed_mesh.stl";
  vtkSmartPointer<vtkSTLWriter> stlWriter = vtkSmartPointer<vtkSTLWriter>::New();
  stlWriter->SetFileName( filename.c_str() );
  stlWriter->SetInput( vtk_mesh );
  stlWriter->Write();

  vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->SetRadius(10);
  sphereSource->SetPhiResolution(50);
  sphereSource->SetThetaResolution(50);
  sphereSource->Update();

  vtkSmartPointer<vtkSmoothPolyDataFilter> smoothFilter = vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
  smoothFilter->SetInputConnection( 0, sphereSource->GetOutputPort() );
  smoothFilter->SetInputConnection( 1, fillHolesFilter->GetOutputPort() );
  smoothFilter->Update();

  filename = output_dir + "06-wrapped_mesh.stl";
  stlWriter->SetFileName( filename.c_str() );
  stlWriter->SetInputConnection(smoothFilter->GetOutputPort());
  stlWriter->Write();

  vtkSmartPointer<vtkSmoothPolyDataFilter> smoothFilter1 = vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
  smoothFilter1->SetInputConnection(smoothFilter->GetOutputPort());
  smoothFilter1->SetNumberOfIterations(5);
  smoothFilter1->Update();

  filename = output_dir + "07-smoothed_mesh.stl";
  stlWriter->SetFileName( filename.c_str() );
  stlWriter->SetInputConnection(smoothFilter1->GetOutputPort());
  stlWriter->Write();

  return false;
}
