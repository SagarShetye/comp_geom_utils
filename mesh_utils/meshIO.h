#pragma once

#ifndef __MESH_IO__
#define __MESH_IO__

#include "halfedgeMesh.h"
#include "parse_stl.h"		// https://github.com/dillonhuff/stl_parser

#include <CGAL/IO/io.h>
//#include <CGAL/IO/STL_reader.h>		// CGALs STL reader (only for ASCII .stl files)

#include <fstream>

using CGAL_double = CGAL::Simple_cartesian<double>;
using Point_3 = CGAL_double::Point_3;
using Vector_3 = CGAL::Vector_3<CGAL_double>;
using Ray_3 = CGAL_double::Ray_3;
using vertex_descriptor = CGAL::Surface_mesh<Point_3>::Vertex_index;

namespace MeshUtils {

	// Routines to read a mesh from .stl, .off files and load them into
	// the 'Halfedge_mesh'structure
	// TODO: Write mesh to .stl, .off files
	class meshIO {
	public:

		// Reads an input .off file and gets the Surface_mesh
		// By default, uses own reader, use the flag to use the CGAL reader
		// Uses the CGAL-provided reader that directly populates the Surface_mesh
		static void ReadOff(Halfedge_mesh& mesh, std::string offFilePath, bool useCGALReader = false);

#if 0
		// Write the halfedge mesh to an .off file
		static void WriteOff(Halfedge_mesh& mesh, std::string offFilePath);
#endif

		// Read a mesh from a .ply file
		static void ReadPly(Halfedge_mesh& mesh, std::string plyFilePath);

		// Write a mesh to a .ply file
		static void WritePly(Halfedge_mesh& mesh, std::string plyFilePath);

		// TODO: Work out the STL reader properly
		// Reads an input STL file and populates the halfedge DS
		static void ReadSTL(Halfedge_mesh& mesh, std::string stlFilePath);

	};
}

#endif // !__MESH_IO__
