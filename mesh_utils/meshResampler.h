#pragma once
#ifndef __MESH_RESAMPLER__
#define __MESH_RESAMPLER__
#include "halfedgeMesh.h"
#include <unordered_set>
#include <unordered_map>

namespace MeshUtils {
	// Used to resample a mesh
	// Subdivision, mesh simplification blah blah ...
	class meshResampler {
	public:
		void LoopSubdivision(Halfedge_mesh& mesh);
		// TODO: 
		// Isotropic remeshing
		void IsotropicRemeshing(Halfedge_mesh& mesh);

		// Linear subdivision
		// Divides each n-sided polygon into n-quads
		// Uses a flag to decide between linear subdivision and Catmull-Clark
		// By default it does a linear subdivision i.e. the original vertex positions remain as they are
		void QuadSubdivision(Halfedge_mesh& mesh, bool linear = true);

		// Catmull-clark subdivision

		// Mesh simplification
	};
}

#endif // !__MESH_RESAMPLER__
