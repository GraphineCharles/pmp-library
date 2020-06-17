// Copyright 2011-2020 the Polygon Mesh Processing Library developers.
// Distributed under a MIT-style license, see LICENSE.txt for details.

#include "pmp/visualization/MeshViewer.h"

#include <iostream>
#include <limits>
#include <sstream>

#include <imgui.h>

namespace pmp {

MeshViewer::MeshViewer(const char* title, int width, int height, bool showgui)
    : TrackballViewer(title, width, height, showgui), currentMesh(0)
{
    // setup draw modes
    clear_draw_modes();
    add_draw_mode("Points");
    add_draw_mode("Hidden Line");
    add_draw_mode("Smooth Shading");
    add_draw_mode("Texture");
	//add_draw_mode("Bake");
	add_draw_mode("Texture Layout");
    set_draw_mode("Smooth Shading");

    crease_angle_ = 180.0;

    // add help items
    add_help_item("Backspace", "Reload mesh", 3);
#ifndef __EMSCRIPTEN__
    add_help_item("W", "Write mesh to 'output.off'", 4);
#endif
}

MeshViewer::~MeshViewer() = default;

bool MeshViewer::load_mesh(const char* filename)
{
    // load mesh
    if (meshes_[0].read(filename))
    {
        // update scene center and bounds
        BoundingBox bb = meshes_[0].bounds();
        set_scene((vec3)bb.center(), 0.5 * bb.size());

        // compute face & vertex normals, update face indices
        update_mesh();

        // set draw mode
        if (meshes_[0].n_faces() == 0)
        {
            set_draw_mode("Points");
        }

        // print mesh statistic
        std::cout << "Load " << filename << ": " << meshes_[0].n_vertices()
                  << " vertices, " << meshes_[0].n_faces() << " faces\n";

        filename_ = filename;
		meshes_[0].set_crease_angle(crease_angle_);
        return true;
    }

    std::cerr << "Failed to read mesh from " << filename << " !" << std::endl;
    return false;
}

bool MeshViewer::load_matcap(const char* filename)
{
    if (!meshes_[0].load_matcap(filename))
        return false;
    set_draw_mode("Texture");
    return true;
}

bool MeshViewer::load_texture(const char* filename, GLint format,
                              GLint min_filter, GLint mag_filter, GLint wrap)
{
    // load texture from file
    if (!meshes_[0].load_texture(filename, format, min_filter, mag_filter, wrap))
        return false;

    set_draw_mode("Texture");

    // set material
	meshes_[0].set_ambient(1.0);
	meshes_[0].set_diffuse(0.9);
	meshes_[0].set_specular(0.0);
	meshes_[0].set_shininess(1.0);

    return true;
}

void MeshViewer::update_mesh()
{
    // update scene center and radius, but don't update camera view
    BoundingBox bb = meshes_[0].bounds();
    center_ = (vec3)bb.center();
    radius_ = 0.5f * bb.size();

    // re-compute face and vertex normals
	for (SurfaceMeshGL &mesh : meshes_)
	{
		mesh.update_opengl_buffers();
	}
}

void MeshViewer::process_imgui()
{
    if (ImGui::CollapsingHeader("Mesh Info", ImGuiTreeNodeFlags_DefaultOpen))
    {
        // output mesh statistics
        ImGui::BulletText("%d vertices", (int)meshes_[0].n_vertices());
        ImGui::BulletText("%d edges", (int)meshes_[0].n_edges());
        ImGui::BulletText("%d faces", (int)meshes_[0].n_faces());

        // control crease angle
        ImGui::PushItemWidth(100);
        ImGui::SliderFloat("Crease Angle", &crease_angle_, 0.0f, 180.0f,
                           "%.0f");
        ImGui::PopItemWidth();
        if (crease_angle_ != meshes_[0].crease_angle())
        {
			meshes_[0].set_crease_angle(crease_angle_);
        }
    }
}

void MeshViewer::draw(const std::string& drawMode)
{
    // draw mesh
	meshes_[currentMesh].draw(projection_matrix_, modelview_matrix_, drawMode);
}

void MeshViewer::keyboard(int key, int scancode, int action, int mods)
{
    if (action != GLFW_PRESS && action != GLFW_REPEAT)
        return;

    switch (key)
    {
        case GLFW_KEY_BACKSPACE: // reload model
        {
            load_mesh(filename_.c_str());
            break;
        }

        case GLFW_KEY_W: // write mesh
        {
			meshes_[currentMesh].write("output.off");
            break;
        }

		case GLFW_KEY_LEFT:
		{
			currentMesh = (currentMesh - 1);
			if (currentMesh < 0)
			{
				currentMesh += meshes_.size();
			}
			break;
		}

		case GLFW_KEY_RIGHT:
		{
			currentMesh = (currentMesh + 1);
			if (currentMesh >= meshes_.size())
			{
				currentMesh -= meshes_.size();
			}
			break;
		}

        default:
        {
            TrackballViewer::keyboard(key, scancode, action, mods);
            break;
        }
    }
}

Vertex MeshViewer::pick_vertex(int x, int y)
{
    Vertex vmin;

    vec3 p;
    Scalar d, dmin(std::numeric_limits<Scalar>::max());

    if (TrackballViewer::pick(x, y, p))
    {
        Point picked_position(p);
        for (auto v : meshes_[currentMesh].vertices())
        {
            d = distance(meshes_[currentMesh].position(v), picked_position);
            if (d < dmin)
            {
                dmin = d;
                vmin = v;
            }
        }
    }
    return vmin;
}

} // namespace pmp
