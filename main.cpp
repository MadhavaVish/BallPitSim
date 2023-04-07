#include <igl/opengl/glfw/Viewer.h>
#include <igl/readOBJ.h>
int main(int argc, char* argv[])
{
	// Inline mesh of a cube
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	igl::readOBJ("../assets/sphere.obj", V, F);
	// Plot the mesh
	igl::opengl::glfw::Viewer viewer;
	viewer.data().set_mesh(V, F);
	viewer.data().set_face_based(true);
	viewer.launch();
}
