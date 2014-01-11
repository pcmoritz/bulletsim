from dolfin import *
import numpy
import os

parameters["form_compiler"]["cpp_optimize"] = True

ffc_options = {"optimize": True, \
               "eliminate_zeros": True, \
               "precompute_basis_const": True, \
               "precompute_ip_const": True}

def solve(mesh_file, marker_file, transformation_file):
    mesh = Mesh(mesh_file)
    mesh_function = MeshFunction("size_t", mesh, 2, mesh_file)

    V = VectorFunctionSpace(mesh, "Lagrange", 1)

    bc0 = DirichletBC(V, u0, lambda x: inner_boundary(x, 'rect'))
    bc1 = DirichletBC(V, u1, lambda x: inner_boundary(x, 'rope'))
    bc2 = DirichletBC(V, u2, lambda x: inner_boundary(x, 'circ'))

if __name__ == "__main__":
    directory = "/home/pcm/bulletsim/build/"
    mesh_file = os.path.join(directory, "output.xml")
    marker_file = os.path.join(directory, "output_facet_region.xml")
