# Mesh Readme

## Running

I'm submitting my QT setup using QT Creator 4.11.0. You should be able to open
the project and run with proper args set up in the QT Creator run settings.

## Implementation / Pics

I implemented isotropic remeshing as an additional mesh operation.

### Subdivide

`[project_root]\mesh_output\icosahedron_input.obj C:\Users\jeffr\Desktop\CSCI2240\Mesh-[project_root]\icosahedron_subdivide.obj subdivide 4`:

![Image of Yaktocat](./mesh_output/icosahedron_subdivide.png)

### Simplify

`[project_root]\mesh_output\moomoo.obj [project_root]\mesh_output\moomoo_simplify.obj simplify 2500`:

![Image of Yaktocat](./mesh_output/moomoo_simplify.png)

### Remesh

`[project_root]\mesh_output\peter.obj [project_root]\mesh_output\peter_remesh.obj remesh 3 1.0`:

![Image of Yaktocat](./mesh_output/peter_remesh.png)

(Middle is 1 iteration, right is 3 iterations)

## Data structures

Almost all my code is in meshadjacencylist.cpp/.h. Sorry that it's really long.
My implementation of an adjacency list has the following data:

 - Vertex, which is initialized by a position, a set of smart pointers to the
 faces that it lies on, and its ID.
 - Face, which is initialized by an integer vector referring to vertex indices.
 - Edge, which isn't actually tracked in the adjacency list. It's mostly there
 for ordering purposes when I use a set as a BST in simplification. You can see
 that it's ordered by cost.

The MeshAdjacenyList itself has as members:
 - \_verts, an unordered_map from vertex ID numbers to Vertex objects. All
 Vertex pointers in the project point to a value in this map.
 - \_faces, a set of smart pointers to face objects.

MeshAdjacenyList also contains the methods and helpers for edgeFlip, edgeSplit,
and edgeCollapse, as well as subdivide, simplify, and remesh.

One really important function is MeshAdjacenyList::getVertex(int), which takes
in an ID and returns a pointer to that vertex in the list. This allows constant
time accessing of adjacent faces and vertices from a vertex with a know ID.
