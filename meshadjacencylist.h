#ifndef MESHADJACENCYLIST_H
#define MESHADJACENCYLIST_H

#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <unordered_map>
#include <set>
#include <memory>

using namespace std;
using namespace Eigen;

class Face
{
public:
    Face(Vector3i verts);
    Vector3i getVerts();
    void setVerts(Vector3i newVerts);

    friend bool operator== (Face &lhs, Face &rhs);

private:
    Vector3i _verts;

};

class Vertex
{
public:
    Vertex(Vector3f position, set<shared_ptr<Face>> faces, int id);
    void addFace(shared_ptr<Face> face);
    Vector3f getPosition();
    void setPosition(Vector3f pos);
    void setNextPosition(Vector3f pos);
    void updatePosition();
    /**
     * @brief adjacentVerts Gets a set of all vertices which share an edge with this one,
     * represented by their integer IDs (in some adjacency list we don't have access to).
     */
    set<int> adjacentVerts() const;
    set<shared_ptr<Face>> getFaces();
    void removeFace(shared_ptr<Face> face);
    int getId();

    friend bool operator== (Vertex &lhs, Vertex &rhs);

private:
    Vector3f _position;
    Vector3f _next_position;
    set<shared_ptr<Face>> _faces;
    int _id;
};

class Edge
{
public:
    Edge(Vertex* v1, Vertex* v2, float initialCost);
    void setCost(float cost);
    float getCost();
    Vertex* getV1();
    Vertex* getV2();
    friend bool operator== (const Edge &lhs, const Edge &rhs);
    friend bool operator< (const Edge &lhs, const Edge &rhs);

private:
    Vertex* _v1;
    Vertex* _v2;
    float _cost;
};

class MeshAdjacencyList
{
public:
    MeshAdjacencyList(vector<Vector3f> verts, vector<Vector3i> faces);
    MeshAdjacencyList(const MeshAdjacencyList &rhs);
    void buildFacesVerts(vector<Vector3f> *vertices, vector<Vector3i> *faces);

    Vertex* getVertex(int id);
    Vector3f faceNormal(shared_ptr<Face> face);
    float faceArea(shared_ptr<Face> face);
    pair<shared_ptr<Face>, shared_ptr<Face>> getFacesAlongEdge(Vertex v1, Vertex v2);
    set<pair<int, int>> getEdges();
    set<pair<Vertex*, Vertex*>> getEdgePointers();

    MeshAdjacencyList subdivide();
    Matrix4f faceQ(shared_ptr<Face> face);
    Matrix4f vertQ(unordered_map<shared_ptr<Face>, Matrix4f> faceQ, Vertex vertex);
    pair<float, Vector3f> edgeMinCost(pair<Vertex*, Vertex*> edge, unordered_map<Vertex*, Matrix4f> vertQs);
    MeshAdjacencyList simplify(int vertsToRemove);
    MeshAdjacencyList remesh(int steps, float smoothingWeight);

    void runTest();

    int numVertices();
    int numFaces();

private:
    set<int> _unusedVertIds;
    int _nextVertId;
    int getNextVertId();

    unordered_map<int, Vertex> _verts;
    set<shared_ptr<Face>> _faces;

    void build(vector<Vector3f> verts, vector<Vector3i> faces);

    /**
     * @brief edgeCollapse The provided faces must shared exactly one edge (two verts).
     * Following successful execution they will be connected by the two vertices that
     * weren't connected to begin with, and the two vertices that were connected won't
     * be any more.
     */
    bool edgeFlip(shared_ptr<Face> face1, shared_ptr<Face> face2);

    /**
     * @brief edgeSplit The provided faces must shared exactly one edge (two verts).
     * A split will be peformed opposite the edge direction of the edge that already
     * exists, and a midpoint vertex will be added. A pointer to that vertex in the _verts
     * map is returned.
     */
    Vertex *edgeSplit(shared_ptr<Face> face1, shared_ptr<Face> face2);

    /**
     * @brief edgeCollapse The provided edges must share exactly two faces. The provided
     * vertices will be collapsed into one vertex and their shared faces will be removed
     * from the face list. All otherwise adjacent faces will be updated to contain the new
     * vertex, which is at the midpoint of the two inputs. A pointer to that new vertex is
     * returned.
     *
     * removeOldVerts option is helpful for simplify, where doing so causes us to lose
     * useful pointers. If set to false, you gotta remove the verts at some point, otherwise
     * topology problems are going to arise quickly.
     */
    Vertex *edgeCollapse(Vertex* v1, Vertex* v2, bool removeOldVerts = true);

};

#endif // MESHADJACENCYLIST_H
