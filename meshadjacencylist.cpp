#include "meshadjacencylist.h"
#include <memory>
#include <iostream>
#include <map>

///////////////////////////////////////////////////////////////////////////////
/// \brief Vertex::Vertex
/// \param position
/// \param faces
/// \param id
///

Vertex::Vertex(Vector3f position, set<shared_ptr<Face>> faces, int id)
{
    _position = position;
    _next_position = position;
    _faces = faces;
    _id = id;
}

Vector3f Vertex::getPosition()
{
    return _position;
}

void Vertex::setPosition(Vector3f pos)
{
    _position = pos;
}

void Vertex::setNextPosition(Vector3f pos)
{
    _next_position = pos;
}

void Vertex::updatePosition()
{
    _position = _next_position;
}

set<int> Vertex::adjacentVerts() const
{
    set<int> av = set<int>();
    for(shared_ptr<Face> f : _faces) {
        for (unsigned int i = 0; i < 3; i++) {
            if (f.get()->getVerts()[i] != _id)
                av.insert(f.get()->getVerts()[i]);
        }
    }
    return av;
}

set<shared_ptr<Face>> Vertex::getFaces()
{
    return _faces;
}

int Vertex::getId()
{
    return _id;
}

void Vertex::addFace(shared_ptr<Face> face)
{
    _faces.insert(face);
}

void Vertex::removeFace(shared_ptr<Face> face)
{
    _faces.erase(face);
}

bool operator==(Vertex &lhs, Vertex &rhs)
{
    return lhs._position == rhs._position;
}

/////////////////////////////////////////////////////////////////////////////
/// \brief Face::Face
/// \param verts
///

Face::Face(Vector3i verts)
{
    _verts = verts;
}

Vector3i Face::getVerts()
{
    return _verts;
}


void Face::setVerts(Vector3i newVerts)
{
    _verts = newVerts;
}

bool operator==(Face &lhs, Face &rhs)
{
    return lhs._verts == rhs._verts;
}

/////////////////////////////////////////////////////////////////////////////


Edge::Edge(Vertex* v1, Vertex* v2, float initialCost)
{
    if (v1->getId() < v2->getId()) {
        _v1 = v1;
        _v2 = v2;
    } else {
        _v1 = v2;
        _v2 = v1;
    }
    _cost = initialCost;
}

void Edge::setCost(float cost) {
    _cost = cost;
}

float Edge::getCost() {
    return _cost;
}

Vertex* Edge::getV1() {
    return _v1;
}

Vertex* Edge::getV2() {
    return _v2;
}

bool operator<(const Edge &lhs, const Edge &rhs){
    if (lhs._cost < rhs._cost)
      return true;
    if (rhs._cost < lhs._cost)
      return false;

    // a1==b1: continue with element 2
    if (lhs._v1->getId() < rhs._v1->getId())
      return true;
    if (rhs._v1->getId() < lhs._v1->getId())
      return false;

    // a2 == b2: continue with element 3
    if (lhs._v2 < rhs._v2)
      return true;
    return false;
}

/////////////////////////////////////////////////////////////////////////////
/// \brief MeshAdjacencyList::MeshAdjacencyList
/// \param verts
/// \param faces
///

static bool getSharedVerts(shared_ptr<Face> face1, shared_ptr<Face> face2, set<int> *sharedVerts)
{
    // It's required that the faces must share exactly one edge.
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (face1.get()->getVerts()[i] == face2.get()->getVerts()[j]) {
                sharedVerts->insert(face1.get()->getVerts()[i]);
            }
        }
    }
    if (sharedVerts->size() != 2) {
        printf("Number of shared verts between faces wasn't equal to 2.\n");
        return false;
    }
    return true;
}

/**
 * @brief getAllVertsOnFaces For two faces which share an edge, returns the vertex indices,
 * where c and b are shared, a belongs only to face 1, and d belongs only to face2.
 */
static vector<int> getAllVertsOnFaces(shared_ptr<Face> face1, shared_ptr<Face> face2, set<int> sharedVerts) {
    int a, b, c, d;
    for (int i = 0; i < 3; i++) {
        int face1Vert = face1.get()->getVerts()[i];
        // This is guaranteed to run only once in the for loop.
        if (sharedVerts.count(face1Vert) == 0) {
            a = face1Vert;
            b = face1.get()->getVerts()[(i + 1) % 3];
            c = face1.get()->getVerts()[(i + 2) % 3];
        }

        int face2Vert = face2.get()->getVerts()[i];
        // So is this.
        if (sharedVerts.count(face2Vert) == 0) {
            d = face2Vert;
        }
    }

    vector<int> arr{a, b, c, d};
    return arr;
}

MeshAdjacencyList::MeshAdjacencyList(vector<Vector3f> verts, vector<Vector3i> faces)
{
    // maps vertex IDs (different from the source OBJ indices) to Vertex objects.
    _verts = unordered_map<int, Vertex>();
    // set of pointers to the faces. vertices have such pointers as a parameter.
    _faces = set<shared_ptr<Face>>();

    _unusedVertIds = set<int>();

    _nextVertId = 0;

    build(verts, faces);
}

MeshAdjacencyList::MeshAdjacencyList(const MeshAdjacencyList &rhs)
{
    _verts = rhs._verts;
    _faces = rhs._faces;
    _unusedVertIds = rhs._unusedVertIds;
    _nextVertId = rhs._nextVertId;
}

Vertex* MeshAdjacencyList::getVertex(int id)
{
    return &_verts.at(id);
}

Vector3f MeshAdjacencyList::faceNormal(shared_ptr<Face> face)
{
    Vector3f e1 = getVertex(face.get()->getVerts().y())->getPosition()
                    - getVertex(face.get()->getVerts().x())->getPosition();
    Vector3f e2 = getVertex(face.get()->getVerts().z())->getPosition()
                    - getVertex(face.get()->getVerts().y())->getPosition();
    return -e1.cross(e2).normalized();
}

float MeshAdjacencyList::faceArea(shared_ptr<Face> face)
{
    Vector3f v1 = getVertex(face.get()->getVerts()[0])->getPosition();
    Vector3f v2 = getVertex(face.get()->getVerts()[1])->getPosition();
    Vector3f v3 = getVertex(face.get()->getVerts()[2])->getPosition();
    return 0.5 * ((v2 - v1).cross(v3 - v1)).norm();
}

pair<shared_ptr<Face>, shared_ptr<Face>> MeshAdjacencyList::getFacesAlongEdge(Vertex v1, Vertex v2)
{
    if (v1.adjacentVerts().count(v2.getId()) == 0) {
        perror("No faces along vertices which aren't connected by an edge.\n");
        return pair<shared_ptr<Face>, shared_ptr<Face>>(nullptr, nullptr);
    }

    pair<shared_ptr<Face>, shared_ptr<Face>> faces = pair<shared_ptr<Face>, shared_ptr<Face>>();
    set<shared_ptr<Face>> intersect;

    set<shared_ptr<Face>> v1Faces = v1.getFaces();
    set<shared_ptr<Face>> v2Faces = v2.getFaces();
    set_intersection(v1Faces.begin(), v1Faces.end(), v2Faces.begin(), v2Faces.end(),
                        inserter(intersect, intersect.begin()));
    if (intersect.size() != 2) {
        perror("Number of faces along edge wasn't 2.\n");
        cout << intersect.size() << endl;
        return pair<shared_ptr<Face>, shared_ptr<Face>>(nullptr, nullptr);
    }

    set<shared_ptr<Face>>::iterator iter = intersect.begin();
    faces.first = *iter;
    ++iter;
    faces.second = *iter;
    return faces;
}

set<pair<int, int>> MeshAdjacencyList::getEdges()
{
    set<set<int>> edges = set<set<int>>();
    for (auto v : _verts) {
        for (int v1 : v.second.adjacentVerts()) {
            set<int> edge = set<int>();
            edge.insert(v.second.getId());
            edge.insert(v1);
            edges.insert(edge);
        }
    }
    set<pair<int, int>> edgePairs = set<pair<int, int>>();
    for (set<int> edge : edges) {
        pair<int, int> edgePair = pair<int, int>();
        set<int>::iterator iter = edge.begin();
        edgePair.first = *iter;
        ++iter;
        edgePair.second = *iter;
        edgePairs.insert(edgePair);
    }
    return edgePairs;
}

set<pair<Vertex*, Vertex*>> MeshAdjacencyList::getEdgePointers()
{
    set<pair<Vertex*, Vertex*>> pointers = set<pair<Vertex*, Vertex*>>();
    for (pair<int, int> e : getEdges()) {
        pointers.insert({getVertex(e.first), getVertex(e.second)});
    }
    return pointers;
}

/**
 * @brief MeshAdjacencyList::runTest
 */
void MeshAdjacencyList::runTest()
{
    Vertex* v1 = getVertex(0);
    Vertex* v2 = getVertex(1);
    Vertex* v3 = getVertex(10);
    Vertex* v4 = getVertex(11);
    pair<shared_ptr<Face>, shared_ptr<Face>> faces = getFacesAlongEdge(*v1, *v2);
    edgeCollapse(v1, v2);
    edgeCollapse(v3, v4);
}

void MeshAdjacencyList::build(vector<Vector3f> verts, vector<Vector3i> faces)
{
    set<int> vertsAdded = set<int>();
    for (unsigned int i = 0; i < faces.size(); i++) {
        shared_ptr<Face> face(new Face(faces.at(i)));
        _faces.insert(face);

        Vector3i vertsAtFace = faces.at(i);

        if (vertsAtFace.x() >= (int) verts.size() ||
                vertsAtFace.y() >= (int) verts.size() ||
                vertsAtFace.z() >= (int) verts.size()) {
            perror("Face vertex index was out of range!");
        }

        Vector3f v1 = verts[vertsAtFace.x()];
        Vector3f v2 = verts[vertsAtFace.y()];
        Vector3f v3 = verts[vertsAtFace.z()];

        // True if v1 has already been added to _verts.
        if (vertsAdded.count(vertsAtFace.x()) != 0) {
            _verts.at(vertsAtFace.x()).addFace(face);
        } else {
            set<shared_ptr<Face>> initFaces{face};
            _verts.insert({vertsAtFace.x(), Vertex(v1, initFaces, vertsAtFace.x())});
            vertsAdded.insert(vertsAtFace.x());
        }

        if (vertsAdded.count(vertsAtFace.y()) != 0) {
            _verts.at(vertsAtFace.y()).addFace(face);
        } else {
            set<shared_ptr<Face>> initFaces{face};
            _verts.insert({vertsAtFace.y(), Vertex(v2, initFaces, vertsAtFace.y())});
            vertsAdded.insert(vertsAtFace.y());
        }

        if (vertsAdded.count(vertsAtFace.z()) != 0) {
            _verts.at(vertsAtFace.z()).addFace(face);
        } else {
            set<shared_ptr<Face>> initFaces{face};
            _verts.insert({vertsAtFace.z(), Vertex(v3, initFaces, vertsAtFace.z())});
            vertsAdded.insert(vertsAtFace.z());
        }
    }
    _nextVertId = verts.size();
}

MeshAdjacencyList MeshAdjacencyList::subdivide()
{
    // Copy
    MeshAdjacencyList newList(*this);
    set<pair<int, int>> edges = newList.getEdges();

    set<Vertex*> oldVertices = set<Vertex*>();

    set<pair<int, int>> edgesToFlip;

    for (pair<int, int> edge : edges) {
        oldVertices.insert(newList.getVertex(edge.first));
        oldVertices.insert(newList.getVertex(edge.second));
    }

    // Figure out new positions for old verts before adding new verts.
    for (Vertex* oldV : oldVertices) {
        set<int> adjacentOlds = oldV->adjacentVerts();
        int n = adjacentOlds.size();
        float u = n == 3 ? 3.0 / 16.0 : 3.0 / (8.0 * n);

        Vector3f sumNeighbors = Vector3f(0, 0, 0);
        for (int adjacent : adjacentOlds) {
            sumNeighbors += newList.getVertex(adjacent)->getPosition();
        }

        oldV->setNextPosition(((1.0 - (n * u)) * oldV->getPosition()) + (u * sumNeighbors));
    }

    // Split edges.
    for (pair<int, int> edge : edges) {
        Vertex* v1 = newList.getVertex(edge.first);
        Vertex* v2 = newList.getVertex(edge.second);

        pair<shared_ptr<Face>, shared_ptr<Face>> faces = newList.getFacesAlongEdge(*v1, *v2);

        Vertex* newMidpoint = newList.edgeSplit(faces.first, faces.second);
        set<int> sharedVerts = {v1->getId(), v2->getId()};

        vector<int> vertsOnFaces = getAllVertsOnFaces(faces.first, faces.second, sharedVerts);
        // Verts a and d are NOT shared between the two faces.
        // I would think I'd need vert A here instead of C but whatever this seems to work :/
        int vertC = vertsOnFaces.at(2);
        int vertD = vertsOnFaces.at(3);

        if (oldVertices.count(newList.getVertex(vertC)) > 0) {
            edgesToFlip.insert(pair<int, int>(vertC, newMidpoint->getId()));
        }
        if (oldVertices.count(newList.getVertex(vertD)) > 0) {
            edgesToFlip.insert(pair<int, int>(newMidpoint->getId(), vertD));
        }

        // Figure out new position for new vert (to be applied later).
        Vector3f AB = v1->getPosition() + v2->getPosition();
        Vector3f CD = newList.getVertex(vertC)->getPosition() + newList.getVertex(vertD)->getPosition();
        newMidpoint->setNextPosition(((3.0 / 8.0) * AB) + ((1.0 / 8.0) * CD));
    }

    // Flip edges to make proper topology.
    for (pair<int, int> edge : edgesToFlip) {
        Vertex* v1 = newList.getVertex(edge.first);
        Vertex* v2 = newList.getVertex(edge.second);

        pair<shared_ptr<Face>, shared_ptr<Face>> faces = newList.getFacesAlongEdge(*v1, *v2);
        newList.edgeFlip(faces.first, faces.second);
    }

    // Update vert positions
    for (pair<int, Vertex> v : _verts) {
        newList.getVertex(v.first)->updatePosition();
    }
    return newList;
}

Matrix4f MeshAdjacencyList::faceQ(shared_ptr<Face> face)
{
    Vector3f Q_normal = faceNormal(face).normalized();
    Vector3f p = getVertex(face.get()->getVerts()[0])->getPosition();
    float a = Q_normal.x();
    float b = Q_normal.y();
    float c = Q_normal.z();
    float d = (-p).dot(Q_normal);

    Matrix4f Q_face;
    Q_face << a * a, a * b, a * c, a * d,
              b * a, b * b, b * c, b * d,
              c * a, c * b, c * c, c * d,
              d * a, d * b, d * c, d * d;
    return Q_face;
}

Matrix4f MeshAdjacencyList::vertQ(unordered_map<shared_ptr<Face>, Matrix4f> faceQ, Vertex vertex)
{
    set<shared_ptr<Face>> adjacentFaces = vertex.getFaces();
    // Calculate Q for vertices using Qs for adjacent faces.
    Matrix4f vQ = Matrix4f::Zero();
    for (shared_ptr<Face> vertFace : adjacentFaces) {
        vQ += faceQ[vertFace];
    }
    return vQ;
}

pair<float, Vector3f> MeshAdjacencyList::edgeMinCost(pair<Vertex *, Vertex *> edge, unordered_map<Vertex*, Matrix4f> vertQs)
{
    Vertex* v1;
    Vertex* v2;
    if(edge.first->getId() < edge.second->getId()){
        v1 = edge.first;
        v2 = edge.second;
    } else {
        v1 = edge.second;
        v2 = edge.first;
    }

    Matrix4f Q_bar = vertQs[v1] + vertQs[v2];

    Matrix4f Q_for_v;
    Q_for_v << Q_bar(0, 0), Q_bar(0, 1), Q_bar(0, 2), Q_bar(0, 3),
               Q_bar(1, 0), Q_bar(1, 1), Q_bar(1, 2), Q_bar(1, 3),
               Q_bar(2, 0), Q_bar(2, 1), Q_bar(2, 2), Q_bar(2, 3),
               0          , 0          , 0          , 1;

    Eigen::FullPivLU<Matrix4f> Q_bar_lu = Q_for_v.fullPivLu();
    if (Q_bar_lu.isInvertible()) {
        // Invertible
        Vector4f v_bar = Q_for_v.inverse() * Vector4f(0, 0, 0, 1);

        Vector3f v_barPos = Vector3f(v_bar.x(), v_bar.y(), v_bar.z());
        float cost = v_bar.transpose() * Q_bar * v_bar;
        return pair<float, Vector3f>(cost, v_barPos);
    } else {
         Vector3f v_barPos = (v1->getPosition() + v2->getPosition()) / 2.0;
         Vector4f v_bar = Vector4f(v_barPos.x(), v_barPos.y(), v_barPos.z(), 1);
         float cost = v_bar.transpose() * Q_bar * v_bar;
         return pair<float, Vector3f>(cost, v_barPos);
    }
}

MeshAdjacencyList MeshAdjacencyList::simplify(int vertsToRemove)
{
    MeshAdjacencyList newList(*this);

    // Maps face pointers to their Q values.
    unordered_map<shared_ptr<Face>, Matrix4f> faceQs = unordered_map<shared_ptr<Face>, Matrix4f>();

    for (shared_ptr<Face> face : newList._faces) {
        // Calculate initial Q for face
        faceQs[face] = newList.faceQ(face);
        count++;
    }

    unordered_map<Vertex*, Matrix4f> vertQs = unordered_map<Vertex*, Matrix4f>();

    int v = 0;
    for (pair<int, Vertex> vert : newList._verts) {
        // Calculate initial Q for every vertex
        vertQs[newList.getVertex(vert.first)] = newList.vertQ(faceQs, vert.second);
        v++;
    }

    // Set of edge-cost pairs, where each edge is a pair of vertices. Parameterized by a comparator that orders based on cost.
    // The edge pairs should be ordered with smallest vertex ID first.
    set<Edge> edges = set<Edge>();
    map<pair<Vertex*, Vertex*>, float> edgeCosts = map<pair<Vertex*, Vertex*>, float>();

    // Initialize cost for every edge.
    for (pair<int, int> edgePair : newList.getEdges()) {
        pair<Vertex*, Vertex*> edgePointers = { newList.getVertex(edgePair.first), newList.getVertex(edgePair.second) };
        float cost = edgeMinCost(edgePointers, vertQs).first;

        Edge edge(newList.getVertex(edgePair.first), newList.getVertex(edgePair.second), cost);

        edges.insert(edge);
        edgeCosts[{edge.getV1(), edge.getV2()}] = cost;
    }

    while (vertsToRemove > 0) {
        set<Edge>::iterator edgeIter = edges.begin();
        Edge lowestCostEdge = *edgeIter;

        Vertex* i = newList.getVertex(lowestCostEdge.getV1()->getId());
        Vertex* j = newList.getVertex(lowestCostEdge.getV2()->getId());

        bool sharedVertDegree3 = false;
        set<int> iAdj = i->adjacentVerts();
        set<int> jAdj = j->adjacentVerts();
        set<int> sharedAdj = set<int>();
        for (int a : iAdj) {
            if (jAdj.count(a) > 0) {
                sharedAdj.insert(a);
            }
        }
        for (int shared : sharedAdj) {
            if (newList.getVertex(shared)->adjacentVerts().size() == 3) {
                sharedVertDegree3 = true;
            }
        }
        if (sharedVertDegree3) {
            continue;
        }


        Vertex* k = newList.edgeCollapse(i, j, false);

        Vector3f newPosition = edgeMinCost({i, j}, vertQs).second;
        k->setPosition(newPosition);

        Matrix4f Qk = vertQs[i] + vertQs[j];
        vertQs[k] = Qk;

        set<int> kAdjacent = k->adjacentVerts();


        for (int idAdj : kAdjacent) {
            Vertex* adjacentVert = newList.getVertex(idAdj);

            pair<Vertex*, Vertex*> adjacentEdge;
            if (adjacentVert->getId() < k->getId()) {
                adjacentEdge = {adjacentVert, k};
            }
            else {
                adjacentEdge = {k, adjacentVert};
            }

            float newEdgeCost = edgeMinCost(adjacentEdge, vertQs).first;

            pair<Vertex*, Vertex*> ai = adjacentVert->getId() < i->getId() ? pair<Vertex*, Vertex*>{adjacentVert, i} : pair<Vertex*, Vertex*>{i, adjacentVert};
            edges.erase(Edge(adjacentVert, i, edgeCosts[ai]));
            pair<Vertex*, Vertex*> aj = adjacentVert->getId() < j->getId() ? pair<Vertex*, Vertex*>{adjacentVert, j} : pair<Vertex*, Vertex*>{j, adjacentVert};
            edges.erase(Edge(adjacentVert, j, edgeCosts[aj]));

            edges.insert(Edge(adjacentEdge.first, adjacentEdge.second, newEdgeCost));
            edgeCosts[adjacentEdge] = newEdgeCost;

        }

        // Doing the removeOldVerts stuff deferred from collapse.
        newList._verts.erase(i->getId());
        newList._verts.erase(j->getId());

        vertQs.erase(i);
        vertQs.erase(j);
        edges.erase(lowestCostEdge);
        edgeCosts.erase({lowestCostEdge.getV1(), lowestCostEdge.getV2()});

        vertsToRemove--;
    }

    return newList;
}

MeshAdjacencyList MeshAdjacencyList::remesh(int steps, float smoothingWeight)
{
    MeshAdjacencyList newList(*this);

    for (int step = 0; step < steps; step++) {
        set<pair<int, int>> edges = newList.getEdges();

        float totalEdgeLength = 0;
        for (pair<int, int> e : edges) {
            totalEdgeLength += (newList.getVertex(e.first)->getPosition() - newList.getVertex(e.second)->getPosition()).norm();
        }
        float L = totalEdgeLength / edges.size();

        // Splitting edges.
        for (pair<int, int> e : edges) {
            Vertex* v1 = newList.getVertex(e.first);
            Vertex* v2 = newList.getVertex(e.second);
            float edgeLength = (v1->getPosition() - v1->getPosition()).norm();

            pair<shared_ptr<Face>, shared_ptr<Face>> faces = newList.getFacesAlongEdge(*v1, *v2);

            if (edgeLength > (4.0 / 3.0) * L) {
                newList.edgeSplit(faces.first, faces.second);
            }
        }

        // Collapse edges
        set<Vertex*> collapsed = set<Vertex*>();
        for (pair<Vertex*, Vertex*> e : newList.getEdgePointers()) {
            Vertex* v1 = e.first;
            Vertex* v2 = e.second;

            float edgeLength = (v2->getPosition() - v1->getPosition()).norm();

            if (edgeLength < (4.0 / 5.0) * L) {
                if (collapsed.count(v1) == 0 && collapsed.count(v2) == 0) {
                    bool sharedVertDegree3 = false;
                    set<int> v1Adj = v1->adjacentVerts();
                    set<int> v2Adj = v2->adjacentVerts();
                    set<int> sharedAdj = set<int>();
                    for (int a : v1Adj) {
                        if (v2Adj.count(a) > 0) {
                            sharedAdj.insert(a);
                        }
                    }
                    for (int shared : sharedAdj) {
                        if (newList.getVertex(shared)->adjacentVerts().size() == 3) {
                            sharedVertDegree3 = true;
                        }
                    }

                    if (!sharedVertDegree3) {
                        newList.edgeCollapse(v1, v2);

                        collapsed.insert(v1);
                        collapsed.insert(v2);
                    }
                }
            }
        }

        // Flip edges to reach valence 6
        float averageDegree = 0;
        for (pair<int, Vertex> v : _verts) {
            averageDegree += v.second.adjacentVerts().size();
        }
        averageDegree /= _verts.size();

        for (pair<int, int> edge : newList.getEdges()) {

            Vertex* v1 = newList.getVertex(edge.first);
            int v1Valence = v1->adjacentVerts().size();
            Vertex* v2 = newList.getVertex(edge.second);
            int v2Valence = v2->adjacentVerts().size();

            if (v1Valence == 3 || v2Valence == 3) {
                continue;
            }

            pair<shared_ptr<Face>, shared_ptr<Face>> facesAlongEdge = newList.getFacesAlongEdge(*v1, *v2);
            if (facesAlongEdge.first == nullptr) {
                continue;
            }
            vector<int> verts = getAllVertsOnFaces(facesAlongEdge.first, facesAlongEdge.second, {v1->getId(), v2->getId()});

            Vertex* a = newList.getVertex(verts.at(0));
            int aValence = a->adjacentVerts().size();
            Vertex* d = newList.getVertex(verts.at(3));
            int dValence = d->adjacentVerts().size();

            assert(a->getId() != v1->getId() && a->getId() != v2->getId());
            assert(d->getId() != v1->getId() && d->getId() != v2->getId());

            int preDistTo6 = abs(v1Valence - 6) + abs(v2Valence - 6) + abs(aValence - 6) + abs(dValence - 6);
            int postDistTo6 = abs(v1Valence - 1 - 6) + abs(v2Valence - 1 - 6) + abs(aValence + 1 - 6) + abs(dValence + 1 - 6);

            if (preDistTo6 > postDistTo6) {
                newList.edgeFlip(facesAlongEdge.first, facesAlongEdge.second);
            }
        }

        // Tangential smoothing.
        for (pair<int, Vertex> vert : newList._verts) {
            Vector3f centroid = Vector3f::Zero();

            int numAdj = vert.second.adjacentVerts().size();
            for (int adj : vert.second.adjacentVerts()) {
                centroid += newList.getVertex(adj)->getPosition();
            }
            centroid /= 1.0 * numAdj;

            Vector3f v = centroid - vert.second.getPosition();

            Vector3f n = Vector3f::Zero();
            float totalArea = 0;
            for (shared_ptr<Face> f : vert.second.getFaces()) {
                float fArea = newList.faceArea(f);
                n += newList.faceNormal(f) * fArea;
                totalArea += fArea;
            }
            n /= totalArea * vert.second.getFaces().size();

            v = v - (n.dot(v) * n);

            Vertex* pointer = newList.getVertex(vert.first);
            pointer->setNextPosition(pointer->getPosition() + (smoothingWeight * v));
        }
        for (pair<int, Vertex> vert : newList._verts) {
            newList.getVertex(vert.first)->updatePosition();
        }
    }

    return newList;

}

bool MeshAdjacencyList::edgeFlip(shared_ptr<Face> face1, shared_ptr<Face> face2)
{
    set<int> sharedVerts = set<int>();
    if (!getSharedVerts(face1, face2, &sharedVerts)) {
        return false;
    }

    vector<int> allVerts = getAllVertsOnFaces(face1, face2, sharedVerts);

    int a, b, c, d;
    a = allVerts.at(0);
    b = allVerts.at(1);
    c = allVerts.at(2);
    d = allVerts.at(3);

    face1->setVerts(Vector3i(a, d, c));
    face2->setVerts(Vector3i(a, b, d));

    // Adding both is redundant but it's safe because set uniqueness
    _verts.at(a).addFace(face1);
    _verts.at(a).addFace(face2);
    _verts.at(d).addFace(face1);
    _verts.at(d).addFace(face2);
    _verts.at(b).removeFace(face1);
    _verts.at(c).removeFace(face2);

    return true;
}

Vertex *MeshAdjacencyList::edgeSplit(shared_ptr<Face> face1, shared_ptr<Face> face2)
{
    set<int> sharedVerts = set<int>();
    if (!getSharedVerts(face1, face2, &sharedVerts)) {
        return nullptr;
    }

    set<int>::iterator iter = sharedVerts.begin();
    Vertex* sharedVertC = getVertex(*iter);
    ++iter;
    Vertex* sharedVertB = getVertex(*iter);
    Vector3f midVector = (sharedVertC->getPosition() + sharedVertB->getPosition()) * 0.5;

    int mId = getNextVertId();
    Vertex midpointM = Vertex(midVector, set<shared_ptr<Face>>(), mId);

    vector<int> allVerts = getAllVertsOnFaces(face1, face2, sharedVerts);

    int a, b, c, d;
    a = allVerts.at(0);
    b = allVerts.at(1);
    c = allVerts.at(2);
    d = allVerts.at(3);

    face1->setVerts(Vector3i(a, mId, c));
    face2->setVerts(Vector3i(c, mId, d));

    shared_ptr<Face> newFace1(new Face(Vector3i(a, b, mId)));
    _verts.at(a).addFace(newFace1);
    _verts.at(b).addFace(newFace1);
    shared_ptr<Face> newFace2(new Face(Vector3i(d, mId, b)));
    _verts.at(d).addFace(newFace2);
    _verts.at(b).addFace(newFace2);

    _verts.at(b).removeFace(face1);
    _verts.at(b).removeFace(face2);

    _verts.insert({mId, midpointM});
    _faces.insert(newFace1);
    _faces.insert(newFace2);

    _verts.at(mId).addFace(face1);
    _verts.at(mId).addFace(face2);
    _verts.at(mId).addFace(newFace1);
    _verts.at(mId).addFace(newFace2);

    return getVertex(mId);
}

Vertex *MeshAdjacencyList::edgeCollapse(Vertex *c, Vertex *d, bool removeOldVerts)
{
    set<shared_ptr<Face>> sharedFaces = set<shared_ptr<Face>>();
    set<shared_ptr<Face>> adjacentFaces = set<shared_ptr<Face>>();
    for (shared_ptr<Face> f : c->getFaces()) {
        if (d->getFaces().count(f) != 0) {
            sharedFaces.insert(f);
        } else {
            adjacentFaces.insert(f);
        }
    }
    for (shared_ptr<Face> f : d->getFaces()) {
        adjacentFaces.insert(f);
    }

    if (sharedFaces.size() != 2) {
        perror("Vertices for edge collapse don't share exactly two faces.");
        return nullptr;
    }

    Vector3f midVector = (c->getPosition() + d->getPosition()) * 0.5;
    int mId = getNextVertId();
    Vertex midpointM = Vertex(midVector, set<shared_ptr<Face>>(), mId);

    int vertIdC = c->getId();
    int vertIdD = d->getId();

    _unusedVertIds.insert(vertIdC);
    _unusedVertIds.insert(vertIdD);
    if (removeOldVerts) {
        _verts.erase(vertIdC);
        _verts.erase(vertIdD);
    }

    _verts.insert({mId, midpointM});

    for(shared_ptr<Face> face : adjacentFaces) {
        if (sharedFaces.count(face) != 0) {
            for(int i = 0; i < 3; i++) {
                if (face->getVerts()[i] != vertIdC && face->getVerts()[i] != vertIdD)
                    getVertex(face->getVerts()[i])->removeFace(face);
            }
            _faces.erase(face);
        } else {
            Vector3i faceVerts = face->getVerts();
            int newX, newY, newZ;
            if (faceVerts.x() == vertIdC || faceVerts.x() == vertIdD) {
                newX = mId;
                newY = faceVerts.y();
                newZ = faceVerts.z();
            }
            else if (faceVerts.y() == vertIdC || faceVerts.y() == vertIdD) {
                newX = faceVerts.x();
                newY = mId;
                newZ = faceVerts.z();
            }
            else if (faceVerts.z() == vertIdC || faceVerts.z() == vertIdD) {
                newX = faceVerts.x();
                newY = faceVerts.y();
                newZ = mId;
            } else {
                perror("This shouldn't happen :(");
                newX = newY = newZ = 0;
            }
            face->setVerts(Vector3i(newX, newY, newZ));
            _verts.at(mId).addFace(face);
        }
    }

    return getVertex(mId);
}

int MeshAdjacencyList::numVertices()
{
    return _verts.size();
}

int MeshAdjacencyList::numFaces()
{
    return _faces.size();
}

void MeshAdjacencyList::buildFacesVerts(vector<Vector3f> *vertices, vector<Vector3i> *faces)
{
    vertices->clear();
    faces->clear();
    // Maps vertex ids from the adjacency list to their position (linear index) in the output mesh list
    unordered_map<int, int> reorderedVerts = unordered_map<int, int>();

    int newVertIndex = 0;
    for (pair<int, Vertex> vert : _verts) {
        vertices->push_back(vert.second.getPosition());
        reorderedVerts.insert({vert.first, newVertIndex});
        newVertIndex++;
    }


    for (auto face : _faces) {
        int v1 = reorderedVerts.at(face.get()->getVerts()[0]);
        int v2 = reorderedVerts.at(face.get()->getVerts()[1]);
        int v3 = reorderedVerts.at(face.get()->getVerts()[2]);
        faces->push_back(Vector3i(v1, v2, v3));
    }
}

/**
 * @brief MeshAdjacencyList::getNextVertId Note the value returned here must be used.
 * @return The vertex ID.
 */
int MeshAdjacencyList::getNextVertId()
{
    if (_unusedVertIds.empty()) {
        _nextVertId += 1;
        return _nextVertId - 1;
    } else {
        set<int>::iterator elem = _unusedVertIds.begin();
        int val = *elem;
        _unusedVertIds.erase(elem);
        return val;
    }
}
