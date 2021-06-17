
#if 0
		struct edge_t {
			uint32_t fst;
			uint32_t snd;

			uint64_t as64() const {
				uint64_t u = *((uint64_t*)this);
				return u;
			}

			bool isUndirectedSame(const edge_t& e) const {
				return (fst == e.fst && snd == e.snd) || (fst == e.snd && snd == e.fst);
			}

			bool containedIn(vector<edge_t*>& edges, size_t& containedIdx) const {
				for (size_t i = 0; i < edges.size(); i++) {
					auto e = edges[i];
					if (isUndirectedSame(*e)) {
						containedIdx = i;
						return true;
					}
				}
				return false;
			}

			static edge_t From64(uint64_t u) {
				edge_t w = *((edge_t*)(&u));
				return w;
			}
		};
		
		struct typed_edge_t {
			uint32_t fst;
			uint32_t snd;

			bool outer;

			bool visited;
			point2_t direction(const vector<point2_t>& vertices) const {
				auto f = vertices[fst];
				auto s = vertices[snd];
				auto dir = s - f;
				dir = dir / lnpz_linalg::length(dir);
				return dir;
			}

			bool tryGetOpposite(uint32_t vi, uint32_t& res) const noexcept {
				if (vi == fst) {
					res = snd;
					return true;
				}
				if (vi == snd) {
					res = fst;
					return true;
				}
				return false;
			}

			bool isIncomingTo(uint32_t vert) const noexcept{
				return (snd == vert) && (fst != vert);
			}

			bool isOutgoingFrom(uint32_t vert) const noexcept{
				return (snd != vert) && (fst == vert);
			}

			bool isSame(const typed_edge_t& e) const noexcept {
				return fst == e.fst && snd == e.snd;
			}

			bool isUndirectedSame(const typed_edge_t& e) const {
				return (fst == e.fst && snd == e.snd) || (fst == e.snd && snd == e.fst);
			}

			bool containedIn(vector<typed_edge_t*>& edges, size_t& containedIdx) const {
				for (size_t i = 0; i < edges.size(); i++) {
					auto e = edges[i];
					if (isUndirectedSame(*e)) {
						containedIdx = i;
						return true;
					}
				}
				return false;
			}
		};

		struct indexedwire_t {
			vector<uint32_t> indices;

			vector<edge_t> toEdges() const {
				vector<edge_t> edges;
				uint32_t sz = indices.size();
				for (uint32_t i = 0; i < sz; i++) {
					uint32_t j = (i + 1) % sz;
					edges.push_back({ i, j });
				}
				return edges;
			}
		};


		struct indexedpolyface_t {
			vector<indexedwire_t>  outerWires;
			vector<indexedwire_t>  innerWires;
			vector<point2_t> vertices;
		};


		//indexedpolyface_t RemoveDegeneracies(const indexedpolyface_t& input) {
		//}

		// Contain information :
		// edge A->B
		// new vertex C in the middle, split edge to edges A->C, C->B
		// now as information refers to edge A->B by index (a cut)
		// we can refer it to either child. If 
		// Use same vertex clustering parametrization for new cuts!
		// so that if we have several cuts targeting edge A->B, there is the possibility they will create only one new vertex, 
		// or none at all if the new vertex matches either of the existing vertices
		// the edges are collected back from picking only those edges that don't have children

		struct split_tree_t {

			struct split_t {
				static const uint32_t npos = 0;
				edge_t parent;
				uint32_t childFst, childSnd;
				
				lnpz_linalg::segment2<double> toSegment(vector<point2_t>& vertices) const noexcept{
					return { vertices[parent.fst], vertices[parent.snd] };
				}

				void appendChilds(vector<split_t>& splits, edge_t fst, edge_t snd) {
					splits.push_back({ fst,split_t::npos, split_t::npos });
					childFst = splits.size() - 1;
					splits.push_back({ snd,split_t::npos, split_t::npos });
					childSnd = splits.size() - 1;
				}
			};

			clusteringradius_t clustering;

			vector<split_t> edges;
			vector<point2_t>& vertices;
			size_t originalVertexSize; // initial vertex buffer size. Any vertex at index larger than this is a cut vertex

			vector<pair<point2_t, uint32_t>> cutVertices; // position, address in vertex array. Pos = 0 implies not set yet

			uint32_t pushCutVertex(const point2_t& pnt) {
				cutVertices.push_back({pnt, 0});
				return cutVertices.size() - 1;
			}

			const point2_t& cutPoint(uint32_t cutVertIdx) const {
				return cutVertices[cutVertIdx].first;
			}

			uint32_t verifyCutStored(uint32_t cutVertIdx) { 

				if (cutVertices[cutVertIdx].second != 0)
					return cutVertices[cutVertIdx].second;
				
				vertices.push_back(cutVertices[cutVertIdx].first);
				auto vertexBufferIdx = vertices.size() - 1;
				cutVertices[cutVertIdx].second = vertexBufferIdx;

				return vertexBufferIdx;
			}

			//const lnpz_linalg::segment2<double>::intersect_res_t& isect

			// the srcEdgeIdx must be a leaf edge
			void cutLeafEdge(size_t srcEdgeIdx, uint32_t cutVertexIdx) {
				if (edges[srcEdgeIdx].childFst != split_t::npos) {
					throw runtime_error("cutLeafEdge: Algorithm error, edge is not leaf");
				}

				split_t& srcNode = edges[srcEdgeIdx];
				auto segmentAtIdx = srcNode.toSegment(vertices);
				auto cutVertex = cutPoint(cutVertexIdx);
				bool cutAtExistingVertex = clustering.withinRange(segmentAtIdx.fst, cutVertex) || clustering.withinRange(segmentAtIdx.fst, cutVertex);
				if (cutAtExistingVertex)
					return;
				
				uint32_t cutVertexBufferIdx = verifyCutStored(cutVertexIdx);
				
				edge_t childFst = {srcNode.parent.fst, cutVertexBufferIdx};
				edge_t childSnd = {cutVertexBufferIdx, srcNode.parent.snd};

				srcNode.appendChilds(edges, childFst, childSnd);
			}

			void collectLeafs(uint32_t src, vector<uint32_t>& leaf) {
				stack<uint32_t> edgePath;
				edgePath.push(src);
				while (!edgePath.empty()) {
					uint32_t candIdx = edgePath.top();
					if (candIdx == split_t::npos) {
						throw std::runtime_error("collectLeafs: algorithm data is malformed");
					}
					edgePath.pop();
					const split_t& cand = edges[candIdx];

					if (cand.childFst == split_t::npos) {
						leaf.push_back(candIdx);
					}
					else {
						edgePath.push(cand.childFst);
						edgePath.push(cand.childSnd);
					}
				}
			}

			void collectAllLeafs(vector<edge_t>& edgesOut) {
				for (const auto& s : edges) {
					if (s.childFst == split_t::npos)
						edgesOut.push_back(s.parent);
				}
			}

			void candidatePairs(uint32_t srcI, uint32_t srcJ, vector<pair<uint32_t, uint32_t>>& res) { 
				vector<uint32_t> iLeafs;
				vector<uint32_t> jLeafs;
				collectLeafs(srcI, iLeafs);
				collectLeafs(srcJ, jLeafs);
				for (uint32_t i = 0; i < iLeafs.size(); i++) {
					for (uint32_t j = 0; j < jLeafs.size(); j++)
						res.push_back({i,j});
				}

			}

			void initialIntersectingPairs() { // collect all pairs from input
				// test each edge with each other edge
				//for edges a,b,c,d 
				//a ->b,c,d, b->c,d, c->d
				size_t initialSize = edges.size();

				vector<pair<uint32_t, uint32_t>> candidateLeafs; // tmp

				// first collect all intersections
				for (size_t iSrc = 0; iSrc < initialSize - 1; iSrc++) {
					for (size_t jSrc = iSrc + 1; jSrc < initialSize; jSrc++) {

						candidateLeafs.clear();
						candidatePairs(iSrc, jSrc, candidateLeafs);
						for (const auto& ij : candidateLeafs) {
							auto i = ij.first;
							auto j = ij.second;
							auto ei = edges[i].toSegment(vertices);
							auto ej = edges[j].toSegment(vertices);
							lnpz_linalg::segment2<double>::intersect_res_t isect = ei.intersect(ej);

							for (uint32_t indexOfCut = 0; indexOfCut < isect.count; indexOfCut++) {
								uint32_t cutVertexIdx = pushCutVertex(isect.pnt[indexOfCut]);
								cutLeafEdge(i, cutVertexIdx);
								cutLeafEdge(j, cutVertexIdx);
							}
						}
					}
				}
			}

			void doSelfIntersect() {
				initialIntersectingPairs();
			}

			static split_tree_t Init(const vector<edge_t>& input, vector<point2_t>& vertices, clusteringradius_t clustering) {
				size_t originalVertexSize = vertices.size();
				vector<split_t> edges;
				for (const auto& e : input) {
					edges.push_back({ e,split_t::npos, split_t::npos });
				}
				vector<pair<point2_t, uint32_t>> cutVertices;
				split_tree_t res = { clustering, edges, vertices, originalVertexSize,  cutVertices};
				return res;
			}
		};

		struct orientable_edge_t {
			size_t srcIndex;
			point2_t outerPoint;

			// need just ccw sorting as cw sorting is ccw sorting reversed

			// is a at lower index than b
			static bool OrderCCW(const orientable_edge_t& a, const orientable_edge_t& b) {

				// divide points to the four quadrants x>0y>0,x<0y>0,x<0y<0 and x>0y<0 and start sorting from x>0y>0

				bool aTop = a.outerPoint.y > 0;
				bool bTop = b.outerPoint.y > 0;

				if (aTop != bTop) // a and b other side of y=0 line, if a is top it goes first
					return aTop;

				if(aTop){ // top row , a & b both > 0
					if (a.outerPoint.x > b.outerPoint.x)
						return true;
					else if (a.outerPoint.x < b.outerPoint.x)
						return false;

					if (a.outerPoint.x > 0) // top right
						return a.outerPoint.y < b.outerPoint.y;
					else // top left
						return a.outerPoint.y > b.outerPoint.y;
				}
				else { // bottom row , a & b both < 0
					if (a.outerPoint.x < b.outerPoint.x)
						return true;
					else if (a.outerPoint.x > b.outerPoint.x)
						return false;

					if (a.outerPoint.x > 0) // bottom right
						return a.outerPoint.y < b.outerPoint.y;
					else // bottom left
						return a.outerPoint.y > b.outerPoint.y;
				}

			}

			static void SortCounterClockwise(vector<orientable_edge_t>& orientable) {
				sort(orientable.begin(), orientable.end(), orientable_edge_t::OrderCCW);
			}

			static void SortClockwise(vector<orientable_edge_t>& orientable) {
				sort(orientable.begin(), orientable.end(), orientable_edge_t::OrderCCW);
				reverse(orientable.begin(), orientable.end());
			}

			static vector<orientable_edge_t> GetOrientableEdges(uint32_t srcVertex, const vector<edge_t*>& edgesIn, const vector<point2_t>& vertices) noexcept {
				const point2_t vertexCoord = vertices[srcVertex];
				vector<orientable_edge_t> res;
				const point2_t* vertBuf = vertices.data();
				for (size_t i = 0; i < edgesIn.size(); i++) {
					const auto& edge = *edgesIn[i];
					point2_t vertOut;
					// set src vertex as coordinate origin
					if (srcVertex == edge.fst)
						vertOut = vertBuf[edge.snd] - vertexCoord;
					else // src == snd
						vertOut = vertBuf[edge.fst] - vertexCoord;

					res.push_back({ i, vertOut });
				}
			}

			static vector<size_t> GetCounterClockwiseOrder(uint32_t srcVertex, const vector<edge_t*>& edgesIn, const vector<point2_t>& vertices) {
				auto oriented = GetOrientableEdges(srcVertex, edgesIn, vertices);
				SortCounterClockwise(oriented);
				vector<size_t> res;
				for (auto ord : oriented)
					res.push_back(ord.srcIndex);
				return res;
			}
			
			static vector<size_t> GetClockwiseOrder(uint32_t srcVertex, const vector<edge_t*>& edgesIn, const vector<point2_t>& vertices) {
				auto oriented = GetOrientableEdges(srcVertex, edgesIn, vertices);
				SortClockwise(oriented);
				vector<size_t> res;
				for (auto ord : oriented)
					res.push_back(ord.srcIndex);
				return res;
			}
		};


		struct vertex_edge_map_t {

			vector<typed_edge_t> edges;
			const vector<point2_t>* vertices = nullptr;
			/*
				edgesFromVertex
				Each element contains list of edges coming and going from vertex
				-- ei --> [vert] -- ej -->

			*/
			vector<vector<typed_edge_t*>> edgesFromVertex;

			vector<bool> vertexVisited;

			vector<vector<edge_t>> outers;

			vertex_edge_map_t() {
			}

			void addEdges(const vector<edge_t>& edgesIn, bool outer) {
				for (const auto& e : edgesIn) {
					typed_edge_t te;
					te.fst = e.fst;
					te.snd = e.snd;
					te.outer = outer;
					edges.push_back(te);
				}
			}

			void setVertices(const vector<point2_t>* verticesIn) {
				vertices = verticesIn;
				vertexVisited = vector<bool>(vertices->size(), false);
			}

			vector<orientable_edge_t> getOrientableEdges(uint32_t srcVertex, const point2_t vertexCoord, const vector<typed_edge_t*>& typedEdges) noexcept {
				vector<orientable_edge_t> res;
				const point2_t* vertBuf = vertices->data();
				for (size_t i = 0; i < typedEdges.size(); i++) {
					const auto& edge = *typedEdges[i];
					point2_t vertOut;
					// set src vertex as coordinate origin
					if (srcVertex == edge.fst)
						vertOut = vertBuf[edge.snd] - vertexCoord;
					else // src == snd
						vertOut = vertBuf[edge.fst] - vertexCoord;

					res.push_back({ i, vertOut });
				}
			}

			void buildRelations(bool priorityOuter) {
				uint32_t maxVert = 0;

				for (const auto& e : edges) {
					maxVert = max(maxVert, e.fst);
					maxVert = max(maxVert, e.snd);
				}
				edgesFromVertex.resize(maxVert);

				// fill relations. Maintain priority so that there is only one edge
				// between vertices, and if there are both inner and outer wires, the wire with priority
				// is written
				for (auto& e : edges) {
					auto& fstContainer = edgesFromVertex[e.fst];
					auto& sndContainer = edgesFromVertex[e.snd];
					size_t idxAt;
					if (!e.containedIn(fstContainer, idxAt))
						fstContainer.push_back(&e);
					else if (e.outer == priorityOuter)
						fstContainer[idxAt] = &e;

					if (!e.containedIn(sndContainer, idxAt))
						sndContainer.push_back(&e);
					else if (e.outer == priorityOuter)
						sndContainer[idxAt] = &e;
				}

				// sort edges around vertices in counter clockwise or clockwise order
				const bool orderCCW = priorityOuter;
				for (size_t i = 0; i < edgesFromVertex.size(); i++) {
					vector<typed_edge_t*>& edgesAroundVertex = edgesFromVertex[i];
					if (edgesAroundVertex.empty())
						continue;

					vector<orientable_edge_t> orientable = getOrientableEdges(i, vertices->at(i), edgesAroundVertex);
					if (orderCCW)
						oriegnSortCounterClockwise(orientable);
					else
						SortClockwise(orientable);
					
					// reorder based on indices
					vector<typed_edge_t*> edgesTmp(edgesAroundVertex.size());
					for (size_t j = 0; j < edgesAroundVertex.size(); j++){
						const orientable_edge_t& oe = orientable[j];
						edgesTmp[j] = edgesAroundVertex[oe.srcIndex];
					}
					
					edgesFromVertex[i] = edgesTmp;
				}

				// now each vertex has it's edges sorted in counterclockwise or clockwise order
			}

			// find leftmost lowest vertex that has incoming and outgoing edges
			// never start from visited vertex - each valid loop must have at least two unvisited vertices
			string findCollectionStartVertexAndEdge(uint32_t& vert, typed_edge_t& edgeOutResult) const  {
				// Find lowest, leftmost vertex
				// First need to find all valid candidates as first vertex

				// Get all vertices that have outgoing and incoming edges that are unvisited

				vector<uint32_t> candidates;
				for (uint32_t vi = 0; vi < edgesFromVertex.size(); vi++) {
					const auto& edges = edgesFromVertex[vi];
					if (edges.empty())
						continue;
				
					/* Include only unused edges and those that can be returned to */
					int unvisitedCount = 0;
					for (const auto& te : edges) {
						if (!te->visited)
							unvisitedCount++;
					}

					if (unvisitedCount < 2)
						continue;

					bool hasOutgoing = false;
					bool hasIncoming = false;
					for (const auto& e : edges) {
						hasIncoming = hasIncoming || ((!e.visited) && e.isIncomingTo(vi));
						hasOutgoing = hasOutgoing || ((!e.visited) && e.isOutgoingFrom(vi));
					}
					if (hasOutgoing && hasIncoming)
						candidates.push_back(vi);
				}

				if(candidates.empty())
					return "Could not find any edges"; // something is wrong
				
				const point2_t* vb = &(*vertices)[0];
				uint32_t startVertIdx = candidates[0];
				point2_t lowest = vb[startVertIdx];
				for (auto vi : candidates) {
					auto p = vb[vi];
					if (p.y > lowest.y)
						continue;
					if (p.y < lowest.y || (p.x < lowest.x) /* when y is equal, find leftmost*/) {
						lowest = p;
						startVertIdx = vi;
					}
				}

				// now pick the outgoing edge with a direction with the lowest y value. If several outgoing edges
				// have SAME y value the earlier clustering is wrong 
				// the input polygon may have crossing edges, but the clustering and clipping stage should have cleaned them up into
				// non-intersecting segments
				typed_edge_t outEdge;
				float outY;
				bool foundEdge = false;
				const auto& edgesOfVert = edgesFromVertex[startVertIdx];
				for (const auto& e : edgesOfVert) {
					if (e.isOutgoingFrom(startVertIdx)) {
						if (!foundEdge) {
							foundEdge = true;
							auto dir = e.direction(*vertices);
							outY = dir.y;
							outEdge = e;
						}
						else {
							auto dir = e.direction(*vertices);
							if (dir.y < outY) {
								outY = dir.y;
								outEdge = e;
							}
						}
					}
				}

				if (!foundEdge)
					return "Could not find candidate edge";
				
				edgeOutResult = outEdge;

				return "";
			}

			uint32_t findPosition(const vector<typed_edge_t>& edges, const typed_edge_t& edgeToFind){
				const uint32_t npos = numeric_limits<uint32_t>::max();
				for (size_t i = 0; i < edges.size(); i++) {
					if (edgeToFind.isSame(edges[i])) {
						return i;
					}
				}
				return npos;
			}


			void extractLoop(const vector<uint32_t>& vertIdx) {

			}

			// call this sequentially to collect all loops into
			bool getNextLoop() {


			}

			// collect outer edges. return non-empty string with error message in case of error
			string collectOuterEdges() {
				// start from left bottom vertex, follow edges until a visited vertex is found. remove edges from visit set traveling backwards from the visited vertex to it's first occurrence
				for (auto& e : edges)
					e.visited = false;
				for (auto& vv : vertexVisited)
					vv = false;


				// First find lowest vertex
				uint32_t curVertIdx;
				typed_edge_t curEdge;
				string startRes = findCollectionStartVertexAndEdge(curVertIdx, curEdge);

				const uint32_t npos = numeric_limits<uint32_t>::max();
				map<uint32_t, typed_edge_t*> visited; // visited, coming from
				bool iterate = true;

				visited[curEdge.fst] = nullptr;
				vector<typed_edge_t> path;
				while (iterate) {
					auto nextVert = curEdge.snd;

					if (visited.count(nextVert) != 0) { // we have visited this vertex already - loop is complete
						// TODO
						// coming to an existing vertex - collect loop backwards
						vector<uint32_t> inloop;
						uint32_t startVert = nextVert;
						uint32_t loopVert = visited[startVert];
						inloop.push_back(startVert);
						while (loopVert != startVert) {
							inloop.push_back(loopVert);
							loopVert = visited[loopVert];
						}

						// Check if anything left to iterate over
						
					}

					// edges oriented in ccw | cw order based on prior configuration
					const auto& nextEdges = edgesFromVertex[nextVert];
					auto idx = findPosition(nextEdges, curEdge);
					auto nextEdge = nextEdges[(idx + 1) % nextEdges.size()];

				}


			}
		};

		// Use only for single wires
		// while the algorithm adds vertices, this stage does not yet remove them (as cuts before boolean only add vertices, never remove them)
		vector<vector<edge_t>> ClipSelfIntersectionsAndFixOrientation(const vector<edge_t>& input, bool outerWire, vector<point2_t>& vertices, clusteringradius_t clustering) {

			// Clean self intersections
			split_tree_t splitTree = split_tree_t::Init(input, vertices, clustering); // use split tree edges from now on
			splitTree.doSelfIntersect();

			vector<edge_t> selfEdges;
			splitTree.collectAllLeafs(selfEdges);
			return {selfEdges};

		}

		string CleanIndexedPolyfaceToRenderable(const indexedpolyface_t& source, double mergeRadius, indexedpolyface_t& result) {


			// TODO ADD HERE CLIPPER LIB

			clusteringradius_t clusteringRadius = clusteringradius_t::Create(mergeRadius);
			clusteredvertexbuilder_t builder;
			indexedpolyface_t clustered = builder.build(source, clusteringRadius);
			if (!builder.err.empty())
				return builder.err;

			edgefaces_t  initialEdges = edgefaces_t::Create(clustered);

			bool cleanedNonDegenerate;
			edgefaces_t cleaned = RemoveSimpleDegeneracies(initialEdges, cleanedNonDegenerate);
			if (!cleanedNonDegenerate)
				return "Input was degenerate";

			// a. Clean clustered data
			// b. Remove degenerate loops
			// c. compute intersection
			// d. 


			return "";
		}
#endif

#if 0
		struct edgefaces_t {
			vector<vector<edge_t>> outerWires;
			vector<vector<edge_t>> innerWires;

			vector<point2_t> vertices;

			static edgefaces_t Create(indexedpolyface_t& pf) {
				edgefaces_t  ef;
				ef.vertices = pf.vertices;
				for (const auto& outerwire : pf.outerWires)
					ef.outerWires.push_back(outerwire.toEdges());

				for (const auto& innerwire : pf.innerWires)
					ef.innerWires.push_back(innerwire.toEdges());

				return ef;
			}
			struct edgeenumerator_t {
				const edgefaces_t& edges;
				uint32_t outerWireIdx = 0;
				uint32_t edgeIdx = 0;
				uint32_t innerWireIdx = 0;
				edge_t edge;

				bool next() {
					if (outerWireIdx < edges.outerWires.size()) {
						if (edgeIdx < edges.outerWires[outerWireIdx].size()) {
							edge = edges.outerWires[outerWireIdx][edgeIdx];
						}
						else {
							edgeIdx = 0;
							outerWireIdx++;
							if (outerWireIdx < edges.outerWires.size()) {
								edge = edges.outerWires[outerWireIdx][edgeIdx];
							}
						}
					}

					if (outerWireIdx >= edges.outerWires.size() && innerWireIdx < edges.innerWires.size()) {
						if (edgeIdx < edges.innerWires[innerWireIdx].size()) {
							edge = edges.innerWires[innerWireIdx][edgeIdx];
						}
						else {
							edgeIdx = 0;
							innerWireIdx++;
							if (innerWireIdx < edges.innerWires.size()) {
								edge = edges.innerWires[innerWireIdx][edgeIdx];
							}
							else {
								return false;
							}
						}
					}
					else {
						return false;
					}

					edgeIdx++;
				}
			};

			edgeenumerator_t getEdgeEnumerator() const {
				return { *this,0,0,0,{0,0} };
			}
#endif
		};

