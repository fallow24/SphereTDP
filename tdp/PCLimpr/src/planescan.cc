/** @file 
 *  @brief this file contains the PlaneScan Class that represents a Scan 
 *  and holds the information about Point-to-Plane correspondences.
 *
 *  @author Fabian Arzberger, JMU, Germany.
 * 
 *  Released under the GPL version 3.
 */

#include "planescan.h"
#include "scanio/helper.h"

#include "debugutil.h"

// initialize the static members
PlaneScans PlaneScan::allPlaneScans;
unsigned int PlaneScan::idx_global = 0;
double PlaneScan::_eps_dist = EPS_POINT_2_PLANE_DIST;
bool PlaneScan::reduce = false; 
Planes PlaneScan::allPlanes;
bool PlaneScan::use_correspondence_min = false;
bool PlaneScan::use_clustering = true;
int PlaneScan::k_neighbours = 20;
double PlaneScan::d_growth = 5.0;
int PlaneScan::n_min_clusterpoints = 100;
double PlaneScan::eps_similarity = 5;
string PlaneScan::cluster_outpath = "";

// Calculates the distance from a transformed point "xyz" to plane "p"
double PlaneScan::dist2Plane(const double *xyz, const double *trans, const NormalPlane *p)
{
    double xyz_transformed[3];
    transform3(trans, xyz, xyz_transformed);
    return dist2Plane( xyz_transformed, p );
}

// Calculates the distance from a point "xyz" to plane "p"
double PlaneScan::dist2Plane(const double *xyz, const NormalPlane *p)
{
    double v[3];
    v[0] = xyz[0] - p->x[0];
    v[1] = xyz[1] - p->x[1];
    v[2] = xyz[2] - p->x[2];
	return Dot(v, p->n); 
}

// Checks if a point "xyz" with transformation "trans" lies on the plane "p"
bool inline PlaneScan::isInPlane(double *xyz, double *trans, const NormalPlane *p)
{
    // Calculate distance to plane and apply threshold
    return fabs( dist2Plane(xyz, trans, p) ) < _eps_dist;
}

// Static setter that sets the distance threshold for plane correspondence.
// Use before constructing the object!
void PlaneScan::setEpsDist(double eps_dist)
{
	_eps_dist = eps_dist;
}

void PlaneScan::setUseCorrespondenceMin(bool val)
{
    use_correspondence_min = val;
}

void PlaneScan::setKNearest(int k)
{
    k_neighbours = k;
}

void PlaneScan::setUseClustering(bool val)
{
    use_clustering = val;
}

void PlaneScan::setGrowthThresh(double val)
{
    d_growth = val;
}

void PlaneScan::setEpsSimilarity(double val)
{
    eps_similarity = val;
}

void PlaneScan::setMinClusterPoints(int n)
{
    n_min_clusterpoints = n;
}

void PlaneScan::setClusterVisualizationPath(string path)
{
    cluster_outpath = path;
}

void PlaneScan::calcNormals(DataXYZ &xyz)
{
    cout << "Calculating normals for scan" << identifier << "...";
    // Setup
    vector<Point> ps; // points
    vector<Point> ns; // normals
    ps.reserve(xyz.size());
    ns.reserve(xyz.size());
    for(size_t j = 0; j < xyz.size(); j++) 
        ps.push_back(Point(xyz[j][0], xyz[j][1], xyz[j][2]));
    // Calc normals with aproximate k nearest neighbours 
    calculateNormalsApxKNN(ns, ps, k_neighbours, rPos, 1.0);
    // Store normals in PlaneScan object 
    for (int i = 0; i < nrpts; i++) 
    {
        normals[i] = new double[3];
        normals[i][0] = ns.at(i).x;
        normals[i][1] = ns.at(i).y;
        normals[i][2] = ns.at(i).z;  
    }
    cout << " done." << endl;	
}

double inline distEuklid(double *p1, double *p2)
{
    return sqrt( sqr(p1[0]-p2[0]) + sqr(p1[1]-p2[1]) + sqr(p1[2]-p2[2]) );  
}

bool inline PlaneScan::similar(double* n1, double* n2)
{
    double alpha = acos(Dot(n1, n2));
    if (alpha > M_PI) alpha = 2*M_PI - alpha; // e.g. 356deg is actually 4deg
    return deg(alpha) < eps_similarity;
}

// Calculates the mean normal of a cluster
// Cached Normal Mean 
double* PlaneScan::normalClusterMean(vector<int> &indices)
{
    double *nmean = new double[3];
    nmean[0] = 0; nmean[1] = 0; nmean[2] = 0; 
    if (indices.size() == 0) return nmean;
    for (uint i = 0; i < indices.size(); ++i) {
        nmean[0] += normals[indices[i]][0];
        nmean[1] += normals[indices[i]][1];
        nmean[2] += normals[indices[i]][2];
    } 
    nmean[0] /= indices.size();
    nmean[1] /= indices.size();
    nmean[2] /= indices.size();
    return nmean;
}

// Calculates the variance of the <k> nearest normals, their indices being stored in <nidx>
double* PlaneScan::normalVariance(ANNidxArray nidx, int k)
{
    // Calculate mean normal for neighbours
    double nmean[] = {0, 0, 0};
    for (int i = 0; i < k; ++i) {
        nmean[0] += normals[nidx[i]][0];
        nmean[1] += normals[nidx[i]][1];
        nmean[2] += normals[nidx[i]][2];
    } 
    nmean[0] /= k;
    nmean[1] /= k;
    nmean[2] /= k;

    // Calc variance 
    double *nvar = new double[3];
    nvar[0] = 0; nvar[1] = 0; nvar[2] = 0;
    for (int i = 0; i < k; ++i) {
        nvar[0] += sqr( normals[nidx[i]][0] - nmean[0] );
        nvar[1] += sqr( normals[nidx[i]][1] - nmean[1] );
        nvar[2] += sqr( normals[nidx[i]][2] - nmean[2] );
    } 
    nvar[0] /= k;
    nvar[1] /= k;
    nvar[2] /= k;
    return nvar;
}

// Calculates the distance from point "p" to the cluster "cluster",
// i.e. the distance from the point "p" to a point in the cluster, where the distance is minimal 
double PlaneScan::clusterPointDist(vector<int> &cluster, double *p)
{
    // TODO: use search tree for mindist, its muhc faster.
    double minDist = DBL_MAX, dist;
    for(int idx : cluster)
    {
        dist = distEuklid(p, points[idx]);
        if ( dist < minDist ) minDist = dist; 
    }
    return minDist;
}

// MAYOR TODO: SPEEDUP THAT SHIT
// Calls the growRegionRecursive(...) function and refines the found clusters.
void PlaneScan::calcLocalClusters()
{
    // Setup searchtree
    ANNpointArray pa = annAllocPts(nrpts, 3);
    for (int i = 0; i < nrpts; ++i) {
        pa[i][0] = points[i][0];
        pa[i][1] = points[i][1];
        pa[i][2] = points[i][2];
    }
    ANNkd_tree t(pa, nrpts, 3);
    ANNidxArray nidx = new ANNidx[k_neighbours];
    ANNdistArray d = new ANNdist[k_neighbours];
    
    // Setup memory for already labled points, init with false
    bool* labled = new bool[nrpts]; for(int i=0;i<nrpts;++i) labled[i] = false;
    
    // Put the points into clusters, representing local planes.
    for (int j = 0; j < nrpts; ++j) 
    {
        // Break condition if we visited a point already
        if (labled[j]) break;
        // init search tree for nearest neighbours
        int k = k_neighbours;
        ANNpoint p = pa[j];
        t.annkSearch(p, k, nidx, d, 1.0);

        // get best cluster
        for (uint i = 0; i < clusters.size(); i++) 
        {
            // The point fits well into an already existing cluster if conditions are met:
            // normals must be similar & distance to an existing cluster must be small
            double *ncm = normalClusterMean(clusters[i]);
            if ( similar(ncm, normals[j]) 
                && clusterPointDist(clusters[i], points[j]) < d_growth )
            {
                clusters[i].push_back(j);
                labled[j] = true;
                break;
            }
            delete[] ncm;   
        }
        // If the point did not get labled, open new cluster.
        if (!labled[j])
        {
            clusters.push_back( vector<int>() );
            clusters[clusters.size()-1].push_back(j);
            labled[j] = true;
        }
    }
    
    // delete clusters that have too few points 
    for (Clusters::iterator it = clusters.begin();
         it != clusters.end();
         ) // no increment here
    {
        // TODO: maybe add other criteria?
        if (it->size() < n_min_clusterpoints) it = clusters.erase(it);
        else ++it; // increment normaly only if you dont delete the elem
    }

    cout << clusters.size() << " clusters found for scan " << identifier << endl;
    if ( cluster_outpath != "")
    {
        if (cluster_outpath[ cluster_outpath.size() - 1] != '/') 
            cluster_outpath += "/";
        writeClustersToScanfile(clusters, cluster_outpath);
        cout << "Writing colored clusters to " << cluster_outpath << endl;
    } 
}

// Kinda neat debug function.
// Saves the result of Clustering to a scanfile. 
// Can be visualized with "show -f uos_rgb -c"
void PlaneScan::writeClustersToScanfile(Clusters& clusters, string path)
{
    ofstream ofile((path+"scan"+identifier+".3d").c_str(), ios_base::out);
    hsv color_hsv = {0, 1, 1};
    for (int i = 0; i < clusters.size(); ++i)
    {
        rgb color_rgb = hsv2rgb( color_hsv );
        for(int j = 0; j < clusters[i].size(); ++j)
        {
            ofile << points[clusters[i][j]][0] << " "
                << points[clusters[i][j]][1] << " "
                << points[clusters[i][j]][2] << " "
                << (int) (color_rgb.r*255) << " " 
                << (int) (color_rgb.g*255) << " " 
                << (int) (color_rgb.b*255) << endl;
        }
        color_hsv.h += 360 / clusters.size(); 
    }
    ofstream ofileposedummy((path+"scan"+identifier+".pose").c_str(), ios_base::out);
    ofileposedummy << 0 << " " << 0 << " " << 0 << endl;
}

void PlaneScan::labelDistEuklid()
{
    for (int i = 0; i < nrpts; ++i)
    {
        Planes *planeList = new Planes(); // can be also empty or bigger than one
        for ( const auto& plane : allPlanes )
            if ( isInPlane(points[i], transMat, plane) )
                planeList->push_back( plane );


        if (use_correspondence_min)
        {
            // find the plane with the minimum distance amongst all candidates
            if (!planeList->empty())
            {
                NormalPlane* min = planeList->at(0);
                for ( unsigned int j = 0; j < planeList->size(); ++j)
                    if ( dist2Plane(points[i], transMat, planeList->at(j)) 
                        < dist2Plane(points[i], transMat, min) )
                        min = planeList->at(j);
                // Store a correspondence (unordered)
                correspondences.insert( Correspondence( points[i], min ) );
            }
        } 
        else
        {
            if (planeList->size() == 1)
                correspondences.insert( Correspondence( points[i], planeList->at(0)) ); 
        } 
    }
}

void PlaneScan::labelUseClusters()
{
    // For each plane, count how many points from a cluster fall into it
    const int nplanes = allPlanes.size();
    const int nclusters = clusters.size();
    int cccount[nclusters][nplanes]; // cluster correspondence count
    // Initialize with zero
    for (int c = 0; c < nclusters; ++c) 
        for (int p = 0; p < nplanes; ++p) 
            cccount[c][p] = 0;
    
    // Iterate the clusters...
    for (int k = 0; k < nclusters; ++k) {
        // ...for each point in the cluster...
        for (int q = 0; q < clusters[k].size(); ++q) {
            // ...check if it belongs to planes...
            for (int j = 0; j < nplanes; ++j) {
                // ...and if so...
                if ( isInPlane( points[clusters[k][q]], transMat, allPlanes[j] ) ) {
                    // ...increment the k-th cluster count for the j-th plane.
                    cccount[k][j]++;
                }
            }
        }
    }
        
        // NEAT FOR DEBUGGING: SHOWS CCCOUNT AS A TABLE
        // cout << "\t";
        // for (int n = 0; n < nplanes; ++n) cout << "P" << n << "\t";
        // cout << endl;
        // for (int i = 0; i < nclusters; ++i) {
        //     cout << "C" << i << "\t";
        //     for (int j = 0; j < nplanes; ++j) {
        //         cout << cccount[i][j] << "\t";
        //     }
        //     cout << endl;
        // }

    // Look at each cluster
    for (int c = 0; c < nclusters; ++c)
    {
        // Get mean normal for the cluster
        double *ncm = normalClusterMean(clusters[c]);
        double alpha_min = DBL_MAX;
        int max_count = 0;
        int p_alpha_min, p_max_count;
        // Search for a plane that has a similar normal and a big overlapping region 
        for (int p = 0; p < nplanes; ++p) 
        {
            // Skip planes that dont overlap with clusters
            if (cccount[c][p] != 0)
            {
                double alpha = acos(Dot(ncm, allPlanes[p]->n));
                if (alpha > M_PI) alpha = 2*M_PI - alpha; // e.g. 356deg is actually 4deg
                // Always keep minimum angle from cluster to plane
                if (alpha < alpha_min) {
                    alpha_min = alpha;
                    p_alpha_min = p;
                }
                // Also keep maximum overlapping region 
                if (cccount[c][p] > max_count) {
                    max_count = cccount[c][p];
                    p_max_count = p;
                }
            }
                
        }
        delete[] ncm;

        // Fuzzy approach to put min. angle together with max. overlap
        if (max_count != 0 && alpha_min < M_PI_4)
        {
            for (int i = 0; i < clusters[c].size(); ++i) 
                correspondences.insert( Correspondence( points[clusters[c][i]], allPlanes[p_alpha_min] ) );
        } 
        // if the clustering didnt work so well, go for classical euklidian labels
        else labelDistEuklid(); 
    }
}

/*
 * With hough transformation, there were planes found specified in "normals.list".
 * Now, a point-2-plane model should be established which labels the points in the current scan
 * onto these planes. The correspondence model should be as accurate as possible.
 * You can therefore use multiple models, depending on your situation.
 * Some models are better in different scenarios:
 * 
 *      - (1) Min Point2Plane distance thresholding:
 *              activated using the "--use_min_cor" option. Look at all the
 *              planes and apply a threshold around them. All points that
 *              fall into the threshold are labled to that plane. If a point
 *              could be labled onto multiple planes, use the one with min distance.
 *      
 *      - (2) Single Correspondence distance thresholding:
 *              Used when no other option is given.
 *              Similar to (1). However, only points with one correspondence 
 *              are considered for labeling. Points that could be labled onto multiple
 *              planes are not considered for correspondence.
 * 
 *      - (3) Local Normal Clustering
 *              Activated using the "--use_clustering" option.
 *              Before labeling points to planes, clusters of normals are calculated
 *              for the scan. Then, apply a threshold around the planes similar to (1) or (2).
 *              Assign each cluster to one plane by using the biggest overlapping region
 *              between them.
 */
void PlaneScan::labelPoints(bool quiet)
{
    if (!quiet) cout << "Labeling points on planes for scan" << identifier << endl;
    if (use_clustering) labelUseClusters();
    else labelDistEuklid();
    if (!quiet)
    {
        int n = correspondences.size();  
        cout << "... " << n << " correspondence" << (n == 1 ? "":"s") << " found." << endl; 
    }
}

PlaneScan::PlaneScan(Scan *scan)
{
    // Read disk data
	idx = idx_global++;
    basedir = scan->getPath();
	timestamp = scan->getTimeStamp();
    identifier = scan->getIdentifier();

    // Read pose data
	const double *scanRPos = scan->get_rPos();
	const double *scanRPosTheta = scan->get_rPosTheta();
	for (int i = 0; i < 3; i++) 
    {
		rPos[i] = scanRPos[i];
		rPosTheta[i] = scanRPosTheta[i];
	}
    EulerToMatrix4(rPos, rPosTheta, orig_transMat);
	EulerToMatrix4(rPos, rPosTheta, transMat);

    // Read point data
    /*
     * Type of reduction was set earlier through program options
     * "xyz reduced" -> global reduced coordinates
     * "xyz reduced show" -> local reduced coordinates
     */ 
    string redstring = reduce ? " reduced show" : "";
	DataXYZ xyz(scan->get("xyz" + redstring));
	nrpts = xyz.size();
	if (nrpts != 0) 
    {
		points = new double*[nrpts];
        // Store points
		for (int i = 0; i < nrpts; i++) 
        {
			points[i] = new double[3];
			points[i][0] = xyz[i][0];
			points[i][1] = xyz[i][1];
			points[i][2] = xyz[i][2];  
        }
        if (use_clustering) 
        {
            // Calculate normals 
            // Setup clusters, initialize with one cluster
            clusters = Clusters(1);
            normals = new double*[nrpts];
            calcNormals(xyz);	
            // Use normals to cluster points together
            calcLocalClusters();
        }
	} else {
		cout << "Scan nr. " << idx << " skipped. Contains no data." << endl;
		empty = true;
	}
    allPlaneScans.push_back(this);
}

PlaneScan::PlaneScan(Scan *scan, Planes &planes) : PlaneScan(scan)
{
    // Save plane data
    allPlanes = planes;
    // Label the points
	labelPoints(false);
}

void PlaneScan::setPlanes(Planes &ps)
{
    allPlanes = ps;
}

// Return the point at index <n>
const double* PlaneScan::operator()(int n)
{
	return points[n];
}

bool PlaneScan::isEmpty()
{
	return correspondences.size() == 0;
}

bool PlaneScan::isOkay()
{
	return !isEmpty();
}

unsigned int PlaneScan::index()
{
    return idx;
}

void PlaneScan::setReduce(bool val)
{
    reduce = val;
}

// instance-function, calculates squared point-2-plane error of one Scan
double PlaneScan::calcErr()
{
	double sum = 0.0;
	map<double*, NormalPlane*>::iterator it = correspondences.begin();
	for (; it != correspondences.end() ; ++it ) sum += sqr( dist2Plane( it->first, transMat, it->second ) );
	return sum;
}

// static function, calculates total point-2-plane distance of all Scans
double PlaneScan::totalError()
{
	double sum = 0.0;
	for ( const auto& pscan : allPlaneScans) sum += pscan->calcErr();
	return sqrt(sum);
}

void PlaneScan::addFrame(Scan::AlgoType type)
{
    sout << transMat << type << endl;
}

string PlaneScan::getFrameData()
{
    return sout.str();
}

bool PlaneScan::writeFrames()
{
    map<std::string, std::string> framesdata;
    for (unsigned int i = 0; i < PlaneScan::allPlaneScans.size(); i++) {
        PlaneScan *pScan = PlaneScan::allPlaneScans[i];
        string framespath = boost::filesystem::path(pScan->basedir + "scan" + pScan->identifier + ".frames").string();
        framesdata[framespath] = pScan->getFrameData();
    }
    bool state = write_multiple(framesdata, ios_base::out);
    return state;
}

// Little helper to count lines in a string
unsigned int countLines(const string &s)
{
    int nlines = 0;
    for( char c : s ) if ( c == '\n' ) ++nlines;
    return nlines;
}

void PlaneScan::fillFrames()
{
    // find maximum number of lines
    int lmax = 0, l;
    for (const auto pScan : allPlaneScans)
    {
        l = countLines( pScan->sout.str() );
        if ( l > lmax ) lmax = l;
    }
    // fill all .frames with the last entry again
    for (const auto pScan : allPlaneScans)
    {
        l = countLines( pScan->sout.str() );
        for (int i = 0; i < lmax - l; i++)
            pScan->addFrame( Scan::ICPINACTIVE );
    }
}

// Destructor
PlaneScan::~PlaneScan()
{
	allPlaneScans.clear();
	correspondences.clear();
}