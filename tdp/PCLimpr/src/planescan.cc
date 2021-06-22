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
#include <queue>
#include "util.h"

// initialize the static members
PlaneScans PlaneScan::allPlaneScans = PlaneScans(0);
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
bool PlaneScan::read_normals = false;
bool PlaneScan::read_clusters = false;
bool PlaneScan::use_normal_cor = false;

double inline distSqr(double *p1, double *p2)
{
    return sqr(p1[0]-p2[0]) + sqr(p1[1]-p2[1]) + sqr(p1[2]-p2[2]);
} 

double inline distEuklid(double *p1, double *p2)
{
    return sqrt( distSqr(p1, p2) );  
}

// Returns the smallest angle between n1 and n2. Looks at all 4 quadrants. Returns in rad
double inline angleBetweenNormals(double* n1, double* n2)
{
    double alpha = acos(Dot(n1, n2));
    // If between 270 and 360 degrees, just take the conjugate angle.
    if (alpha > M_PI + M_PI_2) alpha = M_2_PI - alpha;
    // If between 180 and 270, flip normal.
    else if (alpha > M_PI) alpha = alpha - M_PI;
    // If between 90 and 180, flip normal. 
    else if (alpha > M_PI_2) alpha = M_PI - alpha;
    return alpha;
}

// Looks at two normals and returns if they are similar.
// Considers all quadrants, e.g. an angle between 180 deg. means equal normals.
bool inline PlaneScan::similar(double* n1, double* n2)
{
    return deg(angleBetweenNormals(n1, n2)) < eps_similarity;
}

// Most important constructor.
// Initializes a PlaneScan object from a scan, as read from the scan directory
// Infers additional information to the scan such as point normals or corresponding planes.
PlaneScan::PlaneScan(Scan *scan)
{
    // Read disk data
	idx_global += 1;
    idx = idx_global;
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

    string redstring = reduce ? " reduced show" : "";
	DataXYZ xyz(scan->get("xyz" + redstring));

    /*
     * Read normals from scan files
     */

    if (read_normals)
    {
        DataNormal scan_normals = scan->get("normal");
        int nrnormals = scan_normals.size();
        normals = new double*[nrnormals];
        for (int i = 0; i < nrnormals; ++i)
        {
            normals[i] = new double[3];
            normals[i][0] = scan_normals[i][0]; //nx;
            normals[i][1] = scan_normals[i][1]; //ny;
            normals[i][2] = scan_normals[i][2]; //nz;
        }

    /*
     * Read clustering (corresponding to an RGB value)
     */

    } else if (read_clusters) {
        DataRGB scan_color = scan->get("rgb");
        readClusters(scan_color);
    }
    // Read point data
    /*
     * Type of reduction was set earlier through program options
     * "xyz reduced" -> global reduced coordinates
     * "xyz reduced show" -> local reduced coordinates
     */ 
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

        if (use_clustering && !read_clusters) 
        {
            // Calculate normals (if you havent read them already)
            if (!read_normals)
            {
                normals = new double*[nrpts];
                calcNormals(xyz);
            }	
            // Setup clusters, initialize with one clusterlabelUseNorm
            clusters = Clusters(1);
            // Setup normal cluster mean cache 
            ncm_cache = vector<double*>(0);
            // Init normal cluster mean cache with a single entry
            double *ncm_init = new double[3];
            ncm_init[0] = 0;
            ncm_init[1] = 0;
            ncm_init[2] = 0;
            ncm_cache.push_back( ncm_init );

            // Init Aprox. Nearest Neighbour point storage
            ann_pts = annAllocPts(nrpts, 3);
            for (int i = 0; i < nrpts; ++i) {
                // unfortunaltey, the tree needs his own data structure...
                // TODO: use KDtree class implenentation form slam6d instead
                ann_pts[i][0] = points[i][0];
                ann_pts[i][1] = points[i][1];
                ann_pts[i][2] = points[i][2];
            }
            tree = new ANNkd_tree(ann_pts, nrpts, 3);
            // Use normals to cluster points together
            calcLocalClusters();
        }
	} else {
		cout << "Scan nr. " << idx << " skipped. Contains no data." << endl;
		empty = true;
	}
    allPlaneScans.push_back(this);
   
}

int PlaneScan::readClusters(DataRGB &scan_color)
{
    int nrclusterpts = scan_color.size();
    // Init colors with size 1 for the first point
    vector<rgb> colors(0);
    rgb color = {   
        scan_color[0][0],   // I know. These are some
        scan_color[0][1],   // dirty "unsigned char" to "double" conversions.
        scan_color[0][2]    // Your compiler may complain about that.
    };
    colors.push_back(color);
    // Init cluster with size 1, corresponding to color of first point
    clusters = Clusters(1);
    clusters[0].push_back(0);
    bool rgb_equals = false;
    for (int i = 1; i < nrclusterpts; ++i)
    {   
        rgb color1 = {   
            scan_color[i][0],   // However, i ensure you that 
            scan_color[i][1],   // they will always work in that usecase,
            scan_color[i][2]    // so you don't actually have to worry about it.
        }; 
        for (int j = 0; j < colors.size(); ++j)
        {
            rgb_equals = rgbEquals(color1, colors[j]);
            if ( rgb_equals ) clusters[j].push_back(i);
        }
        if ( !rgb_equals ) {
            clusters.push_back( Cluster(0) );
            colors.push_back(color1);
            clusters[ clusters.size() - 1].push_back(i);
        }
    } 
    //cout << colors.size() << " clusters found." << endl;    
}

// Calculates the distance from a transformed point "xyz" to plane "p"
// TODO: Refactor to use Bounding.
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

void PlaneScan::setReadNormals(bool val)
{
    read_normals = val;
}

void PlaneScan::setReadClusters(bool val)
{
    read_clusters = val;
}

void PlaneScan::setUseNormalCor(bool val) {
    use_normal_cor = val;
}

double PlaneScan::clusterDist(Cluster &c1, Cluster &c2)
{
    double min_dist = DBL_MAX;
    double dist;
    for (int i : c1) {
        for (int j : c2) {
            dist = distEuklid(points[i], points[j]);
            if (i != j && distEuklid(points[i], points[j]) < min_dist )
                min_dist = dist;
        }
    }
    return min_dist;
}

void PlaneScan::calcNormals(DataXYZ &xyz)
{
    // Setup
    vector<Point> ps; // points
    vector<Point> ns; // normals
    ps.reserve(xyz.size());
    ns.reserve(xyz.size());
    for(size_t j = 0; j < xyz.size(); j++) 
        ps.push_back(Point(xyz[j][0], xyz[j][1], xyz[j][2]));
    // Calc normals with aproximate k nearest neighbours 
    //calculateNormalsApxKNN(ns, ps, k_neighbours, rPos, 1.0);
    calculateNormalsKNN(ns, ps, k_neighbours, rPos);
    //calculateNormalsAdaptiveKNN(ns, ps, k_neighbours, 2*k_neighbours, rPos);
    // Store normals in PlaneScan object 
    for (int i = 0; i < nrpts; i++) 
    {
        normals[i] = new double[3];
        normals[i][0] = ns.at(i).x;
        normals[i][1] = ns.at(i).y;
        normals[i][2] = ns.at(i).z;  
    }
}

// Adds the normal for the point at index <idxp> to the normal cluster mean cache for cluster at index <idxc>
// Requires precalculated normals.
void PlaneScan::update_ncm_cache(int idxc, int idxp)
{
    ncm_cache[idxc][0] += normals[idxp][0];
    ncm_cache[idxc][1] += normals[idxp][1];
    ncm_cache[idxc][2] += normals[idxp][2];
}

// Cached approach for calculating clusters. To be used ONLY and ONLY for CALCULATION of clusters.
// Does not work at all after some clusters are merged together or deleted.
double * PlaneScan::normalClusterMeanCached(int idxc)
{
    double *nmean = new double[3];
    nmean[0] = 0; nmean[1] = 0; nmean[2] = 0; 
    const int csize = clusters[idxc].size();
    if (csize == 0) return nmean;
    nmean[0] = ncm_cache[idxc][0] / csize;
    nmean[1] = ncm_cache[idxc][1] / csize;
    nmean[2] = ncm_cache[idxc][2] / csize;
    Normalize3(nmean);
    return nmean;
}

// Calculate normal mean for a cluster, given only the points but no normal information.
double* PlaneScan::calcNormalOnDemand(vector<int> &indices)
{
    vector<Point> temp;
    for (int i : indices) temp.push_back( Point(points[i]) );
    double *norm = new double[3];
    double eigen[3];
    calculateNormal(temp, norm, eigen);
    return norm;
}

// Calculates the mean normal of a cluster
double* PlaneScan::normalClusterMean(vector<int> &indices)
{
    double *nmean = new double[3];
    nmean[0] = 0; nmean[1] = 0; nmean[2] = 0; 
    if (indices.size() == 0) return nmean;
    
    // When clusters are read, there is no normal information.
    if (read_clusters) return calcNormalOnDemand(indices);

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

// This shoudl only be used when normals are available.
// Clusters should always be constructed from those preexisting normals, otherwise this function wont work.
// Think about it. When you read a full pre-defined cluster using RGB, you end up having ONLY the mean normal of this cluster.
// However, you DONT have all the preexisting normals anymore, thus you cant calculate a variance.
double* PlaneScan::normalVariance(Cluster& cluster)
{
    double* nmean = normalClusterMean(cluster);
    // Calc variance 
    double *nvar = new double[3];
    nvar[0] = 0; nvar[1] = 0; nvar[2] = 0;
    for (int i = 0; i < cluster.size(); ++i) {
        nvar[0] += sqr( normals[cluster[i]][0] - nmean[0] );
        nvar[1] += sqr( normals[cluster[i]][1] - nmean[1] );
        nvar[2] += sqr( normals[cluster[i]][2] - nmean[2] );
    } 
    nvar[0] /= cluster.size();
    nvar[1] /= cluster.size();
    nvar[2] /= cluster.size();
    return nvar;
}

// Calculates the distance from point "p" to the cluster "cluster",
// i.e. the distance from the point "p" to a point in the cluster, where the distance is minimal 
double PlaneScan::clusterPointDist(vector<int> &cluster, double *p)
{
    double dist = DBL_MAX, d;
    for (uint i = 0; i < cluster.size(); ++i) {
        d = distSqr(p, points[cluster[i]]);
        if ( d < dist) dist = d;
    }
    return dist;
}

// Sort clusters according to their size
bool cSizeCompare(Cluster& i, Cluster& j)
{
    return i.size() > j.size();
}

/*
 * Calculates local clusters in the scan, using normal information of the points.
 * Normals get calculated automatically if not read beforehand.
 */
void PlaneScan::calcLocalClusters()
{
    // Setup memory for already labled points, init with false
    bool* labled = new bool[nrpts]; for(int i=0;i<nrpts;++i) labled[i] = false;

    // Use a queue to ensure correct iteration order
    // used to recreate a "recusive" growing approach for region growing.
    // unfortunatley, real recusive approach will shit on your stack mem. (stackoverflow)
    // Also, stack switch has quite expensive cost.  
    queue<int> idx_queue;  

    // Init first cluster with first point
    clusters[0].push_back(0);
    // Also init normal cluster mean cache
    update_ncm_cache(0, 0); // Add 0st point to the 0st cluster
    labled[0] = true; // therefore, 0st point is already labled.

    // Put the points into clusters, representing local planes.
    int index;
    for (int i = 1; i < nrpts; ++i) 
    {
        // Break condition if we visited a point already
        if (labled[i]) continue;
        // If we havent visited a point already, push it to the queue
        if (idx_queue.empty()) idx_queue.push(i);

        // The queue stores indices for nearest neighbours, used for iteration
        while(!idx_queue.empty())
        {
            index = idx_queue.front();
            idx_queue.pop();
            if (labled[index]) continue;

            //TODO: use another searchtree class that supports running in a threaded loop.
            // Setup output of ann algorithm
            ANNidxArray nidx = new ANNidx[k_neighbours];
            ANNdistArray d = new ANNdist[k_neighbours];
            int k = k_neighbours;
            // query point
            ANNpoint p = ann_pts[index];
            tree->annkSearch(p, k, nidx, d, 0);
            
            //Add nearest neighbours to queue, so you look at them the next iteration
            for (int q = 1; q < k_neighbours; ++q) {
                int nextIndex = nidx[q];
                if (!labled[nextIndex])
                    idx_queue.push( nextIndex );
            }

            // Now, fit the point to a cluster, or open a new cluster
            int j = index;
            for (uint c = 0; c < clusters.size(); c++) 
            {
                // The point fits well into an already existing cluster if conditions are met:
                // normals must be similar & distance to an existing cluster must be small
                double *ncm = normalClusterMeanCached(c);
                if ( similar(ncm, normals[j]) & clusterPointDist(clusters[c], points[j]) < sqr(d_growth) )
                {
                    clusters[c].push_back(j);
                    update_ncm_cache(c, j);
                    labled[j] = true;
                    break;
                }
                delete[] ncm;   
            }
            // If the point did not get labled, open new cluster.
            if (!labled[j])
            {
                // Add new index vector --> a new cluster
                clusters.push_back( Cluster(0) );
                // Add point to the new, empty cluster
                clusters[clusters.size()-1].push_back(j);
                // Add new normal mean for the new cluster
                double *ncm_tail = new double[3];
                ncm_tail[0] = 0; 
                ncm_tail[1] = 0; 
                ncm_tail[2] = 0;
                ncm_cache.push_back( ncm_tail );
                // Update mean normal for new cluster
                update_ncm_cache(clusters.size()-1, j);
                // don't visit that point again
                labled[j] = true;
            }
            delete[] nidx;
            delete[] d;
        }
    }

    // Cleanup, merge clusters with similar normal if they are near enough
    // TODO: Iterate smarter. e.g. 1 <--> 100 is checked twice: 100 <--> 1 
    for (uint i = 0; i < clusters.size(); ++i)
    {
        for (uint j = 0; j < clusters.size(); ++j)
        {
            double alpha = angleBetweenNormals(normalClusterMean(clusters[i]), normalClusterMean(clusters[j]));
            //cout << "Alpha between " << i << " and " << j << " = " << deg(alpha) << endl;
            if (i != j  
                && deg(alpha) <= eps_similarity
                && clusterDist(clusters[i], clusters[j]) < d_growth)
            {
                mergeClusters(i, j);
            }
        }
    }

    // Use 3 biggest clusters.
    // sort(clusters.begin(), clusters.end(), cSizeCompare);
    // int i = 0;
    // for (Clusters::iterator it = clusters.begin(); it!=clusters.end(); )
    // {
    //     if (10 <= i++) it = clusters.erase(it);
    //     else {
    //         ++it;
    //     }
    // }

    // delete clusters that have too few points 
    vector<double*>::iterator ncm_it = ncm_cache.begin();
    for (Clusters::iterator it = clusters.begin();  // start at the beginning
         it != clusters.end();                      // stop at the end
         )                                          // no increment here
    {
        if (it->size() < n_min_clusterpoints)
        {
            ncm_it = ncm_cache.erase( ncm_it );
            it = clusters.erase(it);
        } 
        else
        {
            ++ncm_it;
            ++it; // increment as usual only if you dont delete the elem
        } 
    }

    cout << clusters.size() << " clusters found for scan " << identifier << endl;
    if ( cluster_outpath != "")
    {
        if (cluster_outpath[ cluster_outpath.size() - 1] != '/') 
            cluster_outpath += "/";
        writeClustersToScanfile(clusters, cluster_outpath);
    } 
}

void PlaneScan::mergeClusters(int c1, int c2)
{
    int n = clusters[c2].size();
    for (int i = 0; i < n; ++i) 
    {
        clusters[c1].push_back( clusters[c2].back() );
        clusters[c2].pop_back();
    }
    clusters.erase( clusters.begin() + c2 );
}

// Kinda neat debug function.
// Saves the result of Clustering to a scanfile. 
// Can be visualized with "show -f uos_rgb -c"
void PlaneScan::writeClustersToScanfile(Clusters& clusters, string path)
{
    ofstream ofile((path+"scan"+identifier+".3d").c_str(), ios_base::out);
    hsv color_hsv = {0, 1, 1};
    for (uint i = 0; i < clusters.size(); ++i)
    {
        rgb color_rgb = hsv2rgb( color_hsv );
        for(uint j = 0; j < clusters[i].size(); ++j)
        {
            ofile << points[clusters[i][j]][0] << " "
                << points[clusters[i][j]][1] << " "
                << points[clusters[i][j]][2] << " "
                << (int) (color_rgb.r*255) << " " 
                << (int) (color_rgb.g*255) << " " 
                << (int) (color_rgb.b*255) << endl;
        }
        color_hsv.h += 360.0 / (double)clusters.size(); 
    }
    // Just use 0 0 0 as pose.
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
                    if ( sqr(dist2Plane(points[i], transMat, planeList->at(j))) 
                        < sqr(dist2Plane(points[i], transMat, min)) )
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

// Find point-2-plane correspondences utilizing local sub-scan clusters. 
void PlaneScan::labelUseClusters()
{

    // TODO : use normalVariance for cluster adaption. Only available when read_normals=true
    // For Later Use: 
    // for (int i = 0; i < clusters.size(); ++i)
    // {
    //     double* var = normalVariance(clusters[i]);
    //     double std[] = 
    //     {
    //         sqrt(var[0]),
    //         sqrt(var[1]),
    //         sqrt(var[2])
    //     };
    //     cout    << "C" << i 
    //             << "[" << clusters[i].size() 
    //             << "] var=[" << var[0] << ", " << var[1] << ", " << var[2]
    //             << "] std=[" << std[0] << ", " << std[1] << ", " << std[2]
    //             << "]" 
    //             << endl;
    // }

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
        ///////////////////////////
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
        //////////////////////////

    // Look at each cluster
    for (int c = 0; c < nclusters; ++c)
    {
        // Get mean normal for the cluster
        double *ncm = normalClusterMean(clusters[c]);
        // cout << "ncm for c=" << c << ": " << ncm[0] << ", " << ncm[1] << ", " << ncm[2] << endl;
        // cout.flush();
        double alpha_min = DBL_MAX;
        int max_count = 0;
        int all_counts = 0;
        int p_alpha_min, p_max_count;
        // Search for a plane that has a similar normal and a big overlapping region 
        for (int p = 0; p < nplanes; ++p) 
        {
            // Skip planes that dont overlap with clusters
            if (cccount[c][p] != 0)
            {
                double alpha = angleBetweenNormals(ncm, allPlanes[p]->n);
                
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
                all_counts += cccount[c][p];
            }
        }

        // Fuzzy approach to put min. angle together with max. overlap
        double score_alpha = 1.0 - (alpha_min / M_PI_2);
        double score_overlap = (double)max_count / all_counts;
        // Fuzzed using max() function. TODO: think about fuzzing using multiplication.
        if (max_count != 0 && deg(alpha_min) < eps_similarity && score_overlap > score_alpha)
        {
            for (uint i = 0; i < clusters[c].size(); ++i) 
                correspondences.insert( Correspondence( points[clusters[c][i]], allPlanes[p_max_count] ) );
        } 
        // Optimally, this should be the case more often
        else if (max_count != 0 && deg(alpha_min) < eps_similarity && score_alpha >= score_overlap)
        {
            for (uint i = 0; i < clusters[c].size(); ++i) 
                correspondences.insert( Correspondence( points[clusters[c][i]], allPlanes[p_alpha_min] ) );
        } 
        // If the clustering didnt work so well, go for classical euklidian labels.
        // Not recommended.
        else if (use_correspondence_min ) labelDistEuklid(); 
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
 * 
 * TODO: add another approach. Direct Point-to-Plane normal correspondences
 */
void PlaneScan::labelPoints(bool quiet)
{
    if (!quiet) cout << "Labeling points on planes for scan" << identifier << endl;

    if (use_normal_cor) labelUseNormals();
    else if (use_clustering) labelUseClusters();
    else labelDistEuklid();
    if (!quiet)
    {
        int n = correspondences.size();  
        cout << "... " << n << " correspondence" << (n == 1 ? "":"s") << " found." << endl; 
    }
}

void PlaneScan::labelUseNormals()
{
    for (int i = 0; i < nrpts; ++i)
    {
        double min_alpha = DBL_MAX;
        int min_j;
        for (int j = 0; j < allPlanes.size(); ++j)
        {
            if ( dist2Plane(points[i], transMat, allPlanes[j]) < _eps_dist )
            {
                double alpha = angleBetweenNormals(normals[i], allPlanes[j]->n);
                if (alpha < min_alpha) 
                {
                    min_j = j;
                    min_alpha = alpha;
                }
            }
        }
        //cout << "Min angle between point " << i+1 << " and plane " << min_j << ": " << deg(min_alpha) << endl;
        if ( min_alpha != DBL_MAX && deg(min_alpha) < eps_similarity ) 
            correspondences.insert( Correspondence(points[i], allPlanes[min_j]) );
    }
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
	for (; it != correspondences.end() ; ++it ) 
    {
        sum += sqr(dist2Plane(it->first, transMat, it->second));
    }
	return sqrt(sum);
}

// static function, calculates total point-2-plane distance of all Scans
double PlaneScan::totalError()
{
	double sum = 0.0;
	for ( const auto& pscan : allPlaneScans) sum += pscan->calcErr();
	return sum;
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