/** @file 
 *  @brief this file contains the PlaneScan Class that represents a Scan 
 *  and holds the information about Point-to-Plane correspondences.
 *
 *  @author Fabian Arzberger, JMU, Germany.
 * 
 *  Released under the GPL version 3.
 */

#ifndef __PLANESCAN_H_
#define __PLANESCAN_H_

#include "slam6d/scan.h"
#include "slam6d/globals.icc"
#include "ioplanes.h"
#include "ANN/ANN.h"

#define EPS_POINT_2_PLANE_DIST 100 // distance in cm (?)

typedef std::pair<double*, NormalPlane*> Correspondence;
typedef std::vector< std::vector<int> > Clusters;

class PlaneScan 
{
public:

    static std::vector<PlaneScan*> allPlaneScans;
    static Planes allPlanes;

    ~PlaneScan();
    PlaneScan() {};
    PlaneScan(Scan*);
    PlaneScan(Scan*, Planes&);

    // for each point, store on which plane(s) it lies
    // good for efficient iteration when pairs are needed
    std::map<double*, NormalPlane*> correspondences;

    // Scan data (indexed)
    double **points;
    double **normals;
    int nrpts;
    double timestamp;
    double orig_transMat[16];
    // Pose Data
    double rPos[3];
    double rPosTheta[3];
    double transMat[16];
    // Disk data
    std::string basedir;
    std::string identifier;

    // Get point
    const double* operator()(int);

    bool isOkay();
    bool isEmpty();
    unsigned int index();

    static void setUseCorrespondenceMin(bool);
    static void setPlanes(Planes&);
    static void setEpsDist(double);
    static void setReduce(bool);
    static double totalError();
    static double dist2Plane(const double*, const double*, const NormalPlane*);
    static double dist2Plane(const double*, const NormalPlane*);
    // writes all the frame buffers to .frames files
    static bool writeFrames();
    // ensures that the .frames files have same length. this has to be done for animation
    static void fillFrames(); 
    static void setUseClustering(bool);
    static void setKNearest(int);
    static void setGrowthThresh(double);
    static void setEpsSimilarity(double);
    static void setMinClusterPoints(int);
    static void setClusterVisualizationPath(string);

    double calcErr();
    void addFrame(Scan::AlgoType); // adds the current transMat to the .frames file 
    string getFrameData();

    void calcNormals(DataXYZ&);
    /*
     * Label points to planes. 
     * This is initially done by the constructor, so you dont have to explicitly call the function
     * unless you want to update correspondences after transforming the scan.
     */
    void labelPoints(bool);
    void labelDistEuklid();
    void labelUseClusters();

private:
    /*
    * The stringstream sout buffers the frame file. It will be written to disk at
    * once at the end of the program. This reduces file operations and saves time.
    */
    stringstream sout;
    bool empty = false;
    unsigned int idx;

    Clusters clusters;

    // Instance helper functions for local clustering
    double * normalClusterMean(vector<int>&);
    double * normalVariance(ANNidxArray, int);
    double clusterPointDist(vector<int>&, double*);
    void growRegionRecursive(int, ANNpointArray&, bool*);
    void calcLocalClusters();

    // Use for debug only.
    void writeClustersToScanfile(Clusters&, string path);

    static double _eps_dist;
    static unsigned int idx_global;
    static bool reduce;
    static bool use_correspondence_min;
    static bool use_clustering;
    static int k_neighbours;
    static double d_growth;
    static int n_min_clusterpoints;
    static double eps_similarity;
    static string cluster_outpath;

    // Private static helper functions
    static bool inline isInPlane(double*, double* trans, const NormalPlane*);
    static bool inline similar(double*, double*);
    
};

typedef std::vector<PlaneScan*> PlaneScans;

#endif //__PLANESCAN_H_