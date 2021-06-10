/**
 * @file
 * @brief Program to cluter 3d points based on their planarity.
 * Exports clusters to be read later by preg6d.
 * 
 * @author Fabian Arzberger, JMU, Germany
 * 
 * Released under GPL version 3. 
 */

#include "cluster.h"

int main(int argc, char **argv)
{
    string scandir;
    IOType type;
    bool quiet;
    int maxDist;
    int minDist;
    int octree;
    double red;
    int start;
    int end;
    int k_nearest;
    int n_threads;
    double d_growth;
    int n_min_clusterpoints;
    double eps_alpha_similarity;
    bool scanserver = false;
    parse_options(  argc, argv, scanserver, scandir, type, quiet, maxDist,
                    minDist, octree, red, start, end, k_nearest, n_threads,
                    d_growth, n_min_clusterpoints, eps_alpha_similarity);
    
    // We use a PlaneScan object to do clustering 

    // This is a cluster program. So that one should be obvious.
    PlaneScan::setReadClusters(false);
    PlaneScan::setUseClustering(true); 
    PlaneScan::setKNearest(k_nearest);
    PlaneScan::setReduce( red != -1);
    PlaneScan::setGrowthThresh(d_growth);
    PlaneScan::setEpsSimilarity(eps_alpha_similarity);
    PlaneScan::setMinClusterPoints(n_min_clusterpoints);
    if (type == IOType::UOS_NORMAL) {
        PlaneScan::setReadNormals(true);
        //omp_set_num_threads(n_threads);
        cout << "Using normals." << endl;
        cout << "Using " << n_threads << endl;
    } 
    else {
        std::cout << "Using " << k_nearest << " nearest neighbours for normal calculation." << endl;
        std::cout << "Multithreading requires precalculated normals. Use bin/calc_normals to export into uos_normal format." << endl;
    }

    // Read the scans and setup save directory

    Scan::openDirectory(scanserver, scandir, type, start, end);
    readFramesAndTransform( scandir, start, end, -1, true, red > -1);
    std::string save_dir = scandir + "clusters/";
    if ( !existsDir( save_dir.c_str() ) ) 
    {
        boost::filesystem::create_directory(save_dir);
        cout << "Creating \"" << save_dir << "\"." << endl;
    } 
    else cout << save_dir << " exists allready." << endl;
    PlaneScan::setClusterVisualizationPath( save_dir );

    // Actually setting up the planescan objects now
    //omp_set_num_threads(n_threads);
    PlaneScan *ps;
    {
        #pragma omp parallel for num_threads(n_threads)
        for( unsigned int k = 0; k < Scan::allScans.size() ; ++k)
        {   
            Scan *scan = Scan::allScans.at(k);
            scan->setRangeFilter( maxDist, minDist ); 
            scan->setReductionParameter( red, octree ); 
            // Finds point-2-plane correspondences
            ps = new PlaneScan( scan ); 
        }  
    }
}