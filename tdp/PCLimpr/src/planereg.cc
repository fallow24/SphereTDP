/** @file 
 *  @brief Point-to-plane based post registration correction algorithm. 
 *  Requires globaly registered scan archive and extracted planes.
 * 
 *  You can use bin/slam6d or bin/correction for registration.
 *  Use bin/planes to extract planes from a scan.
 *  To extract planes from the globaly registered scan, you have
 *  to condense the scan archive into one globaly consistent .3d scan-file.
 *  Use bin/condense for that.
 *
 *  @author Fabian Arzberger, JMU, Germany.
 * 
 *  Released under the GPL version 3.
 */

#include "planescan.h"
#include "optimizer.h"
#include "newmat/newmatio.h"
#include "scanio/framesreader.h"
#include <string>
#include <omp.h>

using namespace std;

#include "slam6d/scan.h"
#include "slam6d/io_types.h"
#include "ioplanes.h"
#include "planescan.h"
#include "optimizer.h"

#include <boost/program_options.hpp>
namespace po = boost::program_options;

// Boost needs to convert from string to IOType. Use this validation function.
void validate(boost::any& v, const std::vector<std::string>& values,
              IOType*, int) {
  if (values.size() == 0)
    throw std::runtime_error("Invalid model specification");
  std::string arg = values.at(0);
  try {
    v = formatname_to_io_type(arg.c_str());
  } catch (...) { // runtime_error
    throw std::runtime_error("Format " + arg + " unknown.");
  }
}

Dimensions formatname_to_dimensions(const char* str)
{
    if (strcasecmp(str, "all") == 0) return ALL;
    else if (strcasecmp(str, "rolling") == 0) return ROLLING;
    else if (strcasecmp(str, "descending") == 0) return DESCENDING;
    else if (strcasecmp(str, "rotating") == 0) return ROTATING;
    else throw std::runtime_error(string("Invalid Dimension type."));
}

// Same for Dimensions enum
void validate(boost::any& v, const std::vector<std::string>& values,
              Dimensions*, int) {
  if (values.size() == 0)
    throw std::runtime_error("Invalid model specification");
  std::string arg = values.at(0);
  try {
    v = formatname_to_dimensions(arg.c_str());
  } catch (...) { // runtime_error
    throw std::runtime_error("Format " + arg + " unknown.");
  }
}

/*
 * Use boost to set and parse command line options.
 */ 
int parse_options(  int argc, 
                    char** argv, 
                    string &scandir, 
                    string &planedir, 
                    IOType &type, 
                    bool &quiet, 
                    int &maxDist, 
                    int &minDist, 
                    int &octree, 
                    double &red,
                    bool &scanserver,
                    double &eps_dist,
                    int& start,
                    int& end,
                    int& n_iter,
                    double& eps_crit,
                    double& alpha,
                    double& rPos_alpha_scale,
                    bool& anim,
                    int& nthreads,
                    Dimensions& dims,
                    bool& autoAlpha,
                    int& updateCor,
                    bool& continuous,
                    bool& use_min_cor,
                    bool& use_frames,
                    int& k_nearest,
                    bool& use_clustering,
                    double& d_growth,
                    int& n_min_clusterpoints, 
                    double& eps_norm_similarity,
                    string& clusters_outputpath,
                    bool& use_normal_cor)
{
    // Add program option descriptions and reference variables accordingly
    po::options_description generic("Generic options");
    po::options_description input("Program options");
    po::options_description hidden("Hidden options");

    // Add program options in a curried way, using the overloaded ()-operator
    generic.add_options()
        ("help,h", "Display a very helpful message");
    input.add_options()
        ("format,f", po::value<IOType>(&type)->default_value(UOS, "uos"),
            "chose input format from {uos, uosr, uos_rgb, ous_normal, old, xyz}")
        ("start,s", po::value<int>(&start)->default_value(0),
            "Skip the first <arg> scans.")
        ("end,e", po::value<int>(&end)->default_value(-1),
            "Stop at scan index <arg>")
        ("planes,p", po::value<string>(&planedir)->default_value("./planes/"),
            "Directory of the globaly consistent planes (either convex hull or normal representation)")
        ("quiet,q", po::bool_switch(&quiet)->default_value(false),
            "Supress any output (except segfault) of the program.")
        ("anim", po::bool_switch(&anim)->default_value(false),
            "If set, store animation in .frames files. Can be visualized with bin/show")
        ("max,m", po::value<int>(&maxDist)->default_value(-1),
            "neglegt all data points with a distance larger than <arg> 'units'")
        ("min,M", po::value<int>(&minDist)->default_value(-1),
            "neglegt all data points with a distance smaller than <arg> 'units'")
        ("reduce,r", po::value<double>(&red)->default_value(-1.0),
            "turns on octree based point reduction (voxel size= <arg>)")
        ("octree,O", po::value<int>(&octree)->default_value(0),
            "Use randomized octree based point reduction (pts per voxel=<arg>)")
        ("scanserver,S", po::value<bool>(&scanserver)->default_value(false),
            "Use the scanserver as an input method and handling of scan data")
        ("eps_dist,d", po::value<double>(&eps_dist)->default_value(EPS_POINT_2_PLANE_DIST),
            "Maximum point-2-plane distance threshold for correspondence.")
        ("iter,i", po::value<int>(&n_iter)->default_value(-1),
            "Number <arg> of maximum gradient descent iterations before updating correspondences.")
        ("eps_crit,E", po::value<double>(&eps_crit)->default_value(-1),
            "Quickstop gradient descent if convergence falls below <arg> threshold.")
        ("alpha,a", po::value<double>(&alpha)->default_value(ALPHA),
            "Gradient descent factor / convergence rate. Use carefully.")
        ("pos_scale_alpha,c", po::value<double>(&rPos_alpha_scale)->default_value(RPOS_ALPHA_SCALE),
            "Scales the <alpha> value applied to the robots position about <arg>.")
        ("nthreads,t", po::value<int>(&nthreads)->default_value(1),
            "Use <arg> threads for gradient descent. Each thread minimizes one sub-scan.")
        ("dimensions,D", po::value<Dimensions>(&dims)->default_value(ALL, "all"),
            "Specifies which dimensions should be optimized. Chose from {all, rolling, descending, rotating}.")
        ("auto", po::bool_switch(&autoAlpha)->default_value(false),
            "Auto-detect optimum alpha to initialize AdaDelta. Convergence will be guaranteed to be fast. However, use with care as sometimes smaller steps on the gradient are prefered.")
        ("update_correspondences,k", po::value<int>(&updateCor)->default_value(10),
            "Update correspondences <arg> times. Total number of iterations will therefor be i*k")
        ("continuous,C", po::bool_switch(&continuous)->default_value(false),
            "Do not parallelize iterations. Instead, apply the last iteration pose change onto the next scan.")
        ("use_min_cor", po::bool_switch(&use_min_cor)->default_value(false),
            "Use minimum distance to plane for correspondence in case of ambiguity.")
        ("use_frames", po::bool_switch(&use_frames)->default_value(false),
            "Use pose specified in .frames file instead of .pose file.")
        ("use_normal_cor", po::bool_switch(&use_normal_cor)->default_value(false),
            "Use direct normals for better point-2-plane correspondences. Uses k nearest neighbours for normal calculation. Or you can read normals using -f uos_normal")
        ("use_clustering", po::bool_switch(&use_clustering)->default_value(false),
            "Use clustering for better point-2-plane correspondences. Utilizes local normal calculation using k-nearest neighbours.")
        ("k_nearest,K", po::value(&k_nearest)->default_value(20),
            "Use <arg> nearest neighbours in local normal calculation.")
        ("d_growth,g", po::value(&d_growth)->default_value(5.0),
            "Threshold for cluster region growing. Stop region growing if there is no new point nearer than <arg>.")
        ("min_cluster_size,z", po::value(&n_min_clusterpoints)->default_value(100),
            "Do not consider a cluster unless it has more then <arg> points in it.")
        ("eps_similarity,y", po::value(&eps_norm_similarity)->default_value(5),
            "Two normals are considered similar if their angle is smaller then <arg> in degrees.")
        ("visualize_clusters,v", po::value(&clusters_outputpath)->default_value(""),
            (string("When used, the program writes the results of the local normal clustering ")+ 
            string("to the specified path in <arg>. You can visualize the clusters with nice colors as follows:\n")+
            string("bin/show -c -f uos_rgb /your/dir/ -s 0")).c_str());
    hidden.add_options()
        ("input-dir", po::value<string>(&scandir), "input dir");

    // All options together
    po::options_description alloptions;
    alloptions.add(generic).add(input).add(hidden);

    // Only commandline, visible with --help
    po::options_description cmdoptions;
    cmdoptions.add(generic).add(input);

    // positional argument for input directory 
    po::positional_options_description pos;
    pos.add("input-dir", 1); // max 1 pos arg

    // Map and store option inputs to variables
    po::variables_map vars;
    po::store( po::command_line_parser(argc, argv).
                options(alloptions).
                positional(pos).
                run(), 
                vars);

    // help display msg
    if ( vars.count("help") )
    {
        cout << cmdoptions;
        cout << endl << "Example usage:" << endl 
            << "\t bin/preg6d dat/ -f uos —planes dat/planes/ —reduce 100 —octree 10 -d 100 -i 50 -k 50 -a 0.01 -c 10 -t 20 —dimensions rolling —anim —max 1500" 
            << endl;
        exit(0);
    }
    po::notify(vars);

    // Add trailing directory slash if there is none. Works differently when compiling under Windows
#ifndef _MSC_VER
    if (scandir[ scandir.length()-1 ] != '/') scandir = scandir + "/";
    if (planedir[ planedir.length()-1 ] != '/') planedir = planedir + "/";
#else
    if (scandir[ scandir.length()-1]  != '\\') scandir = scandir + "\\";
    if (planedir[ planedir.length()-1]  != '\\') planedir = planedir + "\\";
#endif

    // return with success exit code
    return 0;
}

int main(int argc, char **argv)
{
    // Parse program options first:
    string scandir;
    int startScan = 0;
    int endScan = -1;
    string planedir; 
    IOType type = UOS;
    bool quiet = false;
    int maxDist = -1, minDist = -1;
    int octree = 0;
    double red = -1.0;
    bool scanserver = false;
    double eps_dist = EPS_POINT_2_PLANE_DIST;
    int n_iter = -1;
    double eps_crit = -1;
    double alpha = 0.001;
    double rPos_alpha_scale = 100;
    bool anim = false;
    int nthreads = 1;
    Dimensions dims;
    bool autoAlpha = false;
    int updateCor = 10;
    bool continuous = false;
    bool use_min_cor = false;
    bool use_frames = false;
    int k_nearest = 20;
    bool use_clustering = false;
    double d_growth = 5.0;
    int n_min_clusterpoints = 100;
    double eps_norm_similarity = 5;
    string cluster_output_path = "";
    bool use_normal_cor = false;

    parse_options(argc, argv, scandir, planedir, type, 
        quiet, maxDist, minDist, octree, red, scanserver,
        eps_dist, startScan, endScan, n_iter, eps_crit, alpha,
        rPos_alpha_scale, anim, nthreads, dims, autoAlpha, updateCor,
        continuous, use_min_cor, use_frames, k_nearest, use_clustering,
        d_growth, n_min_clusterpoints, eps_norm_similarity, cluster_output_path,
        use_normal_cor);
    
    omp_set_num_threads(nthreads);

    // Open and setup all planes 
    PlaneIO::read( planedir );
    std::cout << "Read " << PlaneIO::allPlanes.size() << " planes:" << std::endl;
    {
        Planes::iterator plane = PlaneIO::allPlanes.begin();
        Planes::iterator end = PlaneIO::allPlanes.end();
        for (; plane != end; ++plane)
        {
            std::cout << "Plane normal ["
                << (*plane)->n[0] << ", "
                << (*plane)->n[1] << ", "
                << (*plane)->n[2] << "]" << std::endl;
        }
    }


    // Output of given command line arguments for optimization
    if (n_iter != -1) {
        std::cout << "Using maximum " << n_iter << " iterations." << std::endl;
        Optimizer::setMaxIter(n_iter);
    } 
    if (eps_crit != -1) {
        std::cout << "Quickstop if convergence < " << eps_crit << std::endl;
        Optimizer::setEpsConvergence(eps_crit);
    }
    if (!autoAlpha) {
        Optimizer::setAuto( false );
        std::cout << "Using alpha = " << alpha << std::endl;
        Optimizer::setAlphaInit( alpha );
    } else {
        std::cout << "Auto detecting initial alpha." << std::endl;
        Optimizer::setAuto( true );
    }
    std::cout << "Scaling alpha applied to position about " << rPos_alpha_scale << std::endl;  
    Optimizer::setRPosAlphaScale( rPos_alpha_scale );
    Optimizer::setUpdateCor( updateCor ); // k iteration value 
    if (quiet) {
        std::cout << "Quiet mode." << std::endl;
        Optimizer::setQuiet(true);
    }
    if (anim) {
        std::cout << "Storing animation in .frames." << std::endl;
        Optimizer::setAnim(true);
    }
    std::cout << "Updating correspondences " << updateCor << " times." << std::endl;
    if (use_normal_cor && use_clustering) 
    {
        cout << "Both normals and clustering correspondences are activated.";
        cout << " Please deactivate one of them. You cannot use both of them at the same time.";
        cout << endl;
        return 1;
    }
    PlaneScan::setUseNormalCor( use_normal_cor );
    PlaneScan::setUseClustering( use_clustering );
    PlaneScan::setKNearest( k_nearest );
    PlaneScan::setGrowthThresh( d_growth );
    PlaneScan::setEpsSimilarity( eps_norm_similarity );
    PlaneScan::setMinClusterPoints( n_min_clusterpoints );
    
    // You can read normals to save time during clustering.
    if (type == IOType::UOS_NORMAL) {
        PlaneScan::setReadNormals(true);
        cout << "Using normals." << endl;
    } 
    // You can read entire clusters 
    else if (type == IOType::UOS_RGB) {
        PlaneScan::setReadClusters(true);
        cout << "Reading clusters." << endl;
    }
    else if (use_clustering) {
        std::cout << "Using " << k_nearest << " nearest neighbours for normal calculation." << endl;
    }

    // Open and pre-process all scans
    Scan::openDirectory(scanserver, scandir, type, startScan, endScan);
    // Very very important 2 lines below!!
    bool use_pose = !use_frames;
    readFramesAndTransform( scandir, startScan, endScan, -1, use_pose, red > -1);
    // Converting the Scan objects into PlaneScan objects
    PlaneScan::setEpsDist( eps_dist );
    PlaneScan::setReduce( red != -1 );
    PlaneScan::setPlanes( PlaneIO::allPlanes );
    PlaneScan::setUseCorrespondenceMin( use_min_cor );
    PlaneScan::setClusterVisualizationPath( cluster_output_path );
    PlaneScan *ps;
    {
        for( unsigned int k = 0; k < Scan::allScans.size() ; ++k)
        {   
            Scan *scan = Scan::allScans.at(k);
            scan->setRangeFilter( maxDist, minDist ); 
            scan->setReductionParameter( red, octree ); 
            // Finds point-2-plane correspondences
            ps = new PlaneScan( scan ); 
            if (!continuous) ps->labelPoints(quiet);
        }  
    }

    // Iterate all the scans. Transformations are buffered between iterations
    if (continuous)
    {
        ColumnVector X(6); 
        ColumnVector dX(6);
        dX << 0 << 0 << 0 << 0 << 0 << 0;
        for ( unsigned int k = 0; k < PlaneScan::allPlaneScans.size(); ++k)
        {
            // Finds point-2-plane correspondences
            cout << "Optimizing Scan " << k << endl;
            PlaneScan *pScan = PlaneScan::allPlaneScans.at(k);
            Optimizer iter(pScan, dims);
            X = iter.getResult();
            iter.setState( X + dX );
            iter.relabel();
            iter(); // do the iterations
            dX = iter.getResult() - X; 
            pScan->addFrame( Scan::ICPINACTIVE );
        }
    }
    else 
    {
        #pragma omp parallel for num_threads(nthreads)
        for ( unsigned int k = 0; k < PlaneScan::allPlaneScans.size(); ++k)
        {
            // Finds point-2-plane correspondences
            cout << "Optimizing Scan " << k << endl;
            PlaneScan *pScan = PlaneScan::allPlaneScans.at(k);
            Optimizer iter(pScan, dims);
            iter(); // do the iterations
            iter.getResult();
            pScan->addFrame( Scan::ICPINACTIVE );
        }
    }

    cout << "Writing transformations to .frames..." << endl;
    // In order for the animation to work, all .frames files must have same length
    PlaneScan::fillFrames();
    // Write .frames files to disk from the buffered transformations
    PlaneScan::writeFrames();
    
    return 0;
}
