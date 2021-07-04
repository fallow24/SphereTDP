#include <iostream>
#include <fstream>
#include <string> 
#include <math.h>
#include <sstream>

using namespace std;

void init(int arr[], int n)
{
    for (int i = 0; i < n; i++)
        arr[i] = 0;
}

int main(int argc, char * argv[])
{
    string line, token;
    string infile = "scan001.3d";
    string outfile = "hist.txt";
    double MAX_DIST=100.0;
    double STEP=0.1;

    if(argc > 1) infile = argv[1];
    else cout << "Usage: " << argv[0] << " {infile outfile maxdist step} \n Continuing with defaul parameters." << endl;
    if(argc > 2) outfile = argv[2];
    if(argc > 3) MAX_DIST = atof(argv[3]);
    if(argc > 4) STEP = atof(argv[4]);
    ifstream file(infile);
    ofstream hisfile(outfile);

    int bins = (MAX_DIST/STEP) + 1;
    int count[bins];
    init(count, bins);
   
    int pointcount = 0;
    if (file.is_open())
    {
        // sum up all data
        cout << "Creating data ..." << infile << endl;
        cout.flush();
        double x, y, z, distance;

        while(file.good()) {
          file >> x;
          file >> y;
          file >> z;
          file >> distance;
          int index = (int)(distance / STEP);
          count[min(index, bins-1)]++;
          pointcount++;
        }

        //for(int i = 0; i < bins; i++) count[i] /=pointcount;
        cout << "Writing data ..." << outfile << endl;
        cout.flush();

        int pointsum = 0;
        double mean = 0;
      	bool p90 = false;
        bool p95 = false;
        bool p98 = false;
        for(int i = 0; i < bins; i++) {
          hisfile << ((i * STEP) + STEP) << " " << (double)count[i] << endl;
          pointsum += count[i];
          if(!p90) {
            if(pointsum > 0.9*pointcount) {
              cout << "P90: " << ((i * STEP) + STEP) << endl;
              p90 = true;
            }
          } if(!p95) {
            if(pointsum > 0.95*pointcount) {
              cout << "P95: " << ((i * STEP) + STEP) << endl;
              p95 = true;
            }
          } if(!p98) {
            if(pointsum > 0.98*pointcount) {
              cout << "P98: " << ((i * STEP) + STEP) << endl;
              p98 = true;
            }
          }

    	    mean += i*STEP*count[i];
        }

	mean = mean/pointcount;

	double var = 0;
	for(int i = 0; i < bins; i++)
	{
	    var += count[i]*(i*STEP - mean)*(i*STEP - mean);
	}
	double std = sqrt(var)/pointcount;

	cout << "Mean: " << mean << " Std: " << std << endl;

        hisfile << endl;
        hisfile.flush();
        hisfile.close();
        cout << " done." << endl;
        cout.flush();
    } else {
      cout << infile << " could not be found!" << endl;
    }
    return 0;
}

