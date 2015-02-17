#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

float getScale() 
{

    std::string label;
    int line, x, y;
    float depth, realDepth;
    float scale = 1;

    double sum1 = 0, sum2 = 0;

    std::fstream associationfile;
    associationfile.open("/home/worxli/Datasets/data/depthunscaled.txt",std::ios::in);

    if(!associationfile.is_open())
    {
        fprintf(stderr,"\nERROR: Could not open File ~/Datasets/data/depthunscaled.txt\n");
    } else {
        fprintf(stderr,"\nReading depth File\n");

        while(!associationfile.eof())
        {
            std::string temp("");

            getline(associationfile,temp);

            std::stringstream stream(temp);

            stream >> label;
            stream >> line; 

            std::stringstream name;
            name << "/home/worxli/Datasets/data/depth/mapped" << line << ".png";
            Mat img = cv::imread(name.str(), 0);

            cout << name.str() << endl;

            while(stream >> x != NULL) 
            {

                stream >> y;
                stream >> depth;

                uint16_t realDepth = img.at<uint16_t>(y, x);

                if(realDepth>0) {
                    sum1 = sum1 + (double) ((float) realDepth * depth);
                    sum2 = sum2 + (double) depth * depth;
                }

                // cout << "line: " << line << " x " << x << " y " << y << " depth " << depth << " realDepth " << realDepth << endl;
            }

        }

        scale = sum1/sum2;

        // cout << sum1 << endl;
        // cout << sum2 << endl;
        cout << "scale " << scale << " to meters: " << scale/1000 <<  endl;
  }

  return scale;
}

void unscaleAssociate(float scale) 
{
    std::fstream associationfile;
    associationfile.open("/home/worxli/Datasets/data/associate_unscaled.txt",std::ios::in);

    ofstream outfile;
    outfile.open ("/home/worxli/Datasets/data/associate.txt");

    if(!associationfile.is_open())
    {
        fprintf(stderr,"\nERROR: Could not open File\n");
    } else {
        fprintf(stderr,"\nReading Association unscaled File\n");

        std::string depthname; std::string rgbname;
        float q1, q2, q3, q4, translation1, translation2, translation3;

        while(!associationfile.eof())
        {
            std::string temp("");
            getline(associationfile,temp);

            std::stringstream stream(temp);

            stream >> translation1; stream >> translation2; stream >> translation3;
            stream >> q1; stream >> q2; stream >> q3; stream >> q4;
            stream >> depthname;
            stream >> rgbname;

            outfile << translation1 * scale << " "
                << translation2 * scale << " "
                << translation3 * scale << " "
                << q1  << " "  
                << q2  << " " 
                << q3  << " " 
                << q4  << " " 
                << depthname << " "
                << rgbname << "\n";


        }

        outfile.close();
        cout << "wrote associate file" << endl;
  }
}

int main(int argc, char** argv)
{

    if ( argc != 2 ) {
        float scale = getScale();
        unscaleAssociate(scale/1000);
    } else {
        unscaleAssociate(std::stof(argv[1]));
    }

    return 0;
}