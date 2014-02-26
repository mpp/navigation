/*
 * RANSAC.h
 *
 *  Created on: Mar 4, 2013
 *      Author: vivanchenko
 *
 *      Description: line is represented by equation
 *      a*x+b*y=d, where a=cos(alpha), b=sin(alpha), alpha - the angle
 *      between a horizontal axis and a line normal, and  d is the distance
 *      from the origin to the line;
 *
 *      Representing a line in such a way is well suited for fitting error
 *      evaluation and allows to avoid some marginal cases that arise from
 *      a traditional line representation y=ax+b
 */

#ifndef RANSAC_H
#define RANSAC_H


#include <string.h>
#include <assert.h>
#include <stdio.h> // NULL symbol
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define SQR(a) ((a)*(a))
#define MAX_FLOAT (std::numeric_limits<float>::digits)
#define LARGE_NUMBER  (MAX_FLOAT/2)
#define SMALL_NUMBER  (1.0F/LARGE_NUMBER)
#define SIGN(x) ((x)>0?1:(-1))

// unit test params
const int NDATA_UNIT_TEST = 100;
const float TRUE_PARAM_UNIT_TEST[2] = {0.9f, -27.0f};
const float NOISE_LEVEL_UNIT_TEST = 1.0f;
const float OUTLIER_PROPORTION_UNIT_TEST = 0.3f;
const float SHIFT_OUTLIERS_UNIT_TEST = 10*NOISE_LEVEL_UNIT_TEST; // shift introduced by outliers
const int UNIT_TEST_IMG_WIDTH = 800;
const int UNIT_TEST_IMG_HEIGHT = 800;

// conversion to string for many types
template <class T>
string toString(T a);

// outputs redctangle that includes the data
cv::Rect dataRange(float* x, float* y, const int N);

// simple linear transformation
inline float linear(float src, float mult_val, float plus_val);

// shows points, marks inliers, draws a fit line (Y points up);
void showPoints(Mat& image, float* x, float* y, const int N,
        bool* inlrs, float* param);

// Root mean sqaure error (RMSE) of line fit y=param[0]*x+param[1]
float errorLine(float* x, float* y, int N, float* param,
        const bool* inliers = NULL);

// simple line fit using sum of squared differences
// inliers are used to specify a subset of points for a fit
float fitLine(float* x, float* y, int N, float* param,
        const bool* inliers = NULL, const bool debug = false);

// RANSAC line fit (number of inliers has priority over RMSE).
float RANSAC_line(float* x, float* y, const int N, float* param,
        const int niter = 10, const float maxError = 1.0f,
        bool* inlrs = NULL, bool debug = false) ;

// generates the data give the random seed
void generateData(float* x, float* y, unsigned int seed = 0, bool hasOutl = false);

// generates the data from pasted numbers
void generateDataPaste(float* x, float* y);

// unit test of quadratic line fit
bool uniTest_fitLine_QUADRATIC(int col = 0, int row = 0);

// unit test of RANSAC line fit
bool uniTest_fitLine_RANSAC(int col = 0, int row = 0);

void paramToVect(float* param, Point2f& normal, float* D);

void paramToPointVect(const float* param, Vec4f &linePointVect);

/*
// RMSE for inliers
float RMSE_Line(const vector<Point2f>& xy, float* param,
        const vector<bool>& inliers) {

    int N = xy.size();
    if (N==0)
        return LARGE_NUMBER;
    int ninliers = 0;
    double RMSE = 0;

    // convert param into the normal and D
    Point2f normal;
    float D;
    paramToVect(param, normal, &D);

    // error accumulation loop
    for (int i = 0; i < N; i++) {
        if (inliers[i]) {
            ninliers++;
            RMSE += SQR(distLine(xy[i], normal, D));
        }
    }

    if (ninliers==0)
        return LARGE_NUMBER;
    else
        return sqrt(RMSE/ninliers);
}*/

#endif // RANSAC_H
